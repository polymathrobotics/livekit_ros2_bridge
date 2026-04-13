// Copyright (c) 2025-present Polymath Robotics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rpc_router.hpp"

#include <array>
#include <chrono>
#include <exception>
#include <future>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>

#include "interface_definition_lookup.hpp"
#include "payloads/interface_payloads.hpp"
#include "payloads/resource_list_payloads.hpp"
#include "payloads/service_call_payloads.hpp"
#include "protocol.hpp"
#include "rclcpp/logging.hpp"
#include "ros_executor_queue.hpp"
#include "ros_service_caller.hpp"
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge
{

namespace
{

const auto kRpcRouterLogger = rclcpp::get_logger("rpc_router");
using ResourcesByName = std::map<std::string, std::vector<std::string>>;

struct RpcMethod
{
  const char * name;
};

constexpr RpcMethod kServiceCallRpcMethod{
  protocol::kRpcServiceCall,
};
constexpr RpcMethod kInterfacesGetRpcMethod{
  protocol::kRpcInterfacesGet,
};
constexpr RpcMethod kServicesListRpcMethod{
  protocol::kRpcServicesList,
};
constexpr RpcMethod kTopicsListRpcMethod{
  protocol::kRpcTopicsList,
};
constexpr std::array<const char *, 4> kRpcMethodNames{
  kServiceCallRpcMethod.name,
  kInterfacesGetRpcMethod.name,
  kServicesListRpcMethod.name,
  kTopicsListRpcMethod.name,
};

LogEvent & addLogFields(LogEvent & event, const RpcMethod & rpc_method, const RpcInvocation & invocation)
{
  // Keep the request-scoped correlation fields uniform across every router
  // rejection/failure log so operators can trace one RPC through the bridge.
  event.field("method", rpc_method.name)
    .fieldOr("request_id", invocation.request_id)
    .fieldOr("requester_identity", invocation.caller_identity);
  return event;
}

std::uint32_t errorCodeFor(const std::exception & exc)
{
  // Keep the wire-level error taxonomy narrow: explicit RpcHandlerError values
  // win, payload/range validation faults become invalid_request, and every
  // other exception is collapsed into the bridge's generic internal error.
  if (const auto * rpc_handler_error = dynamic_cast<const RpcHandlerError *>(&exc)) {
    return rpc_handler_error->code();
  }
  if (
    dynamic_cast<const std::invalid_argument *>(&exc) != nullptr ||
    dynamic_cast<const std::out_of_range *>(&exc) != nullptr)
  {
    return protocol::kRpcErrorInvalidRequest;
  }
  return protocol::kRpcErrorInternal;
}

const char * errorReason(std::uint32_t code)
{
  switch (code) {
    case protocol::kRpcErrorInvalidRequest:
      return "invalid_request";
    case protocol::kRpcErrorUnauthorized:
      return "unauthorized";
    case protocol::kRpcErrorForbidden:
      return "forbidden";
    case protocol::kRpcErrorInternal:
    default:
      return "internal";
  }
}

[[noreturn]] void throwLoggedError(
  const RpcMethod & rpc_method,
  const RpcInvocation & invocation,
  const std::exception & exc,
  std::optional<std::string_view> service = std::nullopt)
{
  // Log once before translating arbitrary exceptions into the stable RPC error
  // surface. Pre-built RpcHandlerError instances are rethrown after logging so
  // method-specific codes and messages survive unchanged.
  const auto code = errorCodeFor(exc);
  const char * reason = errorReason(code);
  LogEvent event(kRpcRouterLogger, code == protocol::kRpcErrorInternal ? "rpc_request_failed" : "rpc_request_rejected");
  addLogFields(event, rpc_method, invocation).field("reason", reason);
  if (const auto request_field = interface_payloads::invalidRequestField(exc)) {
    event.field("request_field", *request_field);
  } else if (const auto request_field = service_call_payloads::invalidRequestField(exc)) {
    event.field("request_field", *request_field);
  } else if (const auto request_field = resource_list_payloads::invalidRequestField(exc)) {
    event.field("request_field", *request_field);
  }

  if (service) {
    event.field("service", *service);
  }
  event.field("error", exc.what());

  if (code == protocol::kRpcErrorInternal) {
    event.error();
  } else {
    event.warn();
  }

  if (dynamic_cast<const RpcHandlerError *>(&exc) != nullptr) {
    throw;
  }
  throw RpcHandlerError(code, exc.what());
}

std::vector<ResourceListEntry> filterResourceEntries(
  const ResourcesByName & resources,
  const AccessPolicy & access_policy,
  AccessOperation operation,
  const ResourceListRequest & request)
{
  std::vector<ResourceListEntry> entries;

  for (const auto & [name, types] : resources) {
    // The RPC schema exposes a single interface type per entry, so resources
    // with multiple ROS types are omitted instead of guessed or duplicated.
    if (types.size() != 1U) {
      continue;
    }
    if (!access_policy.allows(operation, name)) {
      continue;
    }
    const auto & type = types.front();
    if (request.query) {
      const auto & query = *request.query;
      if (name.find(query) == std::string::npos && type.find(query) == std::string::npos) {
        continue;
      }
    }

    entries.push_back({name, type});
    // Apply the page limit after policy/query filtering so denied resources do
    // not consume caller-visible capacity.
    if (request.limit && entries.size() >= *request.limit) {
      break;
    }
  }

  return entries;
}

template <typename HandleRpcT>
std::optional<std::string> withRequiredCallerIdentity(
  const RpcMethod & rpc_method, const RpcInvocation & invocation, HandleRpcT handle_rpc)
{
  // Reject anonymous callers before parsing/dispatch, and centralize shared
  // exception-to-protocol translation for every RPC handler.
  if (invocation.caller_identity.empty()) {
    LogEvent event(kRpcRouterLogger, "rpc_request_rejected");
    addLogFields(event, rpc_method, invocation)
      .field("reason", "unauthorized")
      .field("error", "caller_identity_required");
    event.warn();
    throw RpcHandlerError(protocol::kRpcErrorUnauthorized, "caller_identity is required for this RPC");
  }

  try {
    return handle_rpc();
  } catch (const RpcHandlerError &) {
    throw;
  } catch (const std::exception & exc) {
    throwLoggedError(rpc_method, invocation, exc);
  }
}

}  // namespace

RpcRouter::RpcRouter(
  rclcpp::Node & node,
  const AccessPolicy & access_policy,
  RosExecutorQueue & ros_executor_queue,
  RosServiceCaller & ros_service_caller)
: node_(node)
, access_policy_(access_policy)
, ros_executor_queue_(ros_executor_queue)
, ros_service_caller_(ros_service_caller)
{}

bool RpcRouter::registerRpcs(RoomConnection & room_connection)
{
  bool all_registered = true;
  const auto register_method = [&](const char * method_name, RpcHandler handler) {
    // Registration is best-effort rather than transactional so one failure
    // does not hide other methods that can still be served on this connection.
    all_registered = room_connection.registerRpc(method_name, std::move(handler)) && all_registered;
  };

  register_method(
    kServiceCallRpcMethod.name, [this](const RpcInvocation & invocation) { return callService(invocation); });
  register_method(
    kInterfacesGetRpcMethod.name, [this](const RpcInvocation & invocation) { return getInterfaces(invocation); });
  register_method(
    kServicesListRpcMethod.name, [this](const RpcInvocation & invocation) { return listServices(invocation); });
  register_method(
    kTopicsListRpcMethod.name, [this](const RpcInvocation & invocation) { return listTopics(invocation); });

  return all_registered;
}

void RpcRouter::unregisterRpcs(RoomConnection & room_connection)
{
  for (const char * method_name : kRpcMethodNames) {
    room_connection.unregisterRpc(method_name);
  }
}

std::optional<std::string> RpcRouter::callService(const RpcInvocation & invocation)
{
  return withRequiredCallerIdentity(kServiceCallRpcMethod, invocation, [this, &invocation]() {
    ServiceCallRequest request;
    // Keep the parsed request alive across the dispatch try/catch so failures
    // after authorization can still log the target service name.
    try {
      request = service_call_payloads::parse(invocation.payload);
    } catch (const std::exception & exc) {
      throwLoggedError(kServiceCallRpcMethod, invocation, exc);
    }

    if (!access_policy_.allows(AccessOperation::CallService, request.service)) {
      LogEvent event(kRpcRouterLogger, "rpc_request_rejected");
      addLogFields(event, kServiceCallRpcMethod, invocation)
        .field("reason", "forbidden")
        .field("service", request.service)
        .field("error", "service_not_permitted");
      event.warn();
      throw RpcHandlerError(protocol::kRpcErrorForbidden, "ROS service '" + request.service + "' not permitted.");
    }

    try {
      const auto start = std::chrono::steady_clock::now();
      // First hop onto the ROS executor thread to create the client/request with
      // node-affine APIs, then wait on the returned service-call future here.
      auto submit_future = ros_executor_queue_.submit(
        [this, requester_identity = invocation.caller_identity, request = std::move(request)]() mutable {
          return ros_service_caller_.call(requester_identity, request);
        });
      auto result_future = submit_future.get();

      auto response = result_future.get();
      const int elapsed_ms = static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count());
      return service_call_payloads::serialize(response.service, response.interface_type, response.response, elapsed_ms);
    } catch (const std::exception & exc) {
      throwLoggedError(kServiceCallRpcMethod, invocation, exc, std::string_view(request.service));
    }
  });
}

std::optional<std::string> RpcRouter::getInterfaces(const RpcInvocation & invocation)
{
  return withRequiredCallerIdentity(kInterfacesGetRpcMethod, invocation, [&invocation]() {
    try {
      auto interface_types = interface_payloads::parse(invocation.payload);

      // Repeated requested types and shared nested dependencies can both return
      // the same definition; keep the first-seen wire order while deduplicating.
      std::set<std::string> seen;
      std::vector<InterfaceDefinition> definitions;

      for (const auto & interface_type : interface_types) {
        for (auto & definition : lookupInterfaceDefinitions(interface_type)) {
          if (!seen.insert(definition.interface_type).second) {
            continue;
          }
          definitions.push_back(std::move(definition));
        }
      }
      return interface_payloads::serialize(definitions);
    } catch (const std::exception & exc) {
      throwLoggedError(kInterfacesGetRpcMethod, invocation, exc);
    }
  });
}

std::optional<std::string> RpcRouter::listServices(const RpcInvocation & invocation)
{
  return withRequiredCallerIdentity(kServicesListRpcMethod, invocation, [this, &invocation]() {
    try {
      auto request = resource_list_payloads::parse(invocation.payload);
      auto future = ros_executor_queue_.submit([this, request = std::move(request)]() mutable {
        return filterResourceEntries(
          node_.get_service_names_and_types(), access_policy_, AccessOperation::CallService, request);
      });
      return resource_list_payloads::serializeServiceList(future.get());
    } catch (const std::exception & exc) {
      throwLoggedError(kServicesListRpcMethod, invocation, exc);
    }
  });
}

std::optional<std::string> RpcRouter::listTopics(const RpcInvocation & invocation)
{
  return withRequiredCallerIdentity(kTopicsListRpcMethod, invocation, [this, &invocation]() {
    try {
      auto request = resource_list_payloads::parse(invocation.payload);
      auto future = ros_executor_queue_.submit([this, request = std::move(request)]() mutable {
        return filterResourceEntries(
          node_.get_topic_names_and_types(), access_policy_, AccessOperation::Subscribe, request);
      });
      return resource_list_payloads::serializeTopicList(future.get());
    } catch (const std::exception & exc) {
      throwLoggedError(kTopicsListRpcMethod, invocation, exc);
    }
  });
}

}  // namespace livekit_ros2_bridge
