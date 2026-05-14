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
#include <exception>
#include <optional>
#include <set>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>

#include "livekit/rpc_error.h"
#include "protocol/constants.hpp"
#include "protocol/interfaces_json.hpp"
#include "protocol/resources.hpp"
#include "protocol/resources_json.hpp"
#include "protocol/services_json.hpp"
#include "protocol/validation_error.hpp"
#include "rclcpp/logging.hpp"
#include "ros_executor_queue.hpp"
#include "ros_interfaces/definition_lookup.hpp"
#include "ros_service_caller.hpp"
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge
{

namespace
{

const auto kLogger = rclcpp::get_logger("livekit_ros2_bridge.rpc_router");

constexpr std::array<const char *, 4> kMethods{
  protocol::kCallServiceMethod,
  protocol::kShowInterfaceMethod,
  protocol::kListServicesMethod,
  protocol::kListTopicsMethod,
};

[[noreturn]] void throwRpcError(
  const char * method,
  const livekit::RpcInvocationData & invocation,
  const std::exception & exc,
  std::optional<std::string_view> service = std::nullopt)
{
  // Payload/range validation maps to invalid_request; arbitrary exceptions become internal.
  const bool invalid_request = dynamic_cast<const std::invalid_argument *>(&exc) != nullptr ||
                               dynamic_cast<const std::out_of_range *>(&exc) != nullptr;
  const auto code = invalid_request ? protocol::kInvalidRequestRpcCode : protocol::kInternalRpcCode;
  const bool internal = code == protocol::kInternalRpcCode;
  LogEvent event(kLogger, internal ? "rpc_request_failed" : "rpc_request_rejected");
  event.field("method", method)
    .fieldOr("request_id", invocation.request_id)
    .fieldOr("requester_identity", invocation.caller_identity);
  if (!internal) {
    event.field("reason", "invalid_request");
  }
  const auto * validation = dynamic_cast<const protocol::ValidationError *>(&exc);
  if (validation != nullptr) {
    event.field("request_field", validation->field());
  }

  if (service) {
    event.fieldOr("service", *service);
  }
  event.field("error", exc.what());

  if (internal) {
    event.error();
  } else {
    event.warn();
  }

  throw livekit::RpcError(code, exc.what());
}

ResourceTypesByName filterResources(
  const ResourceTypesByName & resources,
  const AccessPolicy & policy,
  AccessOperation operation,
  const ResourceListRequest & request)
{
  ResourceTypesByName filtered;

  for (const auto & [name, types] : resources) {
    // The RPC schema exposes one interface type per resource; omit ambiguous ROS resources.
    if (types.size() != 1U) {
      continue;
    }
    if (!policy.allows(operation, name)) {
      continue;
    }
    const auto & type = types.front();
    if (request.query) {
      const auto & query = *request.query;
      if (name.find(query) == std::string::npos && type.find(query) == std::string::npos) {
        continue;
      }
    }

    filtered.emplace(name, ResourceTypesByName::mapped_type{type});
    // Limit after policy/query filtering; denied resources do not consume capacity.
    if (request.limit && filtered.size() >= *request.limit) {
      break;
    }
  }

  return filtered;
}

template <typename HandlerT>
std::optional<std::string> withCallerIdentity(
  const char * method, const livekit::RpcInvocationData & invocation, HandlerT handler)
{
  // Reject anonymous callers before parsing; validation details must not leak.
  if (invocation.caller_identity.empty()) {
    LogEvent(kLogger, "rpc_request_rejected")
      .field("method", method)
      .fieldOr("request_id", invocation.request_id)
      .fieldOr("requester_identity", invocation.caller_identity)
      .field("reason", "unauthorized")
      .warn();
    throw livekit::RpcError(protocol::kUnauthorizedRpcCode, "caller_identity is required for this RPC");
  }

  try {
    return handler();
  } catch (const livekit::RpcError &) {
    throw;
  } catch (const std::exception & exc) {
    throwRpcError(method, invocation, exc);
  }
}

}  // namespace

RpcRouter::RpcRouter(
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph,
  const AccessPolicy & policy,
  RosExecutorQueue & queue,
  RosServiceCaller & caller)
: graph_(std::move(graph))
, policy_(policy)
, queue_(queue)
, caller_(caller)
{}

RpcRouter::~RpcRouter()
{
  unregisterRpcs();
}

bool RpcRouter::registerRpcs(RoomConnection & connection)
{
  registered_connection_ = &connection;
  bool all_registered = true;

  // Registration is best-effort rather than transactional so one failure does
  // not hide other methods that can still be served on this connection.
  all_registered = connection.registerRpc(
                     protocol::kCallServiceMethod,
                     [this](const livekit::RpcInvocationData & invocation) { return callService(invocation); }) &&
                   all_registered;
  all_registered = connection.registerRpc(
                     protocol::kShowInterfaceMethod,
                     [this](const livekit::RpcInvocationData & invocation) { return showInterfaces(invocation); }) &&
                   all_registered;
  all_registered = connection.registerRpc(
                     protocol::kListServicesMethod,
                     [this](const livekit::RpcInvocationData & invocation) { return listServices(invocation); }) &&
                   all_registered;
  all_registered = connection.registerRpc(
                     protocol::kListTopicsMethod,
                     [this](const livekit::RpcInvocationData & invocation) { return listTopics(invocation); }) &&
                   all_registered;

  return all_registered;
}

void RpcRouter::unregisterRpcs() noexcept
{
  if (registered_connection_ == nullptr) {
    return;
  }

  RoomConnection & connection = *registered_connection_;
  registered_connection_ = nullptr;

  for (const char * method : kMethods) {
    (void)connection.unregisterRpc(method);
  }
}

std::optional<std::string> RpcRouter::callService(const livekit::RpcInvocationData & invocation)
{
  return withCallerIdentity(protocol::kCallServiceMethod, invocation, [this, &invocation]() {
    auto request = protocol::services::parse(invocation.payload);

    if (!policy_.allows(AccessOperation::CallService, request.name)) {
      LogEvent(kLogger, "rpc_request_rejected")
        .field("method", protocol::kCallServiceMethod)
        .fieldOr("request_id", invocation.request_id)
        .fieldOr("requester_identity", invocation.caller_identity)
        .field("reason", "forbidden")
        .fieldOr("service", request.name)
        .warn();
      throw livekit::RpcError(protocol::kForbiddenRpcCode, "ROS service '" + request.name + "' not permitted.");
    }

    // Keep the normalized service name available after request ownership moves to the executor.
    const std::string service = request.name;
    try {
      auto submit_future =
        queue_.submit([this, requester_identity = invocation.caller_identity, request = std::move(request)]() mutable {
          return caller_.call(requester_identity, request);
        });
      auto result_future = submit_future.get();

      return protocol::services::serialize(result_future.get());
    } catch (const std::exception & exc) {
      throwRpcError(protocol::kCallServiceMethod, invocation, exc, std::string_view(service));
    }
  });
}

std::optional<std::string> RpcRouter::showInterfaces(const livekit::RpcInvocationData & invocation)
{
  return withCallerIdentity(protocol::kShowInterfaceMethod, invocation, [&invocation]() {
    auto types = protocol::interfaces::parse(invocation.payload);

    // Preserve first-seen protocol order while deduplicating shared definitions.
    std::set<std::string> seen;
    std::vector<InterfaceDefinition> definitions;

    for (const auto & type : types) {
      for (auto & definition : ros_interfaces::lookupDefinitions(type)) {
        if (!seen.insert(definition.type).second) {
          continue;
        }
        definitions.push_back(std::move(definition));
      }
    }
    return protocol::interfaces::serialize(definitions);
  });
}

std::optional<std::string> RpcRouter::listServices(const livekit::RpcInvocationData & invocation)
{
  return withCallerIdentity(protocol::kListServicesMethod, invocation, [this, &invocation]() {
    auto request = protocol::resources::parse(invocation.payload);
    auto future = queue_.submit([this, request = std::move(request)]() mutable {
      return filterResources(graph_->get_service_names_and_types(), policy_, AccessOperation::CallService, request);
    });
    return protocol::resources::serializeServices(future.get());
  });
}

std::optional<std::string> RpcRouter::listTopics(const livekit::RpcInvocationData & invocation)
{
  return withCallerIdentity(protocol::kListTopicsMethod, invocation, [this, &invocation]() {
    auto request = protocol::resources::parse(invocation.payload);
    auto future = queue_.submit([this, request = std::move(request)]() mutable {
      return filterResources(graph_->get_topic_names_and_types(), policy_, AccessOperation::Subscribe, request);
    });
    return protocol::resources::serializeTopics(future.get());
  });
}

}  // namespace livekit_ros2_bridge
