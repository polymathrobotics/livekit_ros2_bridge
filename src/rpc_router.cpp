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

#include "interface_definition_lookup.hpp"
#include "livekit/rpc_error.h"
#include "protocol/constants.hpp"
#include "protocol/interfaces_json.hpp"
#include "protocol/resources.hpp"
#include "protocol/resources_json.hpp"
#include "protocol/services_json.hpp"
#include "protocol/validation_error.hpp"
#include "rclcpp/logging.hpp"
#include "ros_executor_queue.hpp"
#include "ros_service_caller.hpp"
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge
{

namespace
{

const auto kLogger = rclcpp::get_logger("rpc_router");

constexpr std::array<const char *, 4> kRpcNames{
  protocol::kCallServiceRpc,
  protocol::kShowInterfaceRpc,
  protocol::kListServicesRpc,
  protocol::kListTopicsRpc,
};

template <typename EventT>
EventT && addLogFields(EventT && event, const char * method_name, const livekit::RpcInvocationData & invocation)
{
  event.field("method", method_name)
    .fieldOr("request_id", invocation.request_id)
    .fieldOr("requester_identity", invocation.caller_identity);
  return std::forward<EventT>(event);
}

[[noreturn]] void throwLoggedError(
  const char * method_name,
  const livekit::RpcInvocationData & invocation,
  const std::exception & exc,
  std::optional<std::string_view> service = std::nullopt)
{
  // Payload/range validation maps to invalid_request; arbitrary exceptions become internal.
  const bool validation_exception = dynamic_cast<const std::invalid_argument *>(&exc) != nullptr ||
                                    dynamic_cast<const std::out_of_range *>(&exc) != nullptr;
  const auto code = validation_exception ? protocol::kInvalidRequestRpcError : protocol::kInternalRpcError;
  const bool internal_error = code == protocol::kInternalRpcError;
  const char * reason = internal_error ? "internal" : "invalid_request";
  LogEvent event(kLogger, internal_error ? "rpc_request_failed" : "rpc_request_rejected");
  addLogFields(event, method_name, invocation).field("reason", reason);
  const auto * validation = dynamic_cast<const protocol::ValidationError *>(&exc);
  if (validation != nullptr) {
    event.field("request_field", validation->field());
  }

  if (service) {
    event.field("service", *service);
  }
  event.field("error", exc.what());

  if (internal_error) {
    event.error();
  } else {
    event.warn();
  }

  throw livekit::RpcError(code, exc.what());
}

ResourceNamesAndTypes filterResources(
  const ResourceNamesAndTypes & graph_resources,
  const AccessPolicy & policy,
  AccessOperation operation,
  const ResourceListRequest & request)
{
  ResourceNamesAndTypes filtered_resources;

  for (const auto & [name, types] : graph_resources) {
    // The RPC schema exposes one interface type per resource; omit ambiguous ROS resources.
    if (types.size() != 1U) {
      continue;
    }
    if (!policy.allows(operation, name)) {
      continue;
    }
    const auto & interface_type = types.front();
    if (request.query) {
      const auto & query = *request.query;
      if (name.find(query) == std::string::npos && interface_type.find(query) == std::string::npos) {
        continue;
      }
    }

    filtered_resources.emplace(name, ResourceNamesAndTypes::mapped_type{interface_type});
    // Limit after policy/query filtering; denied resources do not consume capacity.
    if (request.limit && filtered_resources.size() >= *request.limit) {
      break;
    }
  }

  return filtered_resources;
}

template <typename HandleRpcT>
std::optional<std::string> withCallerIdentity(
  const char * method_name, const livekit::RpcInvocationData & invocation, HandleRpcT handle)
{
  // Reject anonymous callers before parsing; validation details must not leak.
  if (invocation.caller_identity.empty()) {
    addLogFields(LogEvent(kLogger, "rpc_request_rejected"), method_name, invocation)
      .field("reason", "unauthorized")
      .field("error", "caller_identity_required")
      .warn();
    throw livekit::RpcError(protocol::kUnauthorizedRpcError, "caller_identity is required for this RPC");
  }

  try {
    return handle();
  } catch (const livekit::RpcError &) {
    throw;
  } catch (const std::exception & exc) {
    throwLoggedError(method_name, invocation, exc);
  }
}

}  // namespace

RpcRouter::RpcRouter(
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph,
  const AccessPolicy & access_policy,
  RosExecutorQueue & ros_executor_queue,
  RosServiceCaller & ros_service_caller)
: graph_(std::move(graph))
, access_policy_(access_policy)
, ros_executor_queue_(ros_executor_queue)
, ros_service_caller_(ros_service_caller)
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
                     protocol::kCallServiceRpc,
                     [this](const livekit::RpcInvocationData & invocation) { return callService(invocation); }) &&
                   all_registered;
  all_registered = connection.registerRpc(
                     protocol::kShowInterfaceRpc,
                     [this](const livekit::RpcInvocationData & invocation) { return getInterfaces(invocation); }) &&
                   all_registered;
  all_registered = connection.registerRpc(
                     protocol::kListServicesRpc,
                     [this](const livekit::RpcInvocationData & invocation) { return listServices(invocation); }) &&
                   all_registered;
  all_registered = connection.registerRpc(
                     protocol::kListTopicsRpc,
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

  for (const char * method_name : kRpcNames) {
    (void)connection.unregisterRpc(method_name);
  }
}

std::optional<std::string> RpcRouter::callService(const livekit::RpcInvocationData & invocation)
{
  return withCallerIdentity(protocol::kCallServiceRpc, invocation, [this, &invocation]() {
    auto request = protocol::services::parse(invocation.payload);

    if (!access_policy_.allows(AccessOperation::CallService, request.service)) {
      addLogFields(LogEvent(kLogger, "rpc_request_rejected"), protocol::kCallServiceRpc, invocation)
        .field("reason", "forbidden")
        .field("service", request.service)
        .field("error", "service_not_permitted")
        .warn();
      throw livekit::RpcError(protocol::kForbiddenRpcError, "ROS service '" + request.service + "' not permitted.");
    }

    // Keep the normalized service name available after request ownership moves to the executor.
    const std::string service = request.service;
    try {
      auto submit_future = ros_executor_queue_.submit(
        [this, requester_identity = invocation.caller_identity, request = std::move(request)]() mutable {
          return ros_service_caller_.call(requester_identity, request);
        });
      auto result_future = submit_future.get();

      return protocol::services::serialize(result_future.get());
    } catch (const std::exception & exc) {
      throwLoggedError(protocol::kCallServiceRpc, invocation, exc, std::string_view(service));
    }
  });
}

std::optional<std::string> RpcRouter::getInterfaces(const livekit::RpcInvocationData & invocation)
{
  return withCallerIdentity(protocol::kShowInterfaceRpc, invocation, [&invocation]() {
    auto requested_types = protocol::interfaces::parse(invocation.payload);

    // Preserve first-seen protocol order while deduplicating shared definitions.
    std::set<std::string> seen;
    std::vector<InterfaceDefinition> definitions;

    for (const auto & type : requested_types) {
      for (auto & definition : lookupInterfaceDefinitions(type)) {
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
  return withCallerIdentity(protocol::kListServicesRpc, invocation, [this, &invocation]() {
    auto request = protocol::resources::parseRequest(invocation.payload);
    auto future = ros_executor_queue_.submit([this, request = std::move(request)]() mutable {
      return filterResources(
        graph_->get_service_names_and_types(), access_policy_, AccessOperation::CallService, request);
    });
    return protocol::resources::serializeServices(future.get());
  });
}

std::optional<std::string> RpcRouter::listTopics(const livekit::RpcInvocationData & invocation)
{
  return withCallerIdentity(protocol::kListTopicsRpc, invocation, [this, &invocation]() {
    auto request = protocol::resources::parseRequest(invocation.payload);
    auto future = ros_executor_queue_.submit([this, request = std::move(request)]() mutable {
      return filterResources(graph_->get_topic_names_and_types(), access_policy_, AccessOperation::Subscribe, request);
    });
    return protocol::resources::serializeTopics(future.get());
  });
}

}  // namespace livekit_ros2_bridge
