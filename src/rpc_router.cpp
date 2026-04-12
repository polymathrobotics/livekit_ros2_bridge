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
#include <memory>
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
constexpr char kUnknownLogValue[] = "<unknown>";

using GraphResourceMap = std::map<std::string, std::vector<std::string>>;

struct RpcMethodSpec
{
  const char * name;
  const char * resource;
  std::optional<AccessOperation> access_op;
};

struct RpcRequestContext
{
  const RpcMethodSpec & method;
  const RpcInvocation & invocation;
};

struct RpcErrorContext
{
  std::optional<std::string_view> service;
};

constexpr RpcMethodSpec kServiceCallMethod{
  protocol::kRpcServiceCall,
  "services",
  AccessOperation::CallService,
};
constexpr RpcMethodSpec kInterfacesGetMethod{
  protocol::kRpcInterfacesGet,
  "interfaces",
  std::nullopt,
};
constexpr RpcMethodSpec kServicesListMethod{
  protocol::kRpcServicesList,
  "services",
  AccessOperation::CallService,
};
constexpr RpcMethodSpec kTopicsListMethod{
  protocol::kRpcTopicsList,
  "topics",
  AccessOperation::Subscribe,
};

[[noreturn]] void rethrowAsRpcHandlerError(const std::exception & exc);

const char * requestIdForLog(const RpcRequestContext & ctx)
{
  return ctx.invocation.request_id.empty() ? kUnknownLogValue : ctx.invocation.request_id.c_str();
}

const char * requesterIdentityForLog(const RpcRequestContext & ctx)
{
  return ctx.invocation.caller_identity.empty() ? kUnknownLogValue : ctx.invocation.caller_identity.c_str();
}

std::uint32_t rpcErrorCodeForException(const std::exception & exc)
{
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

const char * rpcReasonForCode(std::uint32_t code)
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

[[noreturn]] void logAndRethrowRpcHandlerError(
  const RpcRequestContext & ctx, const std::exception & exc, RpcErrorContext error_context = {})
{
  const auto code = rpcErrorCodeForException(exc);
  const char * reason = rpcReasonForCode(code);
  LogEvent event(kRpcRouterLogger, code == protocol::kRpcErrorInternal ? "rpc_request_failed" : "rpc_request_rejected");
  event.field("reason", reason)
    .field("method", ctx.method.name)
    .field("request_id", requestIdForLog(ctx))
    .field("requester_identity", requesterIdentityForLog(ctx));

  if (error_context.service.has_value()) {
    event.field("resource", ctx.method.resource == nullptr ? kUnknownLogValue : ctx.method.resource)
      .field("service", *error_context.service)
      .field("error", exc.what());
  } else if (ctx.method.resource != nullptr) {
    event.field("resource", ctx.method.resource).field("error", exc.what());
  } else {
    event.field("error", exc.what());
  }

  if (code == protocol::kRpcErrorInternal) {
    event.error();
  } else {
    event.warn();
  }

  rethrowAsRpcHandlerError(exc);
}

std::vector<ResourceListEntry> filterResourceListEntries(
  const GraphResourceMap & all_graph_resources,
  const AccessPolicy & access_policy,
  AccessOperation access_op,
  const ResourceListRequest & request)
{
  std::vector<ResourceListEntry> entries;

  for (const auto & graph_resource_entry : all_graph_resources) {
    const auto & resource_name = graph_resource_entry.first;
    const auto & interface_types = graph_resource_entry.second;

    if (interface_types.size() != 1U) {
      continue;
    }
    if (!access_policy.allows(access_op, resource_name)) {
      continue;
    }
    const auto & interface_type = interface_types.front();
    const bool query_matches_resource = !request.query.has_value() ||
                                        resource_name.find(*request.query) != std::string::npos ||
                                        interface_type.find(*request.query) != std::string::npos;
    if (!query_matches_resource) {
      continue;
    }

    entries.push_back({resource_name, interface_type});
    if (request.limit.has_value() && entries.size() >= *request.limit) {
      break;
    }
  }

  return entries;
}

[[noreturn]] void rethrowAsRpcHandlerError(const std::exception & exc)
{
  if (dynamic_cast<const RpcHandlerError *>(&exc) != nullptr) {
    throw;
  }
  // Payload and bounds errors are caller-fixable invalid requests; everything
  // else is reported as an internal bridge failure.
  if (
    dynamic_cast<const std::invalid_argument *>(&exc) != nullptr ||
    dynamic_cast<const std::out_of_range *>(&exc) != nullptr)
  {
    throw RpcHandlerError(protocol::kRpcErrorInvalidRequest, exc.what());
  }
  throw RpcHandlerError(protocol::kRpcErrorInternal, exc.what());
}

template <typename HandleRpcT>
std::optional<std::string> handleRpcWithCallerIdentity(const RpcRequestContext & ctx, HandleRpcT handle_rpc)
{
  // Caller identity is part of the authorization model, so reject anonymous
  // invocations before parsing payloads or touching bridge state.
  if (ctx.invocation.caller_identity.empty()) {
    LogEvent event(kRpcRouterLogger, "rpc_request_rejected");
    event.field("reason", "unauthorized")
      .field("method", ctx.method.name)
      .field("request_id", requestIdForLog(ctx))
      .field("requester_identity", requesterIdentityForLog(ctx));
    if (ctx.method.resource != nullptr) {
      event.field("resource", ctx.method.resource);
    }
    event.field("error", "caller_identity_required").warn();
    throw RpcHandlerError(protocol::kRpcErrorUnauthorized, "caller_identity is required for this RPC");
  }

  try {
    return handle_rpc();
  } catch (const RpcHandlerError &) {
    throw;
  } catch (const std::exception & exc) {
    logAndRethrowRpcHandlerError(ctx, exc);
  }
}

template <typename QueryResourcesT, typename SerializeResponseT>
std::optional<std::string> handleResourceListRpc(
  const RpcRequestContext & ctx,
  RosExecutorQueue & ros_executor_queue,
  const AccessPolicy & access_policy,
  QueryResourcesT query_resources,
  SerializeResponseT serialize_response)
{
  if (!ctx.method.access_op.has_value()) {
    throw std::logic_error("resource list RPC method must declare an access operation");
  }

  return handleRpcWithCallerIdentity(
    ctx,
    [&ctx,
     &ros_executor_queue,
     &access_policy,
     query_resources = std::move(query_resources),
     serialize_response = std::move(serialize_response)]() mutable {
      try {
        auto request = parseResourceListRequest(ctx.invocation.payload);
        // Query the ROS graph on the executor thread, then apply access-policy
        // filtering before serializing the response back to the RPC caller.
        auto future = ros_executor_queue.submit([request,
                                                 &access_policy,
                                                 access_op = *ctx.method.access_op,
                                                 query_resources = std::move(query_resources)]() mutable {
          return filterResourceListEntries(query_resources(), access_policy, access_op, request);
        });
        return serialize_response(future.get());
      } catch (const std::exception & exc) {
        logAndRethrowRpcHandlerError(ctx, exc);
      }
    });
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

std::array<std::pair<const char *, RpcHandler>, 4> RpcRouter::rpcEntrypoints()
{
  return {{
    {kServiceCallMethod.name, [this](const RpcInvocation & invocation) { return handleServiceCall(invocation); }},
    {kInterfacesGetMethod.name, [this](const RpcInvocation & invocation) { return handleInterfacesGet(invocation); }},
    {kServicesListMethod.name, [this](const RpcInvocation & invocation) { return handleServiceList(invocation); }},
    {kTopicsListMethod.name, [this](const RpcInvocation & invocation) { return handleTopicList(invocation); }},
  }};
}

bool RpcRouter::registerRpcMethods(RoomConnection & room_connection)
{
  bool all_registered = true;
  for (const auto & method : rpcEntrypoints()) {
    if (!room_connection.registerRpcMethod(method.first, method.second)) {
      LogEvent(kRpcRouterLogger, "rpc_method_registration_failed").field("method", method.first).error();
      // Registration is best-effort rather than transactional so one failure
      // does not hide other methods that can still be served on this connection.
      all_registered = false;
    }
  }
  return all_registered;
}

void RpcRouter::unregisterRpcMethods(RoomConnection & room_connection)
{
  for (const auto & method : rpcEntrypoints()) {
    if (!room_connection.unregisterRpcMethod(method.first)) {
      LogEvent(kRpcRouterLogger, "rpc_method_unregistration_failed").field("method", method.first).error();
    }
  }
}

std::optional<std::string> RpcRouter::handleServiceCall(const RpcInvocation & invocation)
{
  const RpcRequestContext ctx{kServiceCallMethod, invocation};
  return handleRpcWithCallerIdentity(ctx, [this, &ctx]() {
    ServiceCallRequest request;
    try {
      request = parseServiceCallRequest(ctx.invocation.payload);
    } catch (const std::exception & exc) {
      logAndRethrowRpcHandlerError(ctx, exc);
    }

    if (!access_policy_.allows(*ctx.method.access_op, request.service)) {
      LogEvent(kRpcRouterLogger, "rpc_request_rejected")
        .field("reason", "forbidden")
        .field("method", ctx.method.name)
        .field("request_id", requestIdForLog(ctx))
        .field("requester_identity", requesterIdentityForLog(ctx))
        .field("resource", ctx.method.resource)
        .field("service", request.service)
        .field("error", "service_not_permitted")
        .warn();
      throw RpcHandlerError(protocol::kRpcErrorForbidden, "ROS service '" + request.service + "' not permitted.");
    }

    try {
      const auto start = std::chrono::steady_clock::now();
      // First hop onto the ROS executor thread to create the client/request with
      // node-affine APIs, then wait on the returned service-call future here.
      auto queued_call_future = ros_executor_queue_.submit(
        [this, requester_identity = ctx.invocation.caller_identity, request = std::move(request)]() mutable {
          return ros_service_caller_.call(requester_identity, request);
        });
      auto result_future = queued_call_future.get();

      auto service_call_response = result_future.get();
      const int elapsed_ms = static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count());
      return serializeServiceCallResponse(
        service_call_response.service,
        service_call_response.interface_type,
        service_call_response.response,
        elapsed_ms);
    } catch (const std::exception & exc) {
      logAndRethrowRpcHandlerError(ctx, exc, RpcErrorContext{std::string_view(request.service)});
    }
  });
}

std::optional<std::string> RpcRouter::handleInterfacesGet(const RpcInvocation & invocation)
{
  const RpcRequestContext ctx{kInterfacesGetMethod, invocation};
  return handleRpcWithCallerIdentity(ctx, [this, &ctx]() {
    try {
      auto types = parseRequestedInterfaceTypes(ctx.invocation.payload);

      std::set<std::string> seen;
      std::vector<InterfaceDefinition> all_interfaces;

      for (const auto & type : types) {
        for (auto & definition : lookupInterfaceDefinitions(type)) {
          if (seen.insert(definition.interface_type).second) {
            all_interfaces.push_back(std::move(definition));
          }
        }
      }
      return serializeInterfacesResponse(all_interfaces);
    } catch (const std::exception & exc) {
      logAndRethrowRpcHandlerError(ctx, exc);
    }
  });
}

std::optional<std::string> RpcRouter::handleServiceList(const RpcInvocation & invocation)
{
  const RpcRequestContext ctx{kServicesListMethod, invocation};
  return handleResourceListRpc(
    ctx,
    ros_executor_queue_,
    access_policy_,
    [this]() { return node_.get_service_names_and_types(); },
    serializeServiceListResponse);
}

std::optional<std::string> RpcRouter::handleTopicList(const RpcInvocation & invocation)
{
  const RpcRequestContext ctx{kTopicsListMethod, invocation};
  return handleResourceListRpc(
    ctx,
    ros_executor_queue_,
    access_policy_,
    [this]() { return node_.get_topic_names_and_types(); },
    serializeTopicListResponse);
}

}  // namespace livekit_ros2_bridge
