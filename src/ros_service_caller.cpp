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

#include "ros_service_caller.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <new>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "payloads/service_call_payloads.hpp"
#include "rcl/client.h"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/typesupport_helpers.hpp"
#include "rclcpp/version.h"
#include "rcpputils/shared_library.hpp"
#include "rmw/types.h"
#include "rosidl_runtime_cpp/message_initialization.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "utils/bounded_lru_cache.hpp"
#include "utils/event_throttle.hpp"
#include "utils/interface_types.hpp"
#include "utils/log_event.hpp"
#include "utils/reentrant_quiesce_guard.hpp"
#include "utils/scope_exit.hpp"

// rclcpp 28+ (Jazzy) renamed get_typesupport_handle -> get_message_typesupport_handle.
#if !RCLCPP_VERSION_GTE(28, 0, 0)
namespace rclcpp
{
inline const rosidl_message_type_support_t * get_message_typesupport_handle(
  const std::string & type, const std::string & typesupport_identifier, rcpputils::SharedLibrary & library)
{
  return get_typesupport_handle(type, typesupport_identifier, library);
}
}  // namespace rclcpp
#endif

namespace livekit_ros2_bridge
{

namespace
{

using MessageMembers = rosidl_typesupport_introspection_cpp::MessageMembers;

constexpr char kSerializationTsId[] = "rosidl_typesupport_cpp";
constexpr char kIntrospectionTsId[] = "rosidl_typesupport_introspection_cpp";
constexpr char kServiceTypeSupportSymbolPrefix[] = "__get_service_type_support_handle__";
constexpr char kRequestTypeSuffix[] = "_Request";
constexpr char kResponseTypeSuffix[] = "_Response";
constexpr auto kPollInterval = std::chrono::milliseconds(10);
constexpr auto kResponseLogThrottleMs = 5000;
constexpr int kDefaultTimeoutMs = 2000;
constexpr int kMaxInflightPerRequester = 4;
constexpr std::size_t kInvalidServiceTypeCacheCapacity = 256U;
constexpr char kAnyServiceLogValue[] = "*";
const auto kRosServiceCallerLogger = rclcpp::get_logger("ros_service_caller");
using FailureCache = BoundedLruCache<std::string, std::exception_ptr>;

rclcpp::SerializedMessage toSerializedMessage(const std::vector<std::uint8_t> & payload)
{
  rclcpp::SerializedMessage serialized(payload.size());
  auto & rcl_msg = serialized.get_rcl_serialized_message();
  if (!payload.empty()) {
    std::memcpy(rcl_msg.buffer, payload.data(), payload.size());
  }
  rcl_msg.buffer_length = payload.size();
  return serialized;
}

std::vector<std::uint8_t> serializedMessageBytes(const rclcpp::SerializedMessage & payload)
{
  const auto & rcl_msg = payload.get_rcl_serialized_message();
  if (rcl_msg.buffer == nullptr || rcl_msg.buffer_length == 0U) {
    return {};
  }
  return std::vector<std::uint8_t>(rcl_msg.buffer, rcl_msg.buffer + rcl_msg.buffer_length);
}

std::string serviceRequestTypeName(const std::string & service_type)
{
  return service_type + kRequestTypeSuffix;
}

std::string serviceResponseTypeName(const std::string & service_type)
{
  return service_type + kResponseTypeSuffix;
}

const MessageMembers & getMessageMembers(const rosidl_message_type_support_t * ts)
{
  if (ts == nullptr || ts->data == nullptr) {
    throw std::runtime_error("Introspection type support handle is null");
  }
  return *static_cast<const MessageMembers *>(ts->data);
}

class DynamicMessageStorage
{
public:
  DynamicMessageStorage(const MessageMembers & members, rosidl_runtime_cpp::MessageInitialization init)
  : members_(members)
  , data_(::operator new(members.size_of_))
  {
    members_.init_function(data_, init);
  }

  ~DynamicMessageStorage()
  {
    members_.fini_function(data_);
    ::operator delete(data_);
  }

  DynamicMessageStorage(const DynamicMessageStorage &) = delete;
  DynamicMessageStorage & operator=(const DynamicMessageStorage &) = delete;

  void * data()
  {
    return data_;
  }

private:
  const MessageMembers & members_;
  void * data_;
};

struct ResolvedMessageSupport
{
  // Keep both libraries alive for as long as the cached serialization and
  // introspection handles may be used by an active client entry.
  explicit ResolvedMessageSupport(const std::string & interface_type)
  : serialization_library(rclcpp::get_typesupport_library(interface_type, kSerializationTsId))
  , introspection_library(rclcpp::get_typesupport_library(interface_type, kIntrospectionTsId))
  , serialization_ts(rclcpp::get_message_typesupport_handle(interface_type, kSerializationTsId, *serialization_library))
  , introspection_ts(rclcpp::get_message_typesupport_handle(interface_type, kIntrospectionTsId, *introspection_library))
  , members(getMessageMembers(introspection_ts))
  , serialization(serialization_ts)
  {}

  std::shared_ptr<rcpputils::SharedLibrary> serialization_library;
  std::shared_ptr<rcpputils::SharedLibrary> introspection_library;
  const rosidl_message_type_support_t * serialization_ts;
  const rosidl_message_type_support_t * introspection_ts;
  const MessageMembers & members;
  rclcpp::SerializationBase serialization;
};

// rclcpp 28+ (Jazzy) provides get_service_typesupport_handle natively.
// On Humble we load the symbol manually.
#if RCLCPP_VERSION_GTE(28, 0, 0)

const rosidl_service_type_support_t * loadServiceTypeSupportHandle(
  const std::string & service_type, const std::string & typesupport_identifier, rcpputils::SharedLibrary & library)
{
  return rclcpp::get_service_typesupport_handle(service_type, typesupport_identifier, library);
}

#else

const rosidl_service_type_support_t * loadServiceTypeSupportHandle(
  const std::string & service_type, const std::string & typesupport_identifier, rcpputils::SharedLibrary & library)
{
  std::string symbol = typesupport_identifier + kServiceTypeSupportSymbolPrefix;
  for (const char ch : service_type) {
    if (ch == '/') {
      symbol += "__";
    } else {
      symbol += ch;
    }
  }

  if (!library.has_symbol(symbol)) {
    throw std::runtime_error("Service typesupport symbol not found: " + symbol);
  }

  using GetServiceTsFunc = const rosidl_service_type_support_t * (*)();
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  auto get_ts = reinterpret_cast<GetServiceTsFunc>(library.get_symbol(symbol));
  return get_ts();
}

#endif

struct ServiceClientEntry
{
  struct ResolvedServiceTypeSupport
  {
    explicit ResolvedServiceTypeSupport(const std::string & service_type)
    : service_ts_library(rclcpp::get_typesupport_library(service_type, kSerializationTsId))
    , service_ts(loadServiceTypeSupportHandle(service_type, kSerializationTsId, *service_ts_library))
    , request_type(serviceRequestTypeName(service_type))
    , response_type(serviceResponseTypeName(service_type))
    {}

    std::shared_ptr<rcpputils::SharedLibrary> service_ts_library;
    const rosidl_service_type_support_t * service_ts;
    ResolvedMessageSupport request_type;
    ResolvedMessageSupport response_type;
  };

  ServiceClientEntry(
    const std::string & service_name,
    const std::string & service_type,
    ResolvedServiceTypeSupport & resolved_type_support,
    rcl_node_t * node_handle)
  : service_name(service_name)
  , service_type_name(service_type)
  , resolved_type_support(&resolved_type_support)
  , client(rcl_get_zero_initialized_client())
  {
    rcl_client_options_t options = rcl_client_get_default_options();
    rcl_ret_t ret =
      rcl_client_init(&client, node_handle, resolved_type_support.service_ts, service_name.c_str(), &options);
    if (ret != RCL_RET_OK) {
      throw std::runtime_error("Failed to create rcl service client for '" + service_name + "'");
    }
    node_handle_ = node_handle;
  }

  ~ServiceClientEntry()
  {
    if (node_handle_ != nullptr) {
      rcl_ret_t ret = rcl_client_fini(&client, node_handle_);
      (void)ret;
    }
  }

  ServiceClientEntry(const ServiceClientEntry &) = delete;
  ServiceClientEntry & operator=(const ServiceClientEntry &) = delete;

  std::string service_name;
  std::string service_type_name;
  ResolvedServiceTypeSupport * resolved_type_support = nullptr;
  rcl_client_t client;
  rcl_node_t * node_handle_ = nullptr;
};

struct PendingCallKey
{
  // Sequence numbers come from the underlying rcl client, so the client
  // instance is part of the key when matching responses back to pending calls.
  const ServiceClientEntry * client = nullptr;
  std::int64_t sequence_number = 0;

  bool operator==(const PendingCallKey & other) const
  {
    return client == other.client && sequence_number == other.sequence_number;
  }
};

struct PendingCallKeyHash
{
  std::size_t operator()(const PendingCallKey & key) const
  {
    return std::hash<const ServiceClientEntry *>{}(key.client) ^ (std::hash<std::int64_t>{}(key.sequence_number) << 1U);
  }
};

}  // namespace

struct RosServiceCaller::Impl
{
  explicit Impl(rclcpp::Node & node)
  : node(node)
  {}

  class InflightReservation
  {
  public:
    // Reserve quota before any request build or send step that can fail, then
    // release automatically unless the call is committed into pending_calls.
    InflightReservation(Impl & owner, std::string requester_identity)
    : owner_(&owner)
    , requester_identity_(std::move(requester_identity))
    {
      owner_->acquireInflight(requester_identity_);
    }

    ~InflightReservation()
    {
      if (owner_ != nullptr) {
        owner_->releaseInflight(requester_identity_);
      }
    }

    InflightReservation(const InflightReservation &) = delete;
    InflightReservation & operator=(const InflightReservation &) = delete;

    InflightReservation(InflightReservation && other) noexcept
    : owner_(other.owner_)
    , requester_identity_(std::move(other.requester_identity_))
    {
      other.owner_ = nullptr;
    }

    InflightReservation & operator=(InflightReservation &&) = delete;

    void commit() noexcept
    {
      owner_ = nullptr;
    }

  private:
    Impl * owner_;
    std::string requester_identity_;
  };

  struct PendingCall
  {
    std::string service;
    std::string interface_type;
    std::string requester_identity;
    std::promise<ServiceCallResponse> promise;
    std::chrono::steady_clock::time_point deadline;
  };

  using PendingCalls = std::unordered_map<PendingCallKey, PendingCall, PendingCallKeyHash>;
  using PendingCallIterator = PendingCalls::iterator;

  std::string resolveServiceType(const std::string & service, const std::string & requested_interface_type) const;
  ServiceClientEntry::ResolvedServiceTypeSupport & getOrResolveTypeSupport(const std::string & service_type);
  ServiceClientEntry & getOrCreateClient(const std::string & service, const std::string & service_type);
  void acquireInflight(const std::string & requester_identity);
  void releaseInflight(const std::string & requester_identity);
  void ensurePollTimer();
  bool beginPollCallback(std::function<void()> & on_enter);
  void endPollCallback();
  void quiescePollCallbacks();
  void setPollCallbackHooks(std::function<void()> on_enter, std::function<void()> on_exit);
  void setTypeSupportResolveHook(std::function<void(const std::string &)> hook);
  void onPollTimer();
  void drainResponses();
  void checkTimeouts();
  void logPendingSettlementSummary(
    rclcpp::Logger logger,
    bool warn,
    const char * reason,
    const char * action,
    const char * service,
    const char * interface_type,
    const char * requester_identity,
    std::size_t count) const;

  template <typename ShouldSettleFn, typename MakeExceptionFn>
  std::size_t settlePendingCallsIf(ShouldSettleFn should_settle, MakeExceptionFn make_exception)
  {
    std::size_t settled_count = 0U;
    for (auto it = pending_calls.begin(); it != pending_calls.end();) {
      if (!should_settle(it->second)) {
        ++it;
        continue;
      }

      it = settlePendingCall(it, [&](PendingCall & call) { call.promise.set_exception(make_exception(call)); });
      ++settled_count;
    }
    return settled_count;
  }

  template <typename ShouldSettleFn>
  std::size_t settlePendingCallsIf(ShouldSettleFn should_settle, const char * message)
  {
    return settlePendingCallsIf(std::move(should_settle), [message](const PendingCall &) {
      return std::make_exception_ptr(std::runtime_error(message));
    });
  }

  template <typename SettlePromiseFn>
  PendingCallIterator settlePendingCall(PendingCallIterator it, SettlePromiseFn && settle_promise)
  {
    auto & call = it->second;
    std::forward<SettlePromiseFn>(settle_promise)(call);
    releaseInflight(call.requester_identity);
    return pending_calls.erase(it);
  }

  rclcpp::Node & node;
  std::unordered_map<std::string, std::unique_ptr<ServiceClientEntry>> clients;
  std::unordered_map<std::string, std::unique_ptr<ServiceClientEntry::ResolvedServiceTypeSupport>>
    resolved_type_supports;
  PendingCalls pending_calls;
  std::unordered_map<std::string, int> inflight;
  FailureCache invalid_service_type_cache{kInvalidServiceTypeCacheCapacity};
  rclcpp::TimerBase::SharedPtr poll_timer;
  std::mutex poll_callback_hook_mutex;
  std::mutex type_support_resolve_hook_mutex;
  // shutdown() disables new poll callbacks, then waits for any callback already
  // running on another thread to finish before tearing down clients or timers.
  ReentrantQuiesceGuard poll_callback_guard;
  std::function<void()> poll_callback_enter_hook;
  std::function<void()> poll_callback_exit_hook;
  std::function<void(const std::string &)> type_support_resolve_hook;
  bool shutdown_flag = false;
  EventThrottle late_response_drop_throttle{std::chrono::milliseconds(kResponseLogThrottleMs)};
};

std::string RosServiceCaller::Impl::resolveServiceType(
  const std::string & service, const std::string & requested_interface_type) const
{
  if (!requested_interface_type.empty()) {
    return requested_interface_type;
  }
  return requireUniqueInterfaceType(node.get_service_names_and_types(), service, "service");
}

ServiceClientEntry::ResolvedServiceTypeSupport & RosServiceCaller::Impl::getOrResolveTypeSupport(
  const std::string & service_type)
{
  auto it = resolved_type_supports.find(service_type);
  if (it != resolved_type_supports.end()) {
    return *it->second;
  }

  if (const auto failure = invalid_service_type_cache.get(service_type); failure.has_value()) {
    std::rethrow_exception(*failure);
  }

  std::function<void(const std::string &)> hook;
  {
    std::lock_guard<std::mutex> lock(type_support_resolve_hook_mutex);
    hook = type_support_resolve_hook;
  }
  if (hook) {
    hook(service_type);
  }

  try {
    auto resolved_type_support = std::make_unique<ServiceClientEntry::ResolvedServiceTypeSupport>(service_type);
    auto & ref = *resolved_type_support;
    resolved_type_supports.emplace(service_type, std::move(resolved_type_support));
    return ref;
  } catch (const std::invalid_argument &) {
    invalid_service_type_cache.insertOrAssign(service_type, std::current_exception());
    throw;
  } catch (const std::runtime_error &) {
    invalid_service_type_cache.insertOrAssign(service_type, std::current_exception());
    throw;
  }
}

ServiceClientEntry & RosServiceCaller::Impl::getOrCreateClient(
  const std::string & service, const std::string & service_type)
{
  const std::string key = service + ":" + service_type;
  auto it = clients.find(key);
  if (it != clients.end()) {
    return *it->second;
  }

  auto * rcl_node = node.get_node_base_interface()->get_rcl_node_handle();
  auto & resolved_type_support = getOrResolveTypeSupport(service_type);
  auto client = std::make_unique<ServiceClientEntry>(service, service_type, resolved_type_support, rcl_node);
  auto & ref = *client;
  clients.emplace(key, std::move(client));
  return ref;
}

void RosServiceCaller::Impl::acquireInflight(const std::string & requester_identity)
{
  if (requester_identity.empty()) {
    return;
  }
  const int current = inflight[requester_identity];
  if (current >= kMaxInflightPerRequester) {
    throw std::runtime_error("Requester identity service call limit reached.");
  }
  inflight[requester_identity] = current + 1;
}

void RosServiceCaller::Impl::releaseInflight(const std::string & requester_identity)
{
  if (requester_identity.empty()) {
    return;
  }
  auto it = inflight.find(requester_identity);
  if (it == inflight.end()) {
    return;
  }
  if (it->second <= 1) {
    inflight.erase(it);
    return;
  }
  it->second -= 1;
}

void RosServiceCaller::Impl::ensurePollTimer()
{
  if (poll_timer != nullptr) {
    return;
  }
  poll_timer = node.create_wall_timer(kPollInterval, [this]() { onPollTimer(); });
}

bool RosServiceCaller::Impl::beginPollCallback(std::function<void()> & on_enter)
{
  if (!poll_callback_guard.tryBeginWork()) {
    return false;
  }

  std::lock_guard<std::mutex> lock(poll_callback_hook_mutex);
  on_enter = poll_callback_enter_hook;
  return true;
}

void RosServiceCaller::Impl::endPollCallback()
{
  std::function<void()> on_exit;
  {
    std::lock_guard<std::mutex> lock(poll_callback_hook_mutex);
    on_exit = poll_callback_exit_hook;
  }

  if (on_exit) {
    on_exit();
  }
  poll_callback_guard.endWork();
}

void RosServiceCaller::Impl::quiescePollCallbacks()
{
  poll_callback_guard.disable();
  poll_callback_guard.quiesce();
}

void RosServiceCaller::Impl::setPollCallbackHooks(std::function<void()> on_enter, std::function<void()> on_exit)
{
  std::lock_guard<std::mutex> lock(poll_callback_hook_mutex);
  poll_callback_enter_hook = std::move(on_enter);
  poll_callback_exit_hook = std::move(on_exit);
}

void RosServiceCaller::Impl::setTypeSupportResolveHook(std::function<void(const std::string &)> hook)
{
  std::lock_guard<std::mutex> lock(type_support_resolve_hook_mutex);
  type_support_resolve_hook = std::move(hook);
}

void RosServiceCaller::Impl::logPendingSettlementSummary(
  rclcpp::Logger logger,
  bool warn,
  const char * reason,
  const char * action,
  const char * service,
  const char * interface_type,
  const char * requester_identity,
  std::size_t count) const
{
  if (count == 0U) {
    return;
  }

  if (warn) {
    LogEvent(logger, "service_calls_settled")
      .kv("reason", reason)
      .kv("action", action)
      .kv("service", service)
      .kv("interface_type", interface_type)
      .kv("requester_identity", requester_identity)
      .kv("count", count)
      .warn();
    return;
  }

  LogEvent(logger, "service_calls_settled")
    .kv("reason", reason)
    .kv("action", action)
    .kv("service", service)
    .kv("interface_type", interface_type)
    .kv("requester_identity", requester_identity)
    .kv("count", count)
    .info();
}

void RosServiceCaller::Impl::onPollTimer()
{
  std::function<void()> on_enter;
  if (!beginPollCallback(on_enter)) {
    return;
  }
  ScopeExit finish_poll([this]() { endPollCallback(); });

  if (on_enter) {
    on_enter();
  }

  drainResponses();
  checkTimeouts();

  if (pending_calls.empty() && poll_timer != nullptr) {
    poll_timer.reset();
  }
}

void RosServiceCaller::Impl::drainResponses()
{
  for (auto & [key, cached] : clients) {
    while (true) {
      DynamicMessageStorage response_storage(
        cached->resolved_type_support->response_type.members, rosidl_runtime_cpp::MessageInitialization::ZERO);
      rmw_request_id_t header{};
      rcl_ret_t ret = rcl_take_response(&cached->client, &header, response_storage.data());
      if (ret != RCL_RET_OK) {
        break;
      }

      // Match on both the client instance and sequence number so concurrent
      // callers to different services cannot steal each other's responses.
      const PendingCallKey pending_call_key{cached.get(), header.sequence_number};
      auto call_it = pending_calls.find(pending_call_key);
      if (call_it == pending_calls.end()) {
        if (const std::size_t count = late_response_drop_throttle.recordAndCheck(); count > 0U) {
          LogEvent(kRosServiceCallerLogger, "service_response_dropped")
            .kv("reason", "late_or_unknown_pending_call")
            .kv("service", cached->service_name)
            .kv("interface_type", cached->service_type_name)
            .kv("sequence_number", static_cast<long long>(header.sequence_number))
            .kv("count", count)
            .warn();
        }
        continue;
      }

      settlePendingCall(call_it, [&](PendingCall & call) {
        try {
          rclcpp::SerializedMessage serialized;
          cached->resolved_type_support->response_type.serialization.serialize_message(
            response_storage.data(), &serialized);
          call.promise.set_value(
            ServiceCallResponse{call.service, call.interface_type, serializedMessageBytes(serialized)});
        } catch (const std::exception & exc) {
          call.promise.set_exception(
            std::make_exception_ptr(
              std::runtime_error(std::string("Failed to convert service response: ") + exc.what())));
        }
      });
    }
  }
}

void RosServiceCaller::Impl::checkTimeouts()
{
  const auto now = std::chrono::steady_clock::now();
  const std::size_t timed_out_count =
    settlePendingCallsIf([now](const PendingCall & call) { return now >= call.deadline; }, "Service call timed out.");
  logPendingSettlementSummary(
    kRosServiceCallerLogger,
    true,
    "timeout",
    "fail_futures",
    kAnyServiceLogValue,
    kAnyServiceLogValue,
    kAnyServiceLogValue,
    timed_out_count);
}

RosServiceCaller::RosServiceCaller(rclcpp::Node & node)
: impl_(std::make_unique<Impl>(node))
{}

RosServiceCaller::~RosServiceCaller()
{
  if (impl_ != nullptr) {
    shutdown();
  }
}

std::future<RosServiceCaller::ServiceCallResponse> RosServiceCaller::call(
  const std::string & requester_identity, const ServiceCallRequest & request)
{
  std::promise<ServiceCallResponse> result_promise;
  auto result_future = result_promise.get_future();

  std::string interface_type;
  ServiceClientEntry * cached = nullptr;
  std::int64_t sequence_number = 0;
  std::optional<Impl::InflightReservation> inflight;

  try {
    if (impl_->shutdown_flag) {
      throw std::runtime_error("Service caller is shut down.");
    }
    if (requester_identity.empty()) {
      throw std::invalid_argument("requester_identity is required");
    }

    interface_type = impl_->resolveServiceType(request.service, request.interface_type);
    // Count the request against the requester before any expensive setup so the
    // limit covers work that is already being assembled for rcl_send_request().
    inflight.emplace(*impl_, requester_identity);

    try {
      cached = &impl_->getOrCreateClient(request.service, interface_type);
    } catch (const std::exception & exc) {
      throw std::runtime_error(std::string("Failed creating service client: ") + exc.what());
    }

    std::unique_ptr<DynamicMessageStorage> request_storage;
    try {
      auto serialized = toSerializedMessage(request.request);
      request_storage = std::make_unique<DynamicMessageStorage>(
        cached->resolved_type_support->request_type.members, rosidl_runtime_cpp::MessageInitialization::ZERO);
      cached->resolved_type_support->request_type.serialization.deserialize_message(
        &serialized, request_storage->data());
    } catch (const std::exception & exc) {
      throw std::runtime_error(std::string("Failed to build service request: ") + exc.what());
    }

    const rcl_ret_t ret = rcl_send_request(&cached->client, request_storage->data(), &sequence_number);
    if (ret != RCL_RET_OK) {
      throw std::runtime_error("Failed to send service request.");
    }
  } catch (const std::exception & exc) {
    if (inflight.has_value()) {
      LogEvent(kRosServiceCallerLogger, "service_call_failed")
        .kv("reason", "start_failed")
        .kv("service", request.service)
        .kv("interface_type", interface_type)
        .kv("requester_identity", requester_identity)
        .kv("action", "fail_future")
        .kv("error", exc.what())
        .error();
    }
    result_promise.set_exception(std::current_exception());
    return result_future;
  }

  const PendingCallKey pending_call_key{cached, sequence_number};
  if (impl_->pending_calls.find(pending_call_key) != impl_->pending_calls.end()) {
    LogEvent(kRosServiceCallerLogger, "service_call_failed")
      .kv("reason", "duplicate_pending_key")
      .kv("service", request.service)
      .kv("interface_type", interface_type)
      .kv("requester_identity", requester_identity)
      .kv("action", "fail_future")
      .error();
    result_promise.set_exception(std::make_exception_ptr(std::runtime_error("Duplicate pending service call key.")));
    return result_future;
  }

  const int timeout_ms = request.timeout_ms > 0 ? request.timeout_ms : kDefaultTimeoutMs;
  impl_->pending_calls.emplace(
    pending_call_key,
    Impl::PendingCall{
      request.service,
      interface_type,
      requester_identity,
      std::move(result_promise),
      std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms),
    });
  // From here on every settlement path goes through pending_calls, so ownership
  // of the inflight quota transfers from the local guard to that map entry.
  inflight->commit();

  impl_->ensurePollTimer();
  return result_future;
}

void RosServiceCaller::cancelCallsForRequester(const std::string & requester_identity)
{
  if (requester_identity.empty()) {
    return;
  }
  const std::size_t canceled_count = impl_->settlePendingCallsIf(
    [&requester_identity](const Impl::PendingCall & call) { return call.requester_identity == requester_identity; },
    "Requester identity disconnected.");
  impl_->logPendingSettlementSummary(
    kRosServiceCallerLogger,
    false,
    "requester_disconnected",
    "fail_futures",
    kAnyServiceLogValue,
    kAnyServiceLogValue,
    requester_identity.c_str(),
    canceled_count);
}

void RosServiceCaller::resetSessionState()
{
  const std::size_t canceled_count =
    impl_->settlePendingCallsIf([](const Impl::PendingCall &) { return true; }, "LiveKit session reset.");
  impl_->logPendingSettlementSummary(
    kRosServiceCallerLogger,
    false,
    "session_reset",
    "fail_futures",
    kAnyServiceLogValue,
    kAnyServiceLogValue,
    kAnyServiceLogValue,
    canceled_count);
  impl_->clients.clear();
  impl_->resolved_type_supports.clear();
}

void RosServiceCaller::shutdown()
{
  impl_->shutdown_flag = true;
  // The poll timer touches pending_calls and clients, so stop future callbacks
  // and wait for any active callback before tearing down shared state below.
  impl_->quiescePollCallbacks();

  if (impl_->poll_timer != nullptr) {
    impl_->poll_timer.reset();
  }

  const std::size_t canceled_count =
    impl_->settlePendingCallsIf([](const Impl::PendingCall &) { return true; }, "Service caller shut down.");
  impl_->logPendingSettlementSummary(
    kRosServiceCallerLogger,
    false,
    "shutdown",
    "fail_futures",
    kAnyServiceLogValue,
    kAnyServiceLogValue,
    kAnyServiceLogValue,
    canceled_count);
  impl_->clients.clear();
  impl_->resolved_type_supports.clear();
}

void RosServiceCaller::setPollCallbackHooksForTest(std::function<void()> on_enter, std::function<void()> on_exit)
{
  impl_->setPollCallbackHooks(std::move(on_enter), std::move(on_exit));
}

void RosServiceCaller::setTypeSupportResolveHookForTest(std::function<void(const std::string &)> hook)
{
  impl_->setTypeSupportResolveHook(std::move(hook));
}

}  // namespace livekit_ros2_bridge
