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
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <new>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rcl/client.h"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/typesupport_helpers.hpp"
#include "rclcpp/version.h"
#include "rcpputils/shared_library.hpp"
#include "rmw/types.h"
#include "rosidl_runtime_cpp/message_initialization.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "utils/event_throttle.hpp"
#include "utils/interface_type_utils.hpp"
#include "utils/log_event.hpp"
#include "utils/lru_cache.hpp"
#include "utils/reentrant_quiesce_gate.hpp"
#include "utils/scope_exit.hpp"
#include "utils/serialized_message.hpp"
#include "wire/services.hpp"

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

constexpr char kSerializationTypeSupportIdentifier[] = "rosidl_typesupport_cpp";
constexpr char kIntrospectionTypeSupportIdentifier[] = "rosidl_typesupport_introspection_cpp";
constexpr char kServiceTypeSupportSymbolPrefix[] = "__get_service_type_support_handle__";
constexpr char kRequestMessageTypeSuffix[] = "_Request";
constexpr char kResponseMessageTypeSuffix[] = "_Response";
constexpr auto kPollInterval = std::chrono::milliseconds(10);
constexpr auto kLogThrottle = std::chrono::seconds(5);
constexpr int kDefaultTimeoutMs = 2000;
constexpr int kMaxInflightPerRequester = 4;
constexpr std::size_t kInvalidServiceTypeCacheCapacity = 256U;
constexpr char kAnyServiceLogValue[] = "*";
constexpr char kInflightLimitReachedError[] = "Requester identity service call limit reached.";
const auto kLogger = rclcpp::get_logger("ros_service_caller");
using FailureCache = LruCache<std::string, std::exception_ptr>;

const MessageMembers & getMessageMembers(const rosidl_message_type_support_t * introspection_type_support)
{
  if (introspection_type_support == nullptr || introspection_type_support->data == nullptr) {
    throw std::runtime_error("Introspection type support handle is null");
  }
  return *static_cast<const MessageMembers *>(introspection_type_support->data);
}

void logServiceCallRejected(
  const ServiceCallRequest & request,
  const std::string & requester,
  const std::string & resolved_interface_type,
  const char * reason,
  const std::exception & exc,
  bool include_error = true)
{
  const std::string & logged_interface_type =
    resolved_interface_type.empty() ? request.interface_type : resolved_interface_type;
  LogEvent(kLogger, "service_call_rejected")
    .field("reason", reason)
    .fieldOr("service", request.service)
    .fieldIfNotEmpty("interface_type", logged_interface_type)
    .fieldOr("requester_identity", requester)
    .fieldIf(include_error, "error", exc.what())
    .warn();
}

class MessageStorage
{
public:
  // Runtime-discovered services only provide introspection callbacks, so we
  // manage a raw message buffer and pair the generated init/fini functions
  // explicitly instead of relying on a concrete generated C++ message type.
  MessageStorage(const MessageMembers & members, rosidl_runtime_cpp::MessageInitialization init)
  : members_(members)
  , data_(::operator new(members.size_of_))
  {
    members_.init_function(data_, init);
  }

  ~MessageStorage()
  {
    members_.fini_function(data_);
    ::operator delete(data_);
  }

  MessageStorage(const MessageStorage &) = delete;
  MessageStorage & operator=(const MessageStorage &) = delete;

  void * data()
  {
    return data_;
  }

private:
  const MessageMembers & members_;
  void * data_;
};

struct MessageTypeSupport
{
  // Keep both libraries alive for as long as the cached serialization and
  // introspection handles may be used by an active client entry.
  explicit MessageTypeSupport(const std::string & interface_type)
  : serialization_library(rclcpp::get_typesupport_library(interface_type, kSerializationTypeSupportIdentifier))
  , introspection_library(rclcpp::get_typesupport_library(interface_type, kIntrospectionTypeSupportIdentifier))
  , serialization_type_support(
      rclcpp::get_message_typesupport_handle(
        interface_type, kSerializationTypeSupportIdentifier, *serialization_library))
  , introspection_type_support(
      rclcpp::get_message_typesupport_handle(
        interface_type, kIntrospectionTypeSupportIdentifier, *introspection_library))
  , message_members(getMessageMembers(introspection_type_support))
  , serializer(serialization_type_support)
  {}

  std::shared_ptr<rcpputils::SharedLibrary> serialization_library;
  std::shared_ptr<rcpputils::SharedLibrary> introspection_library;
  const rosidl_message_type_support_t * serialization_type_support;
  const rosidl_message_type_support_t * introspection_type_support;
  const MessageMembers & message_members;
  rclcpp::SerializationBase serializer;
};

// rclcpp 28+ (Jazzy) provides get_service_typesupport_handle natively.
// On Humble we load the symbol manually.
#if RCLCPP_VERSION_GTE(28, 0, 0)

const rosidl_service_type_support_t * getServiceTypeSupportHandle(
  const std::string & service_type, const std::string & typesupport_identifier, rcpputils::SharedLibrary & library)
{
  return rclcpp::get_service_typesupport_handle(service_type, typesupport_identifier, library);
}

#else

const rosidl_service_type_support_t * getServiceTypeSupportHandle(
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

  using GetServiceTypeSupportHandleFn = const rosidl_service_type_support_t * (*)();
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  auto get_service_type_support_handle = reinterpret_cast<GetServiceTypeSupportHandleFn>(library.get_symbol(symbol));
  return get_service_type_support_handle();
}

#endif

struct CachedServiceClient
{
  struct TypeSupport
  {
    explicit TypeSupport(const std::string & interface_type)
    : library(rclcpp::get_typesupport_library(interface_type, kSerializationTypeSupportIdentifier))
    , service_type_support(getServiceTypeSupportHandle(interface_type, kSerializationTypeSupportIdentifier, *library))
    , request_type_support(interface_type + kRequestMessageTypeSuffix)
    , response_type_support(interface_type + kResponseMessageTypeSuffix)
    {}

    std::shared_ptr<rcpputils::SharedLibrary> library;
    const rosidl_service_type_support_t * service_type_support;
    MessageTypeSupport request_type_support;
    MessageTypeSupport response_type_support;
  };

  CachedServiceClient(
    const std::string & service_name,
    const std::string & interface_type,
    TypeSupport & type_support,
    rcl_node_t * node_handle)
  : service_name(service_name)
  , interface_type(interface_type)
  , type_support(&type_support)
  , client(rcl_get_zero_initialized_client())
  {
    rcl_client_options_t options = rcl_client_get_default_options();
    rcl_ret_t ret =
      rcl_client_init(&client, node_handle, type_support.service_type_support, service_name.c_str(), &options);
    if (ret != RCL_RET_OK) {
      throw std::runtime_error("Failed to create rcl service client for '" + service_name + "'");
    }
    node_handle_ = node_handle;
  }

  ~CachedServiceClient()
  {
    if (node_handle_ != nullptr) {
      rcl_ret_t ret = rcl_client_fini(&client, node_handle_);
      (void)ret;
    }
  }

  CachedServiceClient(const CachedServiceClient &) = delete;
  CachedServiceClient & operator=(const CachedServiceClient &) = delete;

  std::string service_name;
  std::string interface_type;
  TypeSupport * type_support = nullptr;
  rcl_client_t client;
  rcl_node_t * node_handle_ = nullptr;
};

struct InflightKey
{
  // Sequence numbers come from the underlying rcl client, so the client
  // instance is part of the key when matching responses back to inflight calls.
  const CachedServiceClient * client = nullptr;
  std::int64_t sequence_number = 0;

  bool operator==(const InflightKey & other) const
  {
    return client == other.client && sequence_number == other.sequence_number;
  }
};

struct InflightKeyHash
{
  std::size_t operator()(const InflightKey & key) const
  {
    return std::hash<const CachedServiceClient *>{}(key.client) ^
           (std::hash<std::int64_t>{}(key.sequence_number) << 1U);
  }
};

}  // namespace

struct RosServiceCaller::Impl
{
  struct InflightCall
  {
    std::string service;
    std::string interface_type;
    std::string requester;
    std::promise<ServiceCallResponse> promise;
    std::chrono::steady_clock::time_point deadline;
  };

  class InflightReservation
  {
  public:
    InflightReservation(Impl & impl, const std::string & requester)
    : impl_(impl)
    , requester_(requester)
    {
      impl_.reserveInflightSlot(requester_);
    }

    ~InflightReservation()
    {
      if (active_) {
        impl_.releaseInflightSlot(requester_);
      }
    }

    InflightReservation(const InflightReservation &) = delete;
    InflightReservation & operator=(const InflightReservation &) = delete;

    void transferToInflightCall()
    {
      active_ = false;
    }

  private:
    Impl & impl_;
    const std::string & requester_;
    bool active_ = true;
  };

  using InflightMap = std::unordered_map<InflightKey, InflightCall, InflightKeyHash>;
  using InflightIter = InflightMap::iterator;

  explicit Impl(ServiceNodeInterfaces interfaces)
  : interfaces(std::move(interfaces))
  {}

  CachedServiceClient::TypeSupport & getServiceTypeSupport(const std::string & interface_type);
  CachedServiceClient & getClient(const std::string & service, const std::string & interface_type);
  void reserveInflightSlot(const std::string & requester);
  void releaseInflightSlot(const std::string & requester);
  void poll();
  void drainResponses();
  void clearCachedServiceState();

  template <typename ShouldFailFn>
  void failMatchingCalls(
    ShouldFailFn should_fail,
    const char * exception_message,
    const char * reason,
    bool warn = false,
    const char * requester = kAnyServiceLogValue)
  {
    std::size_t count = 0U;
    for (auto it = inflight_calls.begin(); it != inflight_calls.end();) {
      if (!should_fail(it->second)) {
        ++it;
        continue;
      }

      it = settle(it, [exception_message](InflightCall & call) {
        call.promise.set_exception(std::make_exception_ptr(std::runtime_error(exception_message)));
      });
      ++count;
    }

    if (count == 0U) {
      return;
    }

    auto event = LogEvent(kLogger, "service_calls_settled")
                   .field("reason", reason)
                   .field("count", count)
                   .fieldIf(requester != kAnyServiceLogValue, "requester_identity", requester);
    if (warn) {
      event.warn();
      return;
    }
    event.info();
  }

  template <typename SettlePromiseFn>
  InflightIter settle(InflightIter it, SettlePromiseFn && settle_promise)
  {
    // Keep promise completion and quota release coupled so every terminal path
    // returns the requester's inflight slot exactly once.
    auto & call = it->second;
    std::forward<SettlePromiseFn>(settle_promise)(call);
    releaseInflightSlot(call.requester);
    return inflight_calls.erase(it);
  }

  ServiceNodeInterfaces interfaces;
  // CachedServiceClient stores a raw pointer into type_supports, so clients
  // must be destroyed before the backing type-support cache is cleared.
  std::unordered_map<std::string, std::unique_ptr<CachedServiceClient>> cached_clients;
  std::unordered_map<std::string, std::unique_ptr<CachedServiceClient::TypeSupport>> type_supports;
  InflightMap inflight_calls;
  std::unordered_map<std::string, int> inflight_counts;
  // Remember bad interface types so repeated invalid requests fail without
  // reloading type-support libraries every time.
  FailureCache type_support_failures{kInvalidServiceTypeCacheCapacity};
  rclcpp::TimerBase::SharedPtr poll_timer;
  std::mutex poll_callback_mutex;
  std::mutex type_support_load_callback_mutex;
  // shutdown() closes this gate and waits for any active poll() callback
  // before clearing timers or caches. Reentrancy matters because shutdown()
  // may be triggered from within poll().
  ReentrantQuiesceGate poll_gate;
  std::function<void()> on_poll_enter;
  std::function<void()> on_poll_exit;
  std::function<void(const std::string &)> on_type_support_load;
  EventThrottle late_response_throttle{kLogThrottle};
};

RosServiceCaller::RosServiceCaller(ServiceNodeInterfaces interfaces)
: impl_(std::make_unique<Impl>(std::move(interfaces)))
{}

RosServiceCaller::RosServiceCaller(rclcpp::Node & node)
: RosServiceCaller(makeRosNodeInterfaces(node).service())
{}

RosServiceCaller::~RosServiceCaller()
{
  shutdown();
}

// ros2.service.call spans both sides of the runtime boundary.
//
// Phase 1 happens here on the ROS executor: resolve the service type, create
// or reuse the rcl client, deserialize the request, and call
// rcl_send_request().
//
// Phase 2 happens later from poll(): call rcl_take_response(), match by
// client pointer and sequence number, fulfill the stored promise, and time
// out or cancel pending calls when needed.
//
// That split keeps executor-affine request creation safe without blocking the
// executor until the remote service replies. Immediate failures such as
// shutdown, bad requests, quota limits, or client creation errors fail here
// in phase 1. Later failures such as timeout, requester disconnect,
// connection reset, or shutdown while the call is inflight settle the stored
// promise from phase 2.
std::future<RosServiceCaller::ServiceCallResponse> RosServiceCaller::call(
  const std::string & requester, const ServiceCallRequest & request)
{
  std::promise<ServiceCallResponse> promise;
  auto future = promise.get_future();

  std::string interface_type;

  if (!impl_->poll_gate.isOpen()) {
    const std::runtime_error exc("Service caller is shut down.");
    logServiceCallRejected(request, requester, interface_type, "shutdown", exc, false);
    promise.set_exception(std::make_exception_ptr(exc));
    return future;
  }

  if (requester.empty()) {
    const std::invalid_argument exc("requester_identity is required");
    logServiceCallRejected(request, requester, interface_type, "missing_requester_identity", exc, false);
    promise.set_exception(std::make_exception_ptr(exc));
    return future;
  }

  try {
    try {
      interface_type = request.interface_type.empty()
                         ? requireSingleInterfaceType(
                             impl_->interfaces.graph->get_service_names_and_types(), request.service, "service")
                         : request.interface_type;
    } catch (const std::exception & exc) {
      logServiceCallRejected(request, requester, interface_type, "interface_type_resolution_failed", exc);
      throw;
    }
    // Reserve quota before client lookup, request deserialization, or send.
    // inflight_calls takes ownership only after the entry is inserted.
    Impl::InflightReservation inflight_reservation = [&]() -> Impl::InflightReservation {
      try {
        return Impl::InflightReservation(*impl_, requester);
      } catch (const std::runtime_error & exc) {
        logServiceCallRejected(request, requester, interface_type, "requester_inflight_limit_reached", exc, false);
        throw;
      }
    }();
    InflightKey key;
    try {
      CachedServiceClient * client = nullptr;
      try {
        client = &impl_->getClient(request.service, interface_type);
      } catch (const std::exception & exc) {
        throw std::runtime_error(std::string("Failed creating service client: ") + exc.what());
      }

      std::unique_ptr<MessageStorage> service_request;
      try {
        auto serialized = wrapSerializedPayload(request.request_payload);
        service_request = std::make_unique<MessageStorage>(
          client->type_support->request_type_support.message_members, rosidl_runtime_cpp::MessageInitialization::ZERO);
        client->type_support->request_type_support.serializer.deserialize_message(&serialized, service_request->data());
      } catch (const std::exception & exc) {
        throw std::runtime_error(std::string("Failed to build service request: ") + exc.what());
      }

      std::int64_t sequence_number = 0;
      const rcl_ret_t ret = rcl_send_request(&client->client, service_request->data(), &sequence_number);
      if (ret != RCL_RET_OK) {
        throw std::runtime_error("Failed to send service request.");
      }

      key = InflightKey{client, sequence_number};
    } catch (const std::exception & exc) {
      LogEvent(kLogger, "service_call_failed")
        .field("reason", "start_failed")
        .fieldOr("service", request.service)
        .fieldIfNotEmpty("interface_type", interface_type)
        .fieldOr("requester_identity", requester)
        .field("error", exc.what())
        .error();
      throw;
    }

    if (impl_->inflight_calls.find(key) != impl_->inflight_calls.end()) {
      LogEvent(kLogger, "service_call_failed")
        .field("reason", "duplicate_pending_key")
        .fieldOr("service", request.service)
        .fieldIfNotEmpty("interface_type", interface_type)
        .fieldOr("requester_identity", requester)
        .error();
      throw std::runtime_error("Duplicate pending service call key.");
    }

    const int timeout_ms =
      request.timeout_ms.has_value() && *request.timeout_ms > 0 ? *request.timeout_ms : kDefaultTimeoutMs;

    LogEvent(kLogger, "service_call_started")
      .fieldOr("service", request.service)
      .fieldIfNotEmpty("interface_type", interface_type)
      .fieldOr("requester_identity", requester)
      .info();

    impl_->inflight_calls.emplace(
      key,
      Impl::InflightCall{
        request.service,
        interface_type,
        requester,
        std::move(promise),
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms),
      });
    // From here on every settle path goes through inflight_calls, so that entry
    // owns releasing the requester's inflight quota.
    inflight_reservation.transferToInflightCall();
  } catch (const std::exception &) {
    promise.set_exception(std::current_exception());
    return future;
  }

  if (impl_->poll_timer == nullptr) {
    auto * impl = impl_.get();
    impl_->poll_timer = rclcpp::create_wall_timer(
      kPollInterval, [impl]() { impl->poll(); }, nullptr, impl_->interfaces.base.get(), impl_->interfaces.timers.get());
  }
  return future;
}

void RosServiceCaller::cancelCallsForRequester(const std::string & requester)
{
  if (requester.empty()) {
    return;
  }
  impl_->failMatchingCalls(
    [&requester](const Impl::InflightCall & call) { return call.requester == requester; },
    "Requester identity disconnected.",
    "requester_disconnected",
    false,
    requester.c_str());
}

void RosServiceCaller::resetSessionState()
{
  impl_->failMatchingCalls([](const Impl::InflightCall &) { return true; }, "LiveKit session reset.", "session_reset");

  impl_->clearCachedServiceState();
}

void RosServiceCaller::shutdown()
{
  impl_->poll_gate.close();
  impl_->poll_gate.awaitIdle();

  impl_->poll_timer.reset();

  impl_->failMatchingCalls([](const Impl::InflightCall &) { return true; }, "Service caller shut down.", "shutdown");

  impl_->clearCachedServiceState();
}

void RosServiceCaller::setPollCallbacksForTest(std::function<void()> on_poll_enter, std::function<void()> on_poll_exit)
{
  std::lock_guard<std::mutex> lock(impl_->poll_callback_mutex);
  impl_->on_poll_enter = std::move(on_poll_enter);
  impl_->on_poll_exit = std::move(on_poll_exit);
}

void RosServiceCaller::setTypeSupportLoadCallbackForTest(std::function<void(const std::string &)> on_type_support_load)
{
  std::lock_guard<std::mutex> lock(impl_->type_support_load_callback_mutex);
  impl_->on_type_support_load = std::move(on_type_support_load);
}

CachedServiceClient::TypeSupport & RosServiceCaller::Impl::getServiceTypeSupport(const std::string & interface_type)
{
  auto it = type_supports.find(interface_type);
  if (it != type_supports.end()) {
    return *it->second;
  }

  if (const auto failure = type_support_failures.get(interface_type); failure.has_value()) {
    std::rethrow_exception(*failure);
  }

  std::function<void(const std::string &)> on_type_support_load;
  {
    std::lock_guard<std::mutex> lock(type_support_load_callback_mutex);
    on_type_support_load = this->on_type_support_load;
  }
  if (on_type_support_load) {
    on_type_support_load(interface_type);
  }

  try {
    auto type_support = std::make_unique<CachedServiceClient::TypeSupport>(interface_type);
    auto & ref = *type_support;
    type_supports.emplace(interface_type, std::move(type_support));
    return ref;
  } catch (const std::exception & exc) {
    if (
      dynamic_cast<const std::invalid_argument *>(&exc) != nullptr ||
      dynamic_cast<const std::runtime_error *>(&exc) != nullptr)
    {
      type_support_failures.insertOrAssign(interface_type, std::current_exception());
    }
    throw;
  }
}

CachedServiceClient & RosServiceCaller::Impl::getClient(const std::string & service, const std::string & interface_type)
{
  const std::string key = service + ":" + interface_type;
  auto it = cached_clients.find(key);
  if (it != cached_clients.end()) {
    return *it->second;
  }

  auto * rcl_node = interfaces.base->get_rcl_node_handle();
  auto & type_support = getServiceTypeSupport(interface_type);
  auto entry = std::make_unique<CachedServiceClient>(service, interface_type, type_support, rcl_node);
  auto & ref = *entry;
  cached_clients.emplace(key, std::move(entry));
  return ref;
}

void RosServiceCaller::Impl::reserveInflightSlot(const std::string & requester)
{
  if (requester.empty()) {
    return;
  }
  const int current = inflight_counts[requester];
  if (current >= kMaxInflightPerRequester) {
    throw std::runtime_error(kInflightLimitReachedError);
  }
  inflight_counts[requester] = current + 1;
}

void RosServiceCaller::Impl::releaseInflightSlot(const std::string & requester)
{
  if (requester.empty()) {
    return;
  }
  auto it = inflight_counts.find(requester);
  if (it == inflight_counts.end()) {
    return;
  }
  if (it->second <= 1) {
    inflight_counts.erase(it);
    return;
  }
  it->second -= 1;
}

void RosServiceCaller::Impl::poll()
{
  if (!poll_gate.tryEnter()) {
    return;
  }

  std::function<void()> on_enter;
  std::function<void()> on_exit;
  {
    // Callbacks are copied under the mutex but invoked without it so test callbacks can
    // reenter RosServiceCaller, including calling shutdown() from poll().
    std::lock_guard<std::mutex> lock(poll_callback_mutex);
    on_enter = this->on_poll_enter;
    on_exit = this->on_poll_exit;
  }
  ScopeExit finish_poll([this, on_exit = std::move(on_exit)]() mutable {
    if (on_exit) {
      on_exit();
    }
    poll_gate.leave();
  });

  if (on_enter) {
    on_enter();
  }

  drainResponses();
  const auto now = std::chrono::steady_clock::now();
  failMatchingCalls(
    [now](const InflightCall & call) { return now >= call.deadline; }, "Service call timed out.", "timeout", true);

  if (!inflight_calls.empty()) {
    return;
  }

  poll_timer.reset();
}

void RosServiceCaller::Impl::drainResponses()
{
  for (auto & [cache_key, entry] : cached_clients) {
    (void)cache_key;
    while (true) {
      MessageStorage response(
        entry->type_support->response_type_support.message_members, rosidl_runtime_cpp::MessageInitialization::ZERO);
      rmw_request_id_t header{};
      rcl_ret_t ret = rcl_take_response(&entry->client, &header, response.data());
      if (ret != RCL_RET_OK) {
        break;
      }

      // rcl sequence numbers are scoped to a client instance, so settle by
      // both the client pointer and sequence number.
      const InflightKey key{entry.get(), header.sequence_number};
      auto it = inflight_calls.find(key);
      if (it == inflight_calls.end()) {
        // The original call already timed out, was cancelled, or belonged to a
        // reset session. Never match a late response by service name alone,
        // because a newer call on the same service may now be inflight.
        if (const std::size_t count = late_response_throttle.recordAndTakePendingCount(); count > 0U) {
          LogEvent(kLogger, "service_response_dropped")
            .field("reason", "late_or_unknown_pending_call")
            .field("service", entry->service_name)
            .field("interface_type", entry->interface_type)
            .field("count", count)
            .warn();
        }
        continue;
      }

      settle(it, [&](InflightCall & call) {
        try {
          rclcpp::SerializedMessage serialized;
          entry->type_support->response_type_support.serializer.serialize_message(response.data(), &serialized);
          std::vector<std::uint8_t> response_payload;
          const auto & serialized_response = serialized.get_rcl_serialized_message();
          if (serialized_response.buffer != nullptr && serialized_response.buffer_length > 0U) {
            response_payload.assign(
              serialized_response.buffer, serialized_response.buffer + serialized_response.buffer_length);
          }
          call.promise.set_value(ServiceCallResponse{call.service, call.interface_type, std::move(response_payload)});
        } catch (const std::exception & exc) {
          call.promise.set_exception(
            std::make_exception_ptr(
              std::runtime_error(std::string("Failed to convert service response: ") + exc.what())));
        }
      });
    }
  }
}

void RosServiceCaller::Impl::clearCachedServiceState()
{
  // CachedServiceClient stores raw pointers into type_supports, so clients
  // must be destroyed before the backing type-support cache is cleared.
  cached_clients.clear();
  type_supports.clear();
}

}  // namespace livekit_ros2_bridge
