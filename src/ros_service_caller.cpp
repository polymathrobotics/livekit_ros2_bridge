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
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <new>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rcl/client.h"
#include "rcl/error_handling.h"
#include "rcl/timer.h"
#include "rcl/wait.h"
#include "rclcpp/clock.hpp"
#include "rclcpp/guard_condition.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/typesupport_helpers.hpp"
#include "rclcpp/version.h"
#include "rclcpp/waitable.hpp"
#include "rcpputils/shared_library.hpp"
#include "rmw/types.h"
#include "rosidl_runtime_cpp/message_initialization.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "utils/event_throttle.hpp"
#include "utils/interface_type_utils.hpp"
#include "utils/log_event.hpp"
#include "utils/lru_cache.hpp"
#include "utils/scope_exit.hpp"
#include "utils/serialized_message.hpp"

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
constexpr auto kLogThrottle = std::chrono::seconds(5);
constexpr int kDefaultTimeoutMs = 2000;
constexpr int kMaxInflightPerRequester = 4;
constexpr std::size_t kInvalidServiceTypeCacheCapacity = 256U;
constexpr std::size_t kMaxCachedServiceClients = 256U;
constexpr char kAnyServiceLogValue[] = "*";
constexpr char kInflightLimitReachedError[] = "Requester identity service call limit reached.";
constexpr int kReadyEntityId = 0;
const auto kLogger = rclcpp::get_logger("ros_service_caller");
using FailureCache = LruCache<std::string, std::exception_ptr>;

[[noreturn]] void throwRclError(const std::string & action)
{
  std::string message = action + ": " + rcl_get_error_string().str;
  rcl_reset_error();
  throw std::runtime_error(message);
}

void deadlineTimerCallback(rcl_timer_t *, int64_t)
{}

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
  , serialization(
      rclcpp::get_message_typesupport_handle(
        interface_type, kSerializationTypeSupportIdentifier, *serialization_library))
  , introspection(
      rclcpp::get_message_typesupport_handle(
        interface_type, kIntrospectionTypeSupportIdentifier, *introspection_library))
  , members(getMessageMembers(introspection))
  , serializer(serialization)
  {}

  std::shared_ptr<rcpputils::SharedLibrary> serialization_library;
  std::shared_ptr<rcpputils::SharedLibrary> introspection_library;
  const rosidl_message_type_support_t * serialization;
  const rosidl_message_type_support_t * introspection;
  const MessageMembers & members;
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
      continue;
    }
    symbol += ch;
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

struct ServiceClient
{
  struct TypeSupport
  {
    explicit TypeSupport(const std::string & interface_type)
    : library(rclcpp::get_typesupport_library(interface_type, kSerializationTypeSupportIdentifier))
    , service(getServiceTypeSupportHandle(interface_type, kSerializationTypeSupportIdentifier, *library))
    , request(interface_type + kRequestMessageTypeSuffix)
    , response(interface_type + kResponseMessageTypeSuffix)
    {}

    std::shared_ptr<rcpputils::SharedLibrary> library;
    const rosidl_service_type_support_t * service;
    MessageTypeSupport request;
    MessageTypeSupport response;
  };

  ServiceClient(
    const std::string & service_name,
    const std::string & interface_type,
    std::shared_ptr<TypeSupport> support,
    rcl_node_t * node_handle)
  : service_name(service_name)
  , interface_type(interface_type)
  , support(std::move(support))
  , client(rcl_get_zero_initialized_client())
  {
    rcl_client_options_t options = rcl_client_get_default_options();
    rcl_ret_t ret = rcl_client_init(&client, node_handle, this->support->service, service_name.c_str(), &options);
    if (ret != RCL_RET_OK) {
      throw std::runtime_error("Failed to create rcl service client for '" + service_name + "'");
    }
    node_handle_ = node_handle;
  }

  ~ServiceClient()
  {
    if (node_handle_ == nullptr) {
      return;
    }

    rcl_ret_t ret = rcl_client_fini(&client, node_handle_);
    (void)ret;
  }

  ServiceClient(const ServiceClient &) = delete;
  ServiceClient & operator=(const ServiceClient &) = delete;

  std::string service_name;
  std::string interface_type;
  std::shared_ptr<TypeSupport> support;
  rcl_client_t client;
  rcl_node_t * node_handle_ = nullptr;
};

struct InflightKey
{
  // Sequence numbers come from the underlying rcl client, so the client
  // instance is part of the key when matching responses back to inflight calls.
  const ServiceClient * client = nullptr;
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
    return std::hash<const ServiceClient *>{}(key.client) ^ (std::hash<std::int64_t>{}(key.sequence_number) << 1U);
  }
};

}  // namespace

struct RosServiceCaller::Impl
{
  struct InflightCall
  {
    // Keeps the rcl client and its type support alive until the call settles,
    // even if resetSessionState() drops the reusable client cache first.
    std::shared_ptr<ServiceClient> client;
    std::string service;
    std::string interface_type;
    std::string requester;
    std::promise<Response> promise;
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

  class ServiceWaitable;

  using ClientPtr = std::shared_ptr<ServiceClient>;
  using TypeSupportPtr = std::shared_ptr<ServiceClient::TypeSupport>;
  using InflightMap = std::unordered_map<InflightKey, InflightCall, InflightKeyHash>;
  using InflightIter = InflightMap::iterator;

  Impl(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph,
    rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr waitables);

  TypeSupportPtr getTypeSupport(const std::string & interface_type);
  ClientPtr getClient(const std::string & service, const std::string & interface_type);
  void reserveInflightSlot(const std::string & requester);
  void releaseInflightSlot(const std::string & requester);
  void executeWaitable();
  void drainResponses();
  void clearCacheLocked();
  void syncWaitableLocked(bool wake = true);
  void detachWaitable();
  std::optional<std::chrono::steady_clock::time_point> earliestDeadlineLocked() const;
  bool isWaitableOpen();
  void closeWaitable();
  bool tryBeginWaitableCallback();
  void finishWaitableCallback();
  void awaitWaitableIdle();

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

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph;
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr waitables;
  rclcpp::CallbackGroup::SharedPtr callback_group;
  std::shared_ptr<ServiceWaitable> waitable;
  std::unordered_map<std::string, ClientPtr> cached_clients;
  std::unordered_map<std::string, TypeSupportPtr> type_supports;
  InflightMap inflight_calls;
  std::unordered_map<std::string, int> inflight_counts;
  // Remember bad interface types so repeated invalid requests fail without
  // reloading type-support libraries every time.
  FailureCache type_support_failures{kInvalidServiceTypeCacheCapacity};
  std::mutex state_mutex;
  std::mutex waitable_callback_mutex;
  std::mutex type_support_load_callback_mutex;
  // shutdown() closes waitable entry and waits for any active waitable
  // callback before clearing clients or type support. Reentrancy matters
  // because shutdown() may be triggered from within executeWaitable().
  std::mutex waitable_lifecycle_mutex;
  std::condition_variable waitable_idle;
  bool waitable_closed = false;
  bool waitable_active = false;
  std::thread::id waitable_owner_thread_id{};
  std::function<void()> on_waitable_enter;
  std::function<void()> on_waitable_exit;
  std::function<void(const std::string &)> on_type_support_load;
  EventThrottle late_response_throttle{kLogThrottle};
};

class RosServiceCaller::Impl::ServiceWaitable final : public rclcpp::Waitable
{
public:
  ServiceWaitable(Impl & impl, const rclcpp::Context::SharedPtr & context)
  : impl_(impl)
  , context_(context)
  , guard_condition_(std::make_shared<rclcpp::GuardCondition>(context))
  , deadline_clock_(std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME))
  , deadline_timer_(rcl_get_zero_initialized_timer())
  {
    // Jazzy and newer expose rcl_timer_init2() with an explicit autostart
    // flag; Humble only has the original rcl_timer_init().
#if RCLCPP_VERSION_GTE(28, 0, 0)
    const rcl_ret_t init_ret = rcl_timer_init2(
      &deadline_timer_,
      deadline_clock_->get_clock_handle(),
      context_->get_rcl_context().get(),
      std::chrono::nanoseconds(1).count(),
      deadlineTimerCallback,
      rcl_get_default_allocator(),
      false);
#else
    const rcl_ret_t init_ret = rcl_timer_init(
      &deadline_timer_,
      deadline_clock_->get_clock_handle(),
      context_->get_rcl_context().get(),
      std::chrono::nanoseconds(1).count(),
      deadlineTimerCallback,
      rcl_get_default_allocator());
#endif
    if (init_ret != RCL_RET_OK) {
      throwRclError("Failed to initialize service deadline timer");
    }

    const rcl_ret_t cancel_ret = rcl_timer_cancel(&deadline_timer_);
    if (cancel_ret != RCL_RET_OK) {
      throwRclError("Failed to cancel initial service deadline timer");
    }
  }

  ~ServiceWaitable() override
  {
    std::lock_guard<std::mutex> lock(timer_mutex_);
    const rcl_ret_t ret = rcl_timer_fini(&deadline_timer_);
    (void)ret;
  }

  ServiceWaitable(const ServiceWaitable &) = delete;
  ServiceWaitable & operator=(const ServiceWaitable &) = delete;

// Jazzy and newer changed Waitable overrides from wait-set pointers to
// references; Humble still uses pointer-based signatures.
#if RCLCPP_VERSION_GTE(28, 0, 0)
  void add_to_wait_set(rcl_wait_set_t & wait_set) override
  {
    addToWaitSet(&wait_set);
  }

  bool is_ready(const rcl_wait_set_t & wait_set) override
  {
    return isReady(&wait_set);
  }

  void execute(const std::shared_ptr<void> & ignored_data) override
  {
    (void)ignored_data;
    executeReady();
  }
#else
  void add_to_wait_set(rcl_wait_set_t * wait_set) override
  {
    addToWaitSet(wait_set);
  }

  bool is_ready(rcl_wait_set_t * wait_set) override
  {
    return isReady(wait_set);
  }

  void execute(std::shared_ptr<void> & ignored_data) override
  {
    (void)ignored_data;
    executeReady();
  }
#endif

  std::shared_ptr<void> take_data() override
  {
    return nullptr;
  }

  std::shared_ptr<void> take_data_by_entity_id(size_t entity_id) override
  {
    (void)entity_id;
    return nullptr;
  }

  size_t get_number_of_ready_clients() override
  {
    // Kilted and newer reuse the wait-set capacity after the waitable is
    // attached, so this must report the bounded capacity rather than the
    // current snapshot size. getClient() enforces the same cap.
    return kMaxCachedServiceClients;
  }

  size_t get_number_of_ready_timers() override
  {
    return 1U;
  }

  size_t get_number_of_ready_guard_conditions() override
  {
    return 1U;
  }

  void set_on_ready_callback(std::function<void(size_t, int)> on_ready) override
  {
    std::function<void(size_t, int)> callback;
    size_t pending_wakes = 0U;

    {
      std::lock_guard<std::mutex> lock(callback_mutex_);
      on_ready_ = std::move(on_ready);
      callback = on_ready_;
      if (callback == nullptr) {
        return;
      }

      pending_wakes = std::exchange(pending_wakes_, 0U);
    }

    if (pending_wakes == 0U) {
      return;
    }

    callback(pending_wakes, kReadyEntityId);
  }

  void clear_on_ready_callback() override
  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    on_ready_ = nullptr;
  }

  std::vector<std::shared_ptr<rclcpp::TimerBase>> get_timers() const
  {
    return {};
  }

  void setClients(std::vector<ClientPtr> clients)
  {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    // Do not mutate clients_ from caller threads. rcl_wait_set_t only stores
    // raw rcl_client_t pointers, so the executor applies pending snapshots
    // after the current wait returns.
    pending_clients_.emplace(std::move(clients));
  }

  void setDeadline(std::optional<std::chrono::steady_clock::time_point> deadline)
  {
    std::lock_guard<std::mutex> lock(timer_mutex_);
    if (!deadline.has_value()) {
      const rcl_ret_t ret = rcl_timer_cancel(&deadline_timer_);
      if (ret != RCL_RET_OK) {
        throwRclError("Failed to cancel service deadline timer");
      }
      return;
    }

    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(*deadline - std::chrono::steady_clock::now());
    if (duration < std::chrono::nanoseconds(1)) {
      duration = std::chrono::nanoseconds(1);
    }

    int64_t old_period = 0;
    rcl_ret_t ret = rcl_timer_exchange_period(&deadline_timer_, duration.count(), &old_period);
    if (ret != RCL_RET_OK) {
      throwRclError("Failed to update service deadline timer period");
    }

    ret = rcl_timer_reset(&deadline_timer_);
    if (ret != RCL_RET_OK) {
      throwRclError("Failed to reset service deadline timer");
    }
  }

  void wake()
  {
    guard_condition_->trigger();
    std::function<void(size_t, int)> callback;

    {
      std::lock_guard<std::mutex> lock(callback_mutex_);
      callback = on_ready_;
      if (callback == nullptr) {
        ++pending_wakes_;
        return;
      }
    }

    callback(1U, kReadyEntityId);
  }

private:
  void addToWaitSet(rcl_wait_set_t * wait_set)
  {
    if (wait_set == nullptr) {
      throw std::invalid_argument("wait set is null");
    }

    {
      std::lock_guard<std::mutex> lock(clients_mutex_);
      for (const auto & client : clients_) {
        const rcl_ret_t ret = rcl_wait_set_add_client(wait_set, &client->client, nullptr);
        if (ret != RCL_RET_OK) {
          throwRclError("Failed to add service client to wait set");
        }
      }
    }

    {
      std::lock_guard<std::mutex> lock(timer_mutex_);
      const rcl_ret_t ret = rcl_wait_set_add_timer(wait_set, &deadline_timer_, nullptr);
      if (ret != RCL_RET_OK) {
        throwRclError("Failed to add service deadline timer to wait set");
      }
    }

    // GuardCondition follows the same wait-set pointer/reference split as
    // Waitable across the supported distros.
#if RCLCPP_VERSION_GTE(28, 0, 0)
    guard_condition_->add_to_wait_set(*wait_set);
#else
    guard_condition_->add_to_wait_set(wait_set);
#endif
  }

  bool isReady(const rcl_wait_set_t * wait_set)
  {
    if (wait_set == nullptr) {
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(clients_mutex_);
      for (size_t i = 0; i < wait_set->size_of_clients; ++i) {
        const auto * ready_client = wait_set->clients[i];
        if (ready_client == nullptr) {
          continue;
        }
        const auto ready_it = std::find_if(clients_.begin(), clients_.end(), [ready_client](const ClientPtr & client) {
          return &client->client == ready_client;
        });
        if (ready_it != clients_.end()) {
          return true;
        }
      }
    }

    for (size_t i = 0; i < wait_set->size_of_timers; ++i) {
      if (wait_set->timers[i] == &deadline_timer_) {
        return true;
      }
    }

    const auto * rcl_guard_condition = &guard_condition_->get_rcl_guard_condition();
    for (size_t i = 0; i < wait_set->size_of_guard_conditions; ++i) {
      if (wait_set->guard_conditions[i] == rcl_guard_condition) {
        return true;
      }
    }

    return false;
  }

  void executeReady()
  {
    applyPendingClients();
    callReadyDeadlineTimer();
    impl_.executeWaitable();
  }

  void applyPendingClients()
  {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    if (!pending_clients_.has_value()) {
      return;
    }

    clients_ = std::move(*pending_clients_);
    pending_clients_.reset();
  }

  void callReadyDeadlineTimer()
  {
    std::lock_guard<std::mutex> lock(timer_mutex_);

    bool ready = false;
    rcl_ret_t ret = rcl_timer_is_ready(&deadline_timer_, &ready);
    if (ret == RCL_RET_TIMER_CANCELED) {
      rcl_reset_error();
      return;
    }
    if (ret != RCL_RET_OK) {
      throwRclError("Failed to check service deadline timer readiness");
    }
    if (!ready) {
      return;
    }

    ret = rcl_timer_call(&deadline_timer_);
    if (ret == RCL_RET_TIMER_CANCELED) {
      rcl_reset_error();
      return;
    }
    if (ret != RCL_RET_OK) {
      throwRclError("Failed to consume service deadline timer");
    }
  }

  Impl & impl_;
  rclcpp::Context::SharedPtr context_;
  rclcpp::GuardCondition::SharedPtr guard_condition_;
  rclcpp::Clock::SharedPtr deadline_clock_;
  rcl_timer_t deadline_timer_;
  std::mutex clients_mutex_;
  std::vector<ClientPtr> clients_;
  std::optional<std::vector<ClientPtr>> pending_clients_;
  std::mutex timer_mutex_;
  std::mutex callback_mutex_;
  std::function<void(size_t, int)> on_ready_;
  size_t pending_wakes_ = 0U;
};

RosServiceCaller::Impl::Impl(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base,
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph,
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr waitables)
: base(std::move(base))
, graph(std::move(graph))
, waitables(std::move(waitables))
, callback_group(this->base->get_default_callback_group())
{
  waitable = std::make_shared<ServiceWaitable>(*this, this->base->get_context());
  this->waitables->add_waitable(waitable, callback_group);
}

RosServiceCaller::RosServiceCaller(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base,
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph,
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr waitables)
: impl_(std::make_unique<Impl>(std::move(base), std::move(graph), std::move(waitables)))
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
// Phase 2 happens later from the service waitable: call rcl_take_response(),
// match by client pointer and sequence number, fulfill the stored promise,
// and time out or cancel pending calls when needed.
//
// That split keeps executor-affine request creation safe without blocking the
// executor until the remote service replies. Immediate failures such as
// shutdown, bad requests, quota limits, or client creation errors fail here
// in phase 1. Later failures such as timeout, requester disconnect,
// connection reset, or shutdown while the call is inflight settle the stored
// promise from phase 2.
std::future<RosServiceCaller::Response> RosServiceCaller::call(
  const std::string & requester, const ServiceCallRequest & request)
{
  std::promise<Response> promise;
  auto future = promise.get_future();

  std::string interface_type;

  if (!impl_->isWaitableOpen()) {
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
      interface_type = request.interface_type;
      if (interface_type.empty()) {
        interface_type =
          requireSingleInterfaceType(impl_->graph->get_service_names_and_types(), request.service, "service");
      }
    } catch (const std::exception & exc) {
      logServiceCallRejected(request, requester, interface_type, "interface_type_resolution_failed", exc);
      throw;
    }

    std::lock_guard<std::mutex> state_lock(impl_->state_mutex);
    if (!impl_->isWaitableOpen()) {
      const std::runtime_error exc("Service caller is shut down.");
      logServiceCallRejected(request, requester, interface_type, "shutdown", exc, false);
      throw exc;
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
    Impl::ClientPtr client;
    try {
      try {
        client = impl_->getClient(request.service, interface_type);
      } catch (const std::exception & exc) {
        throw std::runtime_error(std::string("Failed creating service client: ") + exc.what());
      }

      std::unique_ptr<MessageStorage> body;
      try {
        auto serialized = wrapSerializedPayload(request.payload);
        body = std::make_unique<MessageStorage>(
          client->support->request.members, rosidl_runtime_cpp::MessageInitialization::ZERO);
        client->support->request.serializer.deserialize_message(&serialized, body->data());
      } catch (const std::exception & exc) {
        throw std::runtime_error(std::string("Failed to build service request: ") + exc.what());
      }

      std::int64_t sequence_number = 0;
      const rcl_ret_t ret = rcl_send_request(&client->client, body->data(), &sequence_number);
      if (ret != RCL_RET_OK) {
        throw std::runtime_error("Failed to send service request.");
      }

      key = InflightKey{client.get(), sequence_number};
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
        client,
        request.service,
        interface_type,
        requester,
        std::move(promise),
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms),
      });
    // From here on every settle path goes through inflight_calls, so that entry
    // owns releasing the requester's inflight quota.
    inflight_reservation.transferToInflightCall();
    impl_->syncWaitableLocked();
  } catch (const std::exception &) {
    promise.set_exception(std::current_exception());
    return future;
  }

  return future;
}

void RosServiceCaller::cancelForRequester(const std::string & requester)
{
  if (requester.empty()) {
    return;
  }
  std::lock_guard<std::mutex> state_lock(impl_->state_mutex);
  impl_->failMatchingCalls(
    [&requester](const Impl::InflightCall & call) { return call.requester == requester; },
    "Requester identity disconnected.",
    "requester_disconnected",
    false,
    requester.c_str());
  impl_->syncWaitableLocked();
}

void RosServiceCaller::resetSessionState()
{
  std::lock_guard<std::mutex> state_lock(impl_->state_mutex);
  impl_->failMatchingCalls([](const Impl::InflightCall &) { return true; }, "LiveKit session reset.", "session_reset");

  impl_->clearCacheLocked();
}

void RosServiceCaller::shutdown()
{
  impl_->closeWaitable();
  impl_->awaitWaitableIdle();
  impl_->detachWaitable();

  std::lock_guard<std::mutex> state_lock(impl_->state_mutex);
  impl_->failMatchingCalls([](const Impl::InflightCall &) { return true; }, "Service caller shut down.", "shutdown");

  impl_->clearCacheLocked();
}

void RosServiceCaller::setWaitableCallbacksForTest(
  std::function<void()> on_waitable_enter, std::function<void()> on_waitable_exit)
{
  std::lock_guard<std::mutex> lock(impl_->waitable_callback_mutex);
  impl_->on_waitable_enter = std::move(on_waitable_enter);
  impl_->on_waitable_exit = std::move(on_waitable_exit);
}

void RosServiceCaller::setTypeSupportLoadCallbackForTest(std::function<void(const std::string &)> on_type_support_load)
{
  std::lock_guard<std::mutex> lock(impl_->type_support_load_callback_mutex);
  impl_->on_type_support_load = std::move(on_type_support_load);
}

RosServiceCaller::Impl::TypeSupportPtr RosServiceCaller::Impl::getTypeSupport(const std::string & interface_type)
{
  auto it = type_supports.find(interface_type);
  if (it != type_supports.end()) {
    return it->second;
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
    return type_supports.emplace(interface_type, std::make_shared<ServiceClient::TypeSupport>(interface_type))
      .first->second;
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

RosServiceCaller::Impl::ClientPtr RosServiceCaller::Impl::getClient(
  const std::string & service, const std::string & interface_type)
{
  const std::string key = service + ":" + interface_type;
  auto it = cached_clients.find(key);
  if (it != cached_clients.end()) {
    return it->second;
  }

  if (cached_clients.size() >= kMaxCachedServiceClients) {
    throw std::runtime_error("Service client cache limit reached.");
  }

  auto * rcl_node = base->get_rcl_node_handle();
  auto support = getTypeSupport(interface_type);
  auto entry = std::make_shared<ServiceClient>(service, interface_type, support, rcl_node);
  return cached_clients.emplace(key, std::move(entry)).first->second;
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

void RosServiceCaller::Impl::executeWaitable()
{
  if (!tryBeginWaitableCallback()) {
    return;
  }

  std::function<void()> on_enter;
  std::function<void()> on_exit;
  {
    // Callbacks are copied under the mutex but invoked without it so test callbacks can
    // reenter RosServiceCaller, including calling shutdown() from the waitable callback.
    std::lock_guard<std::mutex> lock(waitable_callback_mutex);
    on_enter = this->on_waitable_enter;
    on_exit = this->on_waitable_exit;
  }
  ScopeExit finish_waitable_callback([this, on_exit = std::move(on_exit)]() mutable {
    if (on_exit) {
      on_exit();
    }
    finishWaitableCallback();
  });

  if (on_enter) {
    on_enter();
  }

  std::lock_guard<std::mutex> state_lock(state_mutex);
  if (!isWaitableOpen()) {
    return;
  }

  drainResponses();
  const auto now = std::chrono::steady_clock::now();
  failMatchingCalls(
    [now](const InflightCall & call) { return now >= call.deadline; }, "Service call timed out.", "timeout", true);

  syncWaitableLocked(false);
}

bool RosServiceCaller::Impl::isWaitableOpen()
{
  std::lock_guard<std::mutex> lock(waitable_lifecycle_mutex);
  return !waitable_closed;
}

void RosServiceCaller::Impl::closeWaitable()
{
  std::lock_guard<std::mutex> lock(waitable_lifecycle_mutex);
  waitable_closed = true;
}

bool RosServiceCaller::Impl::tryBeginWaitableCallback()
{
  std::lock_guard<std::mutex> lock(waitable_lifecycle_mutex);
  if (waitable_closed || waitable_active) {
    return false;
  }

  waitable_active = true;
  waitable_owner_thread_id = std::this_thread::get_id();
  return true;
}

void RosServiceCaller::Impl::finishWaitableCallback()
{
  {
    std::lock_guard<std::mutex> lock(waitable_lifecycle_mutex);
    waitable_active = false;
    waitable_owner_thread_id = std::thread::id{};
  }

  waitable_idle.notify_all();
}

void RosServiceCaller::Impl::awaitWaitableIdle()
{
  const auto caller_thread_id = std::this_thread::get_id();
  std::unique_lock<std::mutex> lock(waitable_lifecycle_mutex);
  waitable_idle.wait(
    lock, [this, caller_thread_id]() { return !waitable_active || waitable_owner_thread_id == caller_thread_id; });
}

void RosServiceCaller::Impl::syncWaitableLocked(bool wake)
{
  std::shared_ptr<ServiceWaitable> current_waitable;
  {
    std::lock_guard<std::mutex> lock(waitable_lifecycle_mutex);
    current_waitable = waitable;
  }

  if (current_waitable == nullptr) {
    return;
  }

  std::vector<ClientPtr> clients;
  clients.reserve(cached_clients.size());
  for (const auto & [key, client] : cached_clients) {
    (void)key;
    clients.push_back(client);
  }

  current_waitable->setClients(std::move(clients));
  current_waitable->setDeadline(earliestDeadlineLocked());
  if (wake) {
    current_waitable->wake();
  }
}

void RosServiceCaller::Impl::detachWaitable()
{
  std::shared_ptr<ServiceWaitable> removed_waitable;
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables;
  rclcpp::CallbackGroup::SharedPtr group;

  {
    std::lock_guard<std::mutex> lock(waitable_lifecycle_mutex);
    removed_waitable = std::move(waitable);
    node_waitables = std::move(waitables);
    group = std::move(callback_group);
  }

  if (removed_waitable != nullptr) {
    node_waitables->remove_waitable(removed_waitable, group);
  }
}

std::optional<std::chrono::steady_clock::time_point> RosServiceCaller::Impl::earliestDeadlineLocked() const
{
  std::optional<std::chrono::steady_clock::time_point> earliest;
  for (const auto & [key, call] : inflight_calls) {
    (void)key;
    if (!earliest.has_value() || call.deadline < *earliest) {
      earliest = call.deadline;
    }
  }
  return earliest;
}

void RosServiceCaller::Impl::drainResponses()
{
  for (auto & [cache_key, client] : cached_clients) {
    (void)cache_key;
    while (true) {
      MessageStorage response(client->support->response.members, rosidl_runtime_cpp::MessageInitialization::ZERO);
      rmw_request_id_t header{};
      rcl_ret_t ret = rcl_take_response(&client->client, &header, response.data());
      if (ret != RCL_RET_OK) {
        break;
      }

      // rcl sequence numbers are scoped to a client instance, so settle by
      // both the client pointer and sequence number.
      const InflightKey key{client.get(), header.sequence_number};
      auto it = inflight_calls.find(key);
      if (it == inflight_calls.end()) {
        // The original call already timed out, was cancelled, or belonged to a
        // reset session. Never match a late response by service name alone,
        // because a newer call on the same service may now be inflight.
        if (const std::size_t count = late_response_throttle.recordAndTakePendingCount(); count > 0U) {
          LogEvent(kLogger, "service_response_dropped")
            .field("reason", "late_or_unknown_pending_call")
            .field("service", client->service_name)
            .field("interface_type", client->interface_type)
            .field("count", count)
            .warn();
        }
        continue;
      }

      settle(it, [&](InflightCall & call) {
        try {
          rclcpp::SerializedMessage serialized;
          client->support->response.serializer.serialize_message(response.data(), &serialized);
          std::vector<std::uint8_t> payload;
          const auto & raw = serialized.get_rcl_serialized_message();
          if (raw.buffer != nullptr && raw.buffer_length > 0U) {
            payload.assign(raw.buffer, raw.buffer + raw.buffer_length);
          }
          call.promise.set_value(Response{call.service, call.interface_type, std::move(payload)});
        } catch (const std::exception & exc) {
          call.promise.set_exception(
            std::make_exception_ptr(
              std::runtime_error(std::string("Failed to convert service response: ") + exc.what())));
        }
      });
    }
  }
}

void RosServiceCaller::Impl::clearCacheLocked()
{
  cached_clients.clear();
  type_supports.clear();
  syncWaitableLocked();
}

}  // namespace livekit_ros2_bridge
