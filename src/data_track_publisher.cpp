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

#include "data_track_publisher.hpp"

#include <exception>
#include <utility>
#include <vector>

#include "livekit/data_track_error.h"
#include "livekit/local_data_track.h"
#include "rclcpp/logging.hpp"
#include "room_session.hpp"
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge
{

namespace
{
const auto kDataTrackPublisherLogger = rclcpp::get_logger("data_track_publisher");
constexpr auto kPushFailureLogThrottlePeriod = std::chrono::seconds(5);

void unpublishTrackNoThrow(
  RoomSession & session, const std::string & track_name, const std::shared_ptr<livekit::LocalDataTrack> & track)
{
  if (track == nullptr) {
    return;
  }
  try {
    session.unpublishDataTrack(track);
    LogEvent(kDataTrackPublisherLogger, "data_track_unpublished").field("track_name", track_name).info();
  } catch (const std::exception & exc) {
    LogEvent(kDataTrackPublisherLogger, "data_track_unpublish_failed")
      .field("track_name", track_name)
      .field("error", exc.what())
      .warn();
  } catch (...) {
    LogEvent(kDataTrackPublisherLogger, "data_track_unpublish_failed").field("track_name", track_name).warn();
  }
}
}  // namespace

DataTrackPublisher::DataTrackPublisher(RoomSession & session, std::string track_name, rclcpp::Clock::SharedPtr clock)
: session_(session)
, track_name_(std::move(track_name))
, clock_(std::move(clock))
{}

void DataTrackPublisher::tryPush(const std::uint8_t * data, std::size_t size)
{
  if (published_track_ == nullptr) {
    return;
  }
  auto result = session_.tryPushDataTrack(published_track_, std::vector<std::uint8_t>(data, data + size));
  if (!result) {
    if (result.error().code == DataTrackPushErrorCode::kQueueFull) {
      // Forwarding ROS CDR payloads is intentionally best-effort. Dropping here keeps the ROS
      // subscription callback non-blocking even when the participant is not draining the LiveKit
      // queue.
      LogEvent(kDataTrackPublisherLogger, "data_track_delivery_dropped")
        .field("track_name", track_name_)
        .field("reason", "queue_full")
        .warnThrottle(*clock_, kPushFailureLogThrottlePeriod);
      return;
    }
    LogEvent(kDataTrackPublisherLogger, "data_track_push_failed")
      .field("track_name", track_name_)
      .field("error", result.error().message)
      .warnThrottle(*clock_, kPushFailureLogThrottlePeriod);
  }
}

void DataTrackPublisher::publish(
  std::size_t generation, const PublishAcceptedFn & publish_accepted_fn, const PublishFailedFn & publish_failed_fn)
{
  try {
    auto track = session_.publishDataTrack(track_name_);
    // Publish completion races with lease expiry, reset, and same-topic resubscribe. The registry
    // accepts only the current generation for this track name, so stale completions are reclaimed
    // immediately instead of leaving an orphaned LiveKit track behind.
    const bool publication_accepted = publish_accepted_fn(generation);
    if (!publication_accepted) {
      LogEvent(kDataTrackPublisherLogger, "data_track_publish_reclaimed")
        .field("track_name", track_name_)
        .field("generation", generation)
        .field("reason", "stale_registry_state")
        .info();
      unpublishTrackNoThrow(session_, track_name_, track);
      return;
    }
    published_track_ = std::move(track);
    LogEvent(kDataTrackPublisherLogger, "data_track_publish_completed")
      .field("track_name", track_name_)
      .field("generation", generation)
      .info();
  } catch (const std::exception & exc) {
    publish_failed_fn();
    LogEvent(kDataTrackPublisherLogger, "data_track_publish_error")
      .field("track_name", track_name_)
      .field("generation", generation)
      .field("error", exc.what())
      .warn();
  } catch (...) {
    publish_failed_fn();
    LogEvent(kDataTrackPublisherLogger, "data_track_publish_error")
      .field("track_name", track_name_)
      .field("generation", generation)
      .warn();
  }
}

void DataTrackPublisher::unpublish()
{
  if (published_track_ == nullptr) {
    return;
  }

  auto published_track = std::move(published_track_);
  published_track_.reset();
  unpublishTrackNoThrow(session_, track_name_, published_track);
}

void DataTrackPublisher::shutdown()
{
  unpublish();
}

}  // namespace livekit_ros2_bridge
