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
#include "subscription_registry.hpp"
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

DataTrackPublisher::DataTrackPublisher(RoomSession & session, rclcpp::Clock::SharedPtr clock)
: session_(session)
, clock_(std::move(clock))
{}

void DataTrackPublisher::pushMessage(const std::string & track_name, const std::uint8_t * data, std::size_t size)
{
  auto it = published_tracks_.find(track_name);
  if (it == published_tracks_.end() || it->second == nullptr) {
    return;
  }
  auto result = it->second->tryPush(std::vector<std::uint8_t>(data, data + size));
  if (!result) {
    if (result.error().code == livekit::LocalDataTrackTryPushErrorCode::QUEUE_FULL) {
      // Forwarding ROS CDR payloads is intentionally best-effort. Dropping here keeps the ROS
      // subscription callback non-blocking even when the participant is not draining the LiveKit
      // queue.
      LogEvent(kDataTrackPublisherLogger, "data_track_delivery_dropped")
        .field("track_name", track_name)
        .field("reason", "queue_full")
        .warnThrottle(*clock_, kPushFailureLogThrottlePeriod);
      return;
    }
    LogEvent(kDataTrackPublisherLogger, "data_track_push_failed")
      .field("track_name", track_name)
      .field("error", result.error().message)
      .warnThrottle(*clock_, kPushFailureLogThrottlePeriod);
  }
}

void DataTrackPublisher::publishTrack(
  const std::string & track_name, std::size_t generation, SubscriptionRegistry & subscription_registry)
{
  try {
    auto track = session_.publishDataTrack(track_name);
    // Publish completion races with lease expiry, reset, and same-topic resubscribe. The registry
    // accepts only the current generation for this track name, so stale completions are reclaimed
    // immediately instead of leaving an orphaned LiveKit track behind.
    const bool publication_accepted = subscription_registry.onDataTrackPublished(track_name, generation);
    if (!publication_accepted) {
      LogEvent(kDataTrackPublisherLogger, "data_track_publish_reclaimed")
        .field("track_name", track_name)
        .field("generation", generation)
        .field("reason", "stale_registry_state")
        .info();
      unpublishTrackNoThrow(session_, track_name, track);
      return;
    }
    published_tracks_[track_name] = std::move(track);
    LogEvent(kDataTrackPublisherLogger, "data_track_publish_completed")
      .field("track_name", track_name)
      .field("generation", generation)
      .info();
  } catch (const std::exception & exc) {
    subscription_registry.onDataTrackFailed(track_name);
    LogEvent(kDataTrackPublisherLogger, "data_track_publish_error")
      .field("track_name", track_name)
      .field("generation", generation)
      .field("error", exc.what())
      .warn();
  } catch (...) {
    subscription_registry.onDataTrackFailed(track_name);
    LogEvent(kDataTrackPublisherLogger, "data_track_publish_error")
      .field("track_name", track_name)
      .field("generation", generation)
      .warn();
  }
}

void DataTrackPublisher::unpublishTrack(const std::string & track_name)
{
  auto it = published_tracks_.find(track_name);
  if (it == published_tracks_.end()) {
    return;
  }

  unpublishTrackNoThrow(session_, track_name, it->second);
  published_tracks_.erase(it);
}

void DataTrackPublisher::unpublishAll()
{
  auto tracks = std::move(published_tracks_);
  published_tracks_.clear();

  for (const auto & entry : tracks) {
    unpublishTrackNoThrow(session_, entry.first, entry.second);
  }
}

}  // namespace livekit_ros2_bridge
