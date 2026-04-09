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

#include "cdr_track_publisher.hpp"

#include <exception>
#include <utility>
#include <vector>

#include "livekit/data_track_error.h"
#include "livekit/local_data_track.h"
#include "rclcpp/logging.hpp"
#include "room_session.hpp"
#include "subscription_registry.hpp"

namespace livekit_ros2_bridge
{

namespace
{
const auto kCdrTrackPublisherLogger = rclcpp::get_logger("cdr_track_publisher");

void unpublishTrackSafely(
  RoomSession & session, const std::string & track_name, const std::shared_ptr<livekit::LocalDataTrack> & track)
{
  try {
    session.unpublishCdrTrack(track);
    RCLCPP_INFO(kCdrTrackPublisherLogger, "event=cdr_track_unpublished track_name=%s", track_name.c_str());
  } catch (const std::exception & exc) {
    RCLCPP_WARN(
      kCdrTrackPublisherLogger,
      "event=cdr_track_unpublish_failed track_name=%s error=%s",
      track_name.c_str(),
      exc.what());
  } catch (...) {
    RCLCPP_WARN(kCdrTrackPublisherLogger, "event=cdr_track_unpublish_failed track_name=%s", track_name.c_str());
  }
}
}  // namespace

CdrTrackPublisher::CdrTrackPublisher(RoomSession & session, rclcpp::Clock::SharedPtr clock)
: session_(session)
, clock_(std::move(clock))
{}

void CdrTrackPublisher::pushMessage(const std::string & track_name, const std::uint8_t * data, std::size_t size)
{
  auto it = published_tracks_.find(track_name);
  if (it == published_tracks_.end() || it->second == nullptr) {
    return;
  }
  auto result = it->second->tryPush(std::vector<std::uint8_t>(data, data + size));
  if (!result) {
    if (result.error().code == livekit::LocalDataTrackTryPushErrorCode::QUEUE_FULL) {
      // CDR forwarding is intentionally best-effort. Dropping here keeps the ROS subscription
      // callback non-blocking even when the participant is not draining the LiveKit queue.
      RCLCPP_WARN_THROTTLE(
        kCdrTrackPublisherLogger,
        *clock_,
        5000,
        "event=cdr_track_delivery_dropped track_name=%s reason=queue_full",
        track_name.c_str());
      return;
    }
    RCLCPP_WARN_THROTTLE(
      kCdrTrackPublisherLogger,
      *clock_,
      5000,
      "event=cdr_track_push_failed track_name=%s error=%s",
      track_name.c_str(),
      result.error().message.c_str());
  }
}

void CdrTrackPublisher::publishTrack(
  const std::string & track_name, std::size_t generation, SubscriptionRegistry & subscription_registry)
{
  try {
    auto track = session_.publishCdrTrack(track_name);
    // Publish completion races with lease expiry, reset, and same-topic resubscribe. The registry
    // accepts only the current generation for this track name, so stale completions are reclaimed
    // immediately instead of leaving an orphaned LiveKit track behind.
    if (!subscription_registry.onCdrTrackPublished(track_name, generation)) {
      RCLCPP_INFO(
        kCdrTrackPublisherLogger,
        "event=cdr_track_publish_reclaimed track_name=%s generation=%zu reason=stale_registry_state",
        track_name.c_str(),
        generation);
      unpublishTrackSafely(session_, track_name, track);
      return;
    }
    published_tracks_[track_name] = std::move(track);
    RCLCPP_INFO(
      kCdrTrackPublisherLogger,
      "event=cdr_track_publish_completed track_name=%s generation=%zu",
      track_name.c_str(),
      generation);
  } catch (const std::exception & exc) {
    subscription_registry.onCdrTrackFailed(track_name);
    RCLCPP_WARN(
      kCdrTrackPublisherLogger,
      "event=cdr_track_publish_error track_name=%s generation=%zu error=%s",
      track_name.c_str(),
      generation,
      exc.what());
  } catch (...) {
    subscription_registry.onCdrTrackFailed(track_name);
    RCLCPP_WARN(
      kCdrTrackPublisherLogger,
      "event=cdr_track_publish_error track_name=%s generation=%zu",
      track_name.c_str(),
      generation);
  }
}

void CdrTrackPublisher::unpublishTrack(const std::string & track_name)
{
  auto it = published_tracks_.find(track_name);
  if (it != published_tracks_.end()) {
    unpublishTrackSafely(session_, track_name, it->second);
    published_tracks_.erase(it);
  }
}

void CdrTrackPublisher::unpublishAll()
{
  auto tracks = std::move(published_tracks_);
  published_tracks_.clear();

  for (const auto & entry : tracks) {
    unpublishTrackSafely(session_, entry.first, entry.second);
  }
}

}  // namespace livekit_ros2_bridge
