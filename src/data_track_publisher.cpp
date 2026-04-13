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
#include "room_connection.hpp"
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge
{

namespace
{
const auto kDataTrackPublisherLogger = rclcpp::get_logger("data_track_publisher");
constexpr auto kWriteLogThrottlePeriod = std::chrono::seconds(5);

LogEvent dataTrackLog(const char * event_name, const std::string & track_name)
{
  LogEvent event(kDataTrackPublisherLogger, event_name);
  event.field("track_name", track_name);
  return event;
}

const char * dataTrackPushFailureReason(DataTrackPushErrorCode code)
{
  switch (code) {
    case DataTrackPushErrorCode::kUnknown:
      return "unknown";
    case DataTrackPushErrorCode::kInvalidHandle:
      return "invalid_handle";
    case DataTrackPushErrorCode::kTrackUnpublished:
      return "track_unpublished";
    case DataTrackPushErrorCode::kQueueFull:
      return "queue_full";
    case DataTrackPushErrorCode::kInternal:
      return "internal";
  }
  return "unknown";
}

// Cleanup runs on explicit teardown and on rejected/failed publish paths, so it logs and
// swallows unpublish errors instead of letting recovery cascade into caller state handling.
void tryUnpublishDataTrack(
  RoomConnection & connection, const std::string & track_name, const std::shared_ptr<livekit::LocalDataTrack> & track)
{
  if (track == nullptr) {
    return;
  }
  try {
    connection.unpublishDataTrack(track);
  } catch (const std::exception & exc) {
    dataTrackLog("data_track_unpublish_failed", track_name).field("error", exc.what()).warn();
  } catch (...) {
    dataTrackLog("data_track_unpublish_failed", track_name).warn();
  }
}
}  // namespace

DataTrackPublisher::DataTrackPublisher(
  RoomConnection & connection, std::string track_name, rclcpp::Clock::SharedPtr clock)
: room_connection_(connection)
, track_name_(std::move(track_name))
, log_clock_(std::move(clock))
{}

void DataTrackPublisher::write(const std::uint8_t * cdr, std::size_t size)
{
  if (published_track_ == nullptr) {
    return;
  }

  // Copy into an owning buffer before handing the payload to LiveKit; callers usually pass ROS
  // serialization storage whose lifetime ends with the current subscription callback.
  auto result = room_connection_.tryPushDataTrack(published_track_, std::vector<std::uint8_t>(cdr, cdr + size));
  if (result) {
    return;
  }

  const auto & error = result.error();
  if (error.code == DataTrackPushErrorCode::kQueueFull) {
    // Forwarding ROS CDR payloads is intentionally best-effort. Dropping here keeps the ROS
    // subscription callback non-blocking even when the participant is not draining the LiveKit
    // queue.
    dataTrackLog("data_track_delivery_dropped", track_name_)
      .field("reason", "queue_full")
      .warnThrottle(*log_clock_, kWriteLogThrottlePeriod);
    return;
  }

  dataTrackLog("data_track_push_failed", track_name_)
    .field("reason", dataTrackPushFailureReason(error.code))
    .fieldOr("error", error.message)
    .warnThrottle(*log_clock_, kWriteLogThrottlePeriod);
}

void DataTrackPublisher::publish(std::size_t generation, const AcceptHandler & on_accept, const FailHandler & on_fail)
{
  const char * failure_stage = "room_publish";
  try {
    auto track = room_connection_.publishDataTrack(track_name_);
    failure_stage = "registry_accept";

    // Publish completion races with lease expiry, reset, and same-topic resubscribe. The registry
    // accepts only the current generation for this track name, so stale completions are reclaimed
    // immediately instead of leaving an orphaned LiveKit track behind.
    try {
      if (on_accept(generation)) {
        published_track_ = std::move(track);
        return;
      }
    } catch (...) {
      tryUnpublishDataTrack(room_connection_, track_name_, track);
      throw;
    }

    dataTrackLog("data_track_publish_reclaimed", track_name_)
      .field("generation", generation)
      .field("reason", "stale_registry_state")
      .info();
    tryUnpublishDataTrack(room_connection_, track_name_, track);
  } catch (const std::exception & exc) {
    on_fail();
    dataTrackLog("data_track_publish_error", track_name_)
      .field("stage", failure_stage)
      .field("error", exc.what())
      .warn();
  } catch (...) {
    on_fail();
    dataTrackLog("data_track_publish_error", track_name_).field("stage", failure_stage).warn();
  }
}

void DataTrackPublisher::unpublish()
{
  if (published_track_ == nullptr) {
    return;
  }

  // Clear the local handle before calling into LiveKit so repeated unpublish() calls stay
  // idempotent and later writes cannot target a track we are trying to tear down.
  auto track = std::move(published_track_);
  tryUnpublishDataTrack(room_connection_, track_name_, track);
}

}  // namespace livekit_ros2_bridge
