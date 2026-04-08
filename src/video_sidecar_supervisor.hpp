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

#pragma once

#include <sys/types.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <functional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "video_config.hpp"

namespace livekit_ros2_bridge
{

// Builds the bridge-owned sidecar argv using gstreamer-publisher and a resolved launch spec.
std::vector<std::string> buildGstreamerSidecarCommand(
  const SidecarLaunchSpec & spec, const std::string & livekit_url, const std::string & livekit_token);

using SidecarCommandBuilder = std::function<std::vector<std::string>(
  const SidecarLaunchSpec & spec, const std::string & livekit_url, const std::string & livekit_token)>;
using PublisherHealthCheck = std::function<bool(const std::string & publisher_identity)>;

class VideoSidecarSupervisor final
{
public:
  struct Config
  {
    std::string livekit_url;
    std::string livekit_room;
    std::string api_key;
    std::string api_secret;
    std::chrono::seconds token_ttl{3600};
    std::chrono::seconds token_refresh_margin{300};
    std::chrono::milliseconds health_check_startup_grace{std::chrono::seconds(5)};
    std::size_t unhealthy_restart_threshold{5};
    std::string bridge_identity;
  };

  VideoSidecarSupervisor(
    Config config, SidecarCommandBuilder build_sidecar_command, PublisherHealthCheck is_publisher_healthy = {});
  ~VideoSidecarSupervisor();

  // Returns the publisher_identity for the launch spec. Spawns a new sidecar if
  // none exists, or respawns if the previous one crashed.
  std::string ensureSidecar(const SidecarLaunchSpec & spec);

  void stopSidecar(const std::string & sidecar_key);
  bool isSidecarRunning(const std::string & sidecar_key) const;
  void reapExitedSidecars();
  void restartUnhealthy();
  void restartExpiring();
  void shutdown();

private:
  using SteadyClock = std::chrono::steady_clock;

  struct SidecarRecord
  {
    SidecarLaunchSpec spec;
    std::string publisher_identity;
    pid_t pid = -1;
    std::chrono::system_clock::time_point token_expires_at;
    SteadyClock::time_point spawned_at;
    std::size_t consecutive_unhealthy_checks = 0;
  };

  using SidecarMap = std::unordered_map<std::string, SidecarRecord>;

  struct PreparedSidecarLaunch
  {
    std::string publisher_identity;
    std::vector<std::string> argv;
    std::chrono::system_clock::time_point token_expires_at;
  };

  static std::string derivePublisherIdentity(const std::string & bridge_identity, const SidecarLaunchSpec & spec);
  static std::string keyToSlug(const std::string & key);

  std::pair<SidecarMap::iterator, bool> ensureSidecarRecord(const SidecarLaunchSpec & spec);
  void restartSidecar(const std::string & sidecar_key, SidecarRecord & sidecar);
  void tryRestartSidecar(const std::string & sidecar_key, SidecarRecord & sidecar, const char * reason);
  PreparedSidecarLaunch prepareSidecarLaunch(const SidecarRecord & sidecar) const;
  void spawnPreparedSidecar(const std::string & sidecar_key, SidecarRecord & sidecar, PreparedSidecarLaunch launch);
  void killSidecar(const std::string & sidecar_key, SidecarRecord & sidecar);

  Config config_;
  SidecarCommandBuilder build_sidecar_command_;
  PublisherHealthCheck is_publisher_healthy_;
  std::atomic<bool> is_shutdown_{false};
  SidecarMap sidecars_;
};

}  // namespace livekit_ros2_bridge
