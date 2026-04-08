# Copyright (c) 2025-present Polymath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

include(FetchContent)

if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()

set(LIVEKIT_SDK_VERSION "0.3.2" CACHE STRING "Pinned LiveKit C++ SDK version.")
set(
  LIVEKIT_SDK_JAMMY_BASE_URL
  "https://github.com/jon-mcmillan/livekit-client-sdk-cpp/releases/download/v${LIVEKIT_SDK_VERSION}"
  CACHE STRING
  "Base URL for the pinned Jammy LiveKit C++ SDK artifact source. The fork publishes the jammy-specific Linux tarball."
)
set(
  LIVEKIT_SDK_NOBLE_URL
  "https://github.com/livekit/client-sdk-cpp/releases/download/v${LIVEKIT_SDK_VERSION}/livekit-sdk-linux-x64-${LIVEKIT_SDK_VERSION}.tar.gz"
  CACHE STRING
  "Default Noble LiveKit C++ SDK artifact URL. Upstream publishes a generic linux-x64 tarball that works for Noble builds."
)
set(
  LIVEKIT_SDK_URL_OVERRIDE
  ""
  CACHE STRING
  "Optional full URL override for the LiveKit C++ SDK artifact."
)
set(
  LIVEKIT_SDK_SHA256_OVERRIDE
  ""
  CACHE STRING
  "Optional SHA256 override for a custom LiveKit C++ SDK artifact URL."
)
set(
  LIVEKIT_SDK_DISTRO
  ""
  CACHE STRING
  "Artifact distro to fetch for the LiveKit C++ SDK. Empty selects jammy for humble and noble otherwise."
)
set(
  LIVEKIT_SDK_SHA256_JAMMY
  "ecf05fbd1d828ed5964139fa26731cb50de4fd78208525d491dc54227395a046"
  CACHE STRING
  "SHA256 for the jammy LiveKit C++ SDK artifact."
)
set(
  LIVEKIT_SDK_SHA256_NOBLE
  "3849bd875266e97c1244d751c44c111eea19eea38291f750dbc1773b8cb39df0"
  CACHE STRING
  "SHA256 for the default Noble LiveKit C++ SDK artifact."
)

macro(livekit_ros2_bridge_configure_livekit_sdk)
  if(LIVEKIT_SDK_DISTRO)
    set(_sdk_distro "${LIVEKIT_SDK_DISTRO}")
  elseif("$ENV{ROS_DISTRO}" STREQUAL "humble")
    set(_sdk_distro "jammy")
  else()
    set(_sdk_distro "noble")
  endif()

  if(NOT _sdk_distro STREQUAL "jammy" AND NOT _sdk_distro STREQUAL "noble")
    message(FATAL_ERROR "LIVEKIT_SDK_DISTRO must be 'jammy' or 'noble', got '${_sdk_distro}'.")
  endif()

  if(_sdk_distro STREQUAL "jammy")
    set(_sdk_sha256 "${LIVEKIT_SDK_SHA256_JAMMY}")
  else()
    set(_sdk_sha256 "${LIVEKIT_SDK_SHA256_NOBLE}")
  endif()

  # Upstream livekit/client-sdk-cpp publishes a generic linux-x64 tarball. Jammy
  # stays on the fork because we need a jammy-specific artifact there. Noble uses
  # the official upstream tarball by default. Both can still be overridden.
  if(LIVEKIT_SDK_URL_OVERRIDE)
    set(_sdk_url "${LIVEKIT_SDK_URL_OVERRIDE}")
  elseif(_sdk_distro STREQUAL "jammy")
    set(_sdk_url "${LIVEKIT_SDK_JAMMY_BASE_URL}/livekit-sdk-linux-x64-${_sdk_distro}-${LIVEKIT_SDK_VERSION}.tar.gz")
  else()
    set(_sdk_url "${LIVEKIT_SDK_NOBLE_URL}")
  endif()

  if(LIVEKIT_SDK_SHA256_OVERRIDE)
    set(_sdk_sha256 "${LIVEKIT_SDK_SHA256_OVERRIDE}")
  endif()

  fetchcontent_declare(livekit_sdk
    URL "${_sdk_url}"
    URL_HASH SHA256=${_sdk_sha256}
  )
  fetchcontent_populate(livekit_sdk)
  list(APPEND CMAKE_PREFIX_PATH "${livekit_sdk_SOURCE_DIR}")
  find_package(LiveKit REQUIRED)

  file(GLOB _sdk_libs "${livekit_sdk_SOURCE_DIR}/lib/*.so*")
  install(FILES ${_sdk_libs} DESTINATION lib)
endmacro()
