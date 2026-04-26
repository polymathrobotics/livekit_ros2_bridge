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

set(LIVEKIT_SDK_VERSION "0.3.4")
set(
  LIVEKIT_SDK_JAMMY_BASE_URL
  "https://github.com/jon-mcmillan/livekit-client-sdk-cpp/releases/download/v${LIVEKIT_SDK_VERSION}"
)
set(
  LIVEKIT_SDK_NOBLE_BASE_URL
  "https://github.com/livekit/client-sdk-cpp/releases/download/v${LIVEKIT_SDK_VERSION}"
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
  LIVEKIT_SDK_ARCH
  ""
  CACHE STRING
  "Artifact architecture to fetch for the LiveKit C++ SDK. Empty selects from CMAKE_SYSTEM_PROCESSOR."
)
set(
  LIVEKIT_SDK_SHA256_JAMMY_X64
  "00c9f1b0290b56f43ca691a877ee2c882e2a55988eae1c62475e7347f56707b9"
)
set(
  LIVEKIT_SDK_SHA256_JAMMY_ARM64
  "72d995bae25770f5994db17d974c95502b51ea1d839b0400fc347e636b0148b7"
)
set(
  LIVEKIT_SDK_SHA256_NOBLE_X64
  "9eae2d490092059ef96f7e5b6909808be8538897eb6779d4a7fa91e8bdf2ef03"
)
set(
  LIVEKIT_SDK_SHA256_NOBLE_ARM64
  "fc054e567ae04facd749b430fd6d1126b2ebe0bd3eb5042df01fe8317d275a5b"
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

  if(LIVEKIT_SDK_ARCH)
    set(_sdk_arch "${LIVEKIT_SDK_ARCH}")
  else()
    string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" _sdk_processor)
    if(_sdk_processor STREQUAL "x86_64" OR _sdk_processor STREQUAL "amd64")
      set(_sdk_arch "x64")
    elseif(_sdk_processor STREQUAL "aarch64" OR _sdk_processor STREQUAL "arm64")
      set(_sdk_arch "arm64")
    else()
      message(FATAL_ERROR "Unsupported LiveKit SDK architecture '${CMAKE_SYSTEM_PROCESSOR}'. Set LIVEKIT_SDK_ARCH.")
    endif()
  endif()

  if(NOT _sdk_arch STREQUAL "x64" AND NOT _sdk_arch STREQUAL "arm64")
    message(FATAL_ERROR "LIVEKIT_SDK_ARCH must be 'x64' or 'arm64', got '${_sdk_arch}'.")
  endif()

  if(_sdk_distro STREQUAL "jammy" AND _sdk_arch STREQUAL "x64")
    set(_sdk_sha256 "${LIVEKIT_SDK_SHA256_JAMMY_X64}")
  elseif(_sdk_distro STREQUAL "jammy" AND _sdk_arch STREQUAL "arm64")
    set(_sdk_sha256 "${LIVEKIT_SDK_SHA256_JAMMY_ARM64}")
  elseif(_sdk_distro STREQUAL "noble" AND _sdk_arch STREQUAL "x64")
    set(_sdk_sha256 "${LIVEKIT_SDK_SHA256_NOBLE_X64}")
  else()
    set(_sdk_sha256 "${LIVEKIT_SDK_SHA256_NOBLE_ARM64}")
  endif()

  # Jammy stays on the fork because we need Jammy-specific artifacts there.
  # Noble uses the official upstream tarballs by default. Both can still be
  # overridden.
  if(LIVEKIT_SDK_URL_OVERRIDE)
    set(_sdk_url "${LIVEKIT_SDK_URL_OVERRIDE}")
  elseif(_sdk_distro STREQUAL "jammy")
    set(_sdk_url "${LIVEKIT_SDK_JAMMY_BASE_URL}/livekit-sdk-linux-${_sdk_arch}-${_sdk_distro}-${LIVEKIT_SDK_VERSION}.tar.gz")
  else()
    set(_sdk_url "${LIVEKIT_SDK_NOBLE_BASE_URL}/livekit-sdk-linux-${_sdk_arch}-${LIVEKIT_SDK_VERSION}.tar.gz")
  endif()

  if(LIVEKIT_SDK_SHA256_OVERRIDE)
    set(_sdk_sha256 "${LIVEKIT_SDK_SHA256_OVERRIDE}")
  endif()

  message(STATUS "Using LiveKit C++ SDK ${LIVEKIT_SDK_VERSION} for ${_sdk_distro}/${_sdk_arch}: ${_sdk_url}")

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
