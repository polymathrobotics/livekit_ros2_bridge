# Copyright 2025 Polymath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
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

set(LIVEKIT_SDK_VERSION "1.12.2")
set(
  LIVEKIT_SDK_BASE_URL
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

macro(livekit_ros2_bridge_configure_livekit_sdk)
  if(LIVEKIT_SDK_DISTRO)
    set(_sdk_distro "${LIVEKIT_SDK_DISTRO}")
  elseif("$ENV{ROS_DISTRO}" STREQUAL "humble")
    set(_sdk_distro "jammy")
  else()
    set(_sdk_distro "noble")
  endif()

  if(_sdk_distro STREQUAL "jammy")
    set(_sdk_ubuntu_version "22.04")
    set(_sdk_x64_sha256 "a262e98006f95bd24f75c44068eeeaeb91d4830c75f6c20fee032c8fefd1b034")
    set(_sdk_arm64_sha256 "57234960500ea1013fd89e6c9cac313bd90170101f8366da973d61d46ef104e9")
  elseif(_sdk_distro STREQUAL "noble")
    set(_sdk_ubuntu_version "24.04")
    set(_sdk_x64_sha256 "a7566af830b839ec8a0682f8a2569b9f8becbba91bca85b2348a09e893d763e9")
    set(_sdk_arm64_sha256 "7db2d9d76c014bad248d83f34d5da43d66758180b0b6b8578a1f1c5fd06e29f8")
  else()
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

  if(_sdk_arch STREQUAL "x64")
    set(_sdk_sha256 "${_sdk_x64_sha256}")
  elseif(_sdk_arch STREQUAL "arm64")
    set(_sdk_sha256 "${_sdk_arm64_sha256}")
  else()
    message(FATAL_ERROR "LIVEKIT_SDK_ARCH must be 'x64' or 'arm64', got '${_sdk_arch}'.")
  endif()

  if(LIVEKIT_SDK_URL_OVERRIDE)
    set(_sdk_url "${LIVEKIT_SDK_URL_OVERRIDE}")
  else()
    set(_sdk_url "${LIVEKIT_SDK_BASE_URL}/livekit-sdk-ubuntu-${_sdk_ubuntu_version}-${_sdk_arch}-${LIVEKIT_SDK_VERSION}.tar.gz")
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
