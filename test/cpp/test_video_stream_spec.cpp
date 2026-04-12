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

#include <utility>

#include "gtest/gtest.h"
#include "video_stream_spec.hpp"

namespace livekit_ros2_bridge
{
namespace
{

VideoPublishConfig makePublishConfig(
  VideoPublishCodec codec, std::uint64_t max_bitrate_bps, double max_framerate, VideoPublishSimulcast simulcast)
{
  VideoPublishConfig config;
  config.codec = codec;
  config.max_bitrate_bps = max_bitrate_bps;
  config.max_framerate = max_framerate;
  config.simulcast = simulcast;
  return config;
}

void expectPublishConfigEq(const VideoPublishConfig & actual, const VideoPublishConfig & expected)
{
  EXPECT_EQ(actual.codec, expected.codec);
  EXPECT_EQ(actual.max_bitrate_bps, expected.max_bitrate_bps);
  EXPECT_DOUBLE_EQ(actual.max_framerate, expected.max_framerate);
  EXPECT_EQ(actual.simulcast, expected.simulcast);
}

RosTopicRule makeRosRule(const char * id, const char * pattern, const char * transform)
{
  RosTopicRule rule;
  rule.id = id;
  rule.pattern = pattern;
  rule.transform = transform;
  return rule;
}

TEST(VideoStreamSpecTest, DefaultConfigResolvesBuiltInRawImageRule)
{
  const auto config = makeDefaultVideoConfig();

  ASSERT_EQ(config.ros_topic_rules.size(), 1U);
  const auto & rule = config.ros_topic_rules.front();
  EXPECT_EQ(rule.pattern, "/*");
  EXPECT_EQ(rule.id, video_defaults::kDefaultRosProfileId);
  EXPECT_EQ(rule.transform, video_defaults::kDefaultRosTransform);

  const auto spec = resolveRosVideoStreamSpec(config, "/camera/front/image", kImageInterfaceType);

  EXPECT_EQ(spec.stream_key, "topic:/camera/front/image");
  EXPECT_EQ(spec.track_name, "ros.video.camera.front.image");
  EXPECT_EQ(spec.ros_topic, "/camera/front/image");
  EXPECT_EQ(spec.interface_type, kImageInterfaceType);
  EXPECT_EQ(spec.input_kind, VideoInputKind::RosTopic);
  EXPECT_EQ(spec.ingest_mode, kRawImageIngestMode);
  EXPECT_EQ(spec.selected_config_id, video_defaults::kDefaultRosProfileId);
  EXPECT_EQ(spec.transform_fragment, video_defaults::kDefaultRosTransform);
  expectPublishConfigEq(spec.publish_config, config.publish);
}

TEST(VideoStreamSpecTest, DefaultConfigResolvesBuiltInCompressedImageRule)
{
  const auto config = makeDefaultVideoConfig();

  const auto spec = resolveRosVideoStreamSpec(config, "/camera/front/image/compressed", kCompressedImageInterfaceType);

  EXPECT_EQ(spec.stream_key, "topic:/camera/front/image/compressed");
  EXPECT_EQ(spec.track_name, "ros.video.camera.front.image.compressed");
  EXPECT_EQ(spec.ros_topic, "/camera/front/image/compressed");
  EXPECT_EQ(spec.interface_type, kCompressedImageInterfaceType);
  EXPECT_EQ(spec.input_kind, VideoInputKind::RosTopic);
  EXPECT_EQ(spec.ingest_mode, kCompressedImageIngestMode);
  EXPECT_EQ(spec.selected_config_id, video_defaults::kDefaultRosProfileId);
  EXPECT_EQ(spec.transform_fragment, video_defaults::kDefaultRosTransform);
  expectPublishConfigEq(spec.publish_config, config.publish);
}

TEST(VideoStreamSpecTest, ResolveRosVideoStreamSpecUsesLongestMatch)
{
  VideoConfig config = makeDefaultVideoConfig();

  RosTopicRule broad_rule = makeRosRule("broad", "/camera/*", "videoconvert ! broad-filter");
  broad_rule.publish = makePublishConfig(VideoPublishCodec::Vp8, 500000, 30.0, VideoPublishSimulcast::Disabled);
  RosTopicRule specific_rule = makeRosRule("specific", "/camera/front/*", "videoconvert ! specific-filter");
  specific_rule.publish = makePublishConfig(VideoPublishCodec::H264, 800000, 15.0, VideoPublishSimulcast::Enabled);

  config.ros_topic_rules.insert(config.ros_topic_rules.begin(), broad_rule);
  config.ros_topic_rules.insert(config.ros_topic_rules.begin(), specific_rule);

  const auto spec = resolveRosVideoStreamSpec(config, "/camera/front/image", kImageInterfaceType);

  EXPECT_EQ(spec.selected_config_id, "specific");
  EXPECT_EQ(spec.ingest_mode, kRawImageIngestMode);
  EXPECT_EQ(spec.transform_fragment, "videoconvert ! specific-filter");
  expectPublishConfigEq(spec.publish_config, specific_rule.publish);
}

TEST(VideoStreamSpecTest, ResolveRosVideoStreamSpecSameLengthUsesFirstDeclared)
{
  VideoConfig config = makeDefaultVideoConfig();

  const RosTopicRule first_rule = makeRosRule("first", "/camera/front/*", "videoconvert ! first-filter");
  const RosTopicRule second_rule = makeRosRule("second", "/camera/front/*", "videoconvert ! second-filter");

  config.ros_topic_rules.insert(config.ros_topic_rules.begin(), first_rule);
  config.ros_topic_rules.insert(config.ros_topic_rules.end() - 1, second_rule);

  const auto spec = resolveRosVideoStreamSpec(config, "/camera/front/image", kImageInterfaceType);

  EXPECT_EQ(spec.selected_config_id, "first");
  EXPECT_EQ(spec.transform_fragment, "videoconvert ! first-filter");
  expectPublishConfigEq(spec.publish_config, first_rule.publish);
}

TEST(VideoStreamSpecTest, UserCatchAllOverridesBuiltInDefault)
{
  VideoConfig config = makeDefaultVideoConfig();

  const RosTopicRule user_rule = makeRosRule("user_default", "/*", "videoconvert ! user-filter");
  config.ros_topic_rules.insert(config.ros_topic_rules.begin(), user_rule);

  const auto spec = resolveRosVideoStreamSpec(config, "/camera/front/image", kImageInterfaceType);

  EXPECT_EQ(spec.selected_config_id, "user_default");
  EXPECT_EQ(spec.transform_fragment, "videoconvert ! user-filter");
  expectPublishConfigEq(spec.publish_config, user_rule.publish);
}

TEST(VideoStreamSpecTest, ResolveRosVideoStreamSpecDoesNotInterpolateTopicPlaceholders)
{
  VideoConfig config = makeDefaultVideoConfig();
  config.ros_topic_rules.insert(config.ros_topic_rules.begin(), makeRosRule("front", "/camera/front/*", "{topic}"));

  const auto spec = resolveRosVideoStreamSpec(config, "/camera/front/image", kImageInterfaceType);

  EXPECT_EQ(spec.transform_fragment, "{topic}");
  expectPublishConfigEq(spec.publish_config, config.ros_topic_rules.front().publish);
}

TEST(VideoStreamSpecTest, TrimConfiguredSourceNameOnlyRemovesSurroundingWhitespace)
{
  EXPECT_EQ(trimConfiguredSourceName("  front_camera  "), "front_camera");
  EXPECT_EQ(trimConfiguredSourceName("  /sources/front/  "), "/sources/front/");
  EXPECT_EQ(trimConfiguredSourceName(" \t\n "), "");
}

TEST(VideoStreamSpecTest, ResolveConfiguredSourceVideoStreamSpecTrimsConfiguredSourceName)
{
  VideoConfig config = makeDefaultVideoConfig();

  ConfiguredVideoSourceConfig configured_source_config;
  configured_source_config.ingress_fragment = "videotestsrc is-live=true pattern=black";
  configured_source_config.transform_fragment = "videobalance saturation=0.0";
  configured_source_config.publish =
    makePublishConfig(VideoPublishCodec::H265, 1200000, 10.0, VideoPublishSimulcast::Disabled);
  config.configured_sources.emplace("front_camera", std::move(configured_source_config));

  const auto spec = resolveConfiguredSourceVideoStreamSpec(config, "  front_camera  ");

  EXPECT_EQ(spec.stream_key, "configured_source:front_camera");
  EXPECT_EQ(spec.track_name, "ros.video.configured_source.front_camera");
  EXPECT_EQ(spec.configured_source_name, "front_camera");
  EXPECT_EQ(spec.input_kind, VideoInputKind::ConfiguredSource);
  EXPECT_EQ(spec.ingest_mode, kConfiguredSourceIngestMode);
  EXPECT_EQ(spec.selected_config_id, "front_camera");
  EXPECT_EQ(spec.ingress_fragment, "videotestsrc is-live=true pattern=black");
  EXPECT_EQ(spec.transform_fragment, "videobalance saturation=0.0");
  expectPublishConfigEq(
    spec.publish_config, makePublishConfig(VideoPublishCodec::H265, 1200000, 10.0, VideoPublishSimulcast::Disabled));
}

TEST(VideoStreamSpecTest, ResolveConfiguredSourceVideoStreamSpecPercentEncodesTrackNameSuffix)
{
  VideoConfig config = makeDefaultVideoConfig();

  ConfiguredVideoSourceConfig configured_source_config;
  configured_source_config.ingress_fragment = "videotestsrc is-live=true pattern=black";
  config.configured_sources.emplace("/sources/front:rgb%", std::move(configured_source_config));

  const auto spec = resolveConfiguredSourceVideoStreamSpec(config, "/sources/front:rgb%");

  EXPECT_EQ(spec.track_name, "ros.video.configured_source.%2Fsources%2Ffront%3Argb%25");
}

TEST(VideoStreamSpecTest, ResolveConfiguredSourceVideoStreamSpecRejectsUnknownConfiguredSourceName)
{
  const VideoConfig config = makeDefaultVideoConfig();

  try {
    (void)resolveConfiguredSourceVideoStreamSpec(config, "sources/missing");
    FAIL() << "Expected invalid_argument";
  } catch (const std::invalid_argument & exc) {
    EXPECT_STREQ(exc.what(), "Unknown configured video source 'sources/missing'.");
  }
}

}  // namespace
}  // namespace livekit_ros2_bridge
