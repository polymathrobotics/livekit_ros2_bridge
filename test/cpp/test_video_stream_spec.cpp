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

RosVideoTopicRule makeRosVideoTopicRule(const char * rule_id, const char * pattern, const char * transform_fragment)
{
  RosVideoTopicRule rule;
  rule.rule_id = rule_id;
  rule.pattern = pattern;
  rule.transform_fragment = transform_fragment;
  return rule;
}

TEST(VideoStreamSpecTest, DefaultConfigResolvesBuiltInRawImageRule)
{
  const auto stream_config = makeDefaultVideoStreamConfig();

  ASSERT_EQ(stream_config.ros_topic_rules.size(), 1U);
  const auto & rule = stream_config.ros_topic_rules.front();
  EXPECT_EQ(rule.pattern, "/*");
  EXPECT_EQ(rule.rule_id, "default_ros");
  EXPECT_EQ(rule.transform_fragment, "");

  const auto spec = resolveRosVideoStreamSpec(stream_config, "/camera/front/image", kImageInterfaceType);

  EXPECT_EQ(spec.stream_key, "topic:/camera/front/image");
  EXPECT_EQ(spec.track_name, "ros.video.camera.front.image");
  EXPECT_EQ(spec.ros_topic, "/camera/front/image");
  EXPECT_EQ(spec.interface_type, kImageInterfaceType);
  EXPECT_EQ(spec.input_kind, VideoInputKind::RosTopic);
  EXPECT_EQ(spec.ingest_mode, kRawImageIngestMode);
  EXPECT_EQ(spec.selected_config_id, "default_ros");
  EXPECT_EQ(spec.transform_fragment, "");
  expectPublishConfigEq(spec.publish_config, stream_config.default_publish_config);
}

TEST(VideoStreamSpecTest, DefaultConfigResolvesBuiltInCompressedImageRule)
{
  const auto stream_config = makeDefaultVideoStreamConfig();

  const auto spec =
    resolveRosVideoStreamSpec(stream_config, "/camera/front/image/compressed", kCompressedImageInterfaceType);

  EXPECT_EQ(spec.stream_key, "topic:/camera/front/image/compressed");
  EXPECT_EQ(spec.track_name, "ros.video.camera.front.image.compressed");
  EXPECT_EQ(spec.ros_topic, "/camera/front/image/compressed");
  EXPECT_EQ(spec.interface_type, kCompressedImageInterfaceType);
  EXPECT_EQ(spec.input_kind, VideoInputKind::RosTopic);
  EXPECT_EQ(spec.ingest_mode, kCompressedImageIngestMode);
  EXPECT_EQ(spec.selected_config_id, "default_ros");
  EXPECT_EQ(spec.transform_fragment, "");
  expectPublishConfigEq(spec.publish_config, stream_config.default_publish_config);
}

TEST(VideoStreamSpecTest, ResolveRosVideoStreamSpecUsesLongestMatch)
{
  VideoStreamConfig stream_config = makeDefaultVideoStreamConfig();

  RosVideoTopicRule broad_rule = makeRosVideoTopicRule("broad", "/camera/*", "videoconvert ! broad-filter");
  broad_rule.publish_config = makePublishConfig(VideoPublishCodec::Vp8, 500000, 30.0, VideoPublishSimulcast::Disabled);
  RosVideoTopicRule specific_rule =
    makeRosVideoTopicRule("specific", "/camera/front/*", "videoconvert ! specific-filter");
  specific_rule.publish_config =
    makePublishConfig(VideoPublishCodec::H264, 800000, 15.0, VideoPublishSimulcast::Enabled);

  stream_config.ros_topic_rules.insert(stream_config.ros_topic_rules.begin(), broad_rule);
  stream_config.ros_topic_rules.insert(stream_config.ros_topic_rules.begin(), specific_rule);

  const auto spec = resolveRosVideoStreamSpec(stream_config, "/camera/front/image", kImageInterfaceType);

  EXPECT_EQ(spec.selected_config_id, "specific");
  EXPECT_EQ(spec.ingest_mode, kRawImageIngestMode);
  EXPECT_EQ(spec.transform_fragment, "videoconvert ! specific-filter");
  expectPublishConfigEq(spec.publish_config, specific_rule.publish_config);
}

TEST(VideoStreamSpecTest, ResolveRosVideoStreamSpecSameLengthUsesFirstDeclared)
{
  VideoStreamConfig stream_config = makeDefaultVideoStreamConfig();

  const RosVideoTopicRule first_rule = makeRosVideoTopicRule("first", "/camera/front/*", "videoconvert ! first-filter");
  const RosVideoTopicRule second_rule =
    makeRosVideoTopicRule("second", "/camera/front/*", "videoconvert ! second-filter");

  stream_config.ros_topic_rules.insert(stream_config.ros_topic_rules.begin(), first_rule);
  stream_config.ros_topic_rules.insert(stream_config.ros_topic_rules.end() - 1, second_rule);

  const auto spec = resolveRosVideoStreamSpec(stream_config, "/camera/front/image", kImageInterfaceType);

  EXPECT_EQ(spec.selected_config_id, "first");
  EXPECT_EQ(spec.transform_fragment, "videoconvert ! first-filter");
  expectPublishConfigEq(spec.publish_config, first_rule.publish_config);
}

TEST(VideoStreamSpecTest, UserCatchAllOverridesBuiltInDefault)
{
  VideoStreamConfig stream_config = makeDefaultVideoStreamConfig();

  const RosVideoTopicRule user_rule = makeRosVideoTopicRule("user_default", "/*", "videoconvert ! user-filter");
  stream_config.ros_topic_rules.insert(stream_config.ros_topic_rules.begin(), user_rule);

  const auto spec = resolveRosVideoStreamSpec(stream_config, "/camera/front/image", kImageInterfaceType);

  EXPECT_EQ(spec.selected_config_id, "user_default");
  EXPECT_EQ(spec.transform_fragment, "videoconvert ! user-filter");
  expectPublishConfigEq(spec.publish_config, user_rule.publish_config);
}

TEST(VideoStreamSpecTest, ResolveRosVideoStreamSpecDoesNotInterpolateTopicPlaceholders)
{
  VideoStreamConfig stream_config = makeDefaultVideoStreamConfig();
  stream_config.ros_topic_rules.insert(
    stream_config.ros_topic_rules.begin(), makeRosVideoTopicRule("front", "/camera/front/*", "{topic}"));

  const auto spec = resolveRosVideoStreamSpec(stream_config, "/camera/front/image", kImageInterfaceType);

  EXPECT_EQ(spec.transform_fragment, "{topic}");
  expectPublishConfigEq(spec.publish_config, stream_config.ros_topic_rules.front().publish_config);
}

TEST(VideoStreamSpecTest, TrimConfiguredSourceNameOnlyRemovesSurroundingWhitespace)
{
  EXPECT_EQ(trimConfiguredSourceName("  front_camera  "), "front_camera");
  EXPECT_EQ(trimConfiguredSourceName("  /sources/front/  "), "/sources/front/");
  EXPECT_EQ(trimConfiguredSourceName(" \t\n "), "");
}

TEST(VideoStreamSpecTest, ResolveConfiguredSourceVideoStreamSpecTrimsConfiguredSourceName)
{
  VideoStreamConfig stream_config = makeDefaultVideoStreamConfig();

  ConfiguredVideoStreamSource configured_source;
  configured_source.ingress_fragment = "videotestsrc is-live=true pattern=black";
  configured_source.transform_fragment = "videobalance saturation=0.0";
  configured_source.publish_config =
    makePublishConfig(VideoPublishCodec::H265, 1200000, 10.0, VideoPublishSimulcast::Disabled);
  stream_config.configured_sources.emplace("front_camera", std::move(configured_source));

  const auto spec = resolveConfiguredSourceVideoStreamSpec(stream_config, "  front_camera  ");

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
  VideoStreamConfig stream_config = makeDefaultVideoStreamConfig();

  ConfiguredVideoStreamSource configured_source;
  configured_source.ingress_fragment = "videotestsrc is-live=true pattern=black";
  stream_config.configured_sources.emplace("/sources/front:rgb%", std::move(configured_source));

  const auto spec = resolveConfiguredSourceVideoStreamSpec(stream_config, "/sources/front:rgb%");

  EXPECT_EQ(spec.track_name, "ros.video.configured_source.%2Fsources%2Ffront%3Argb%25");
}

TEST(VideoStreamSpecTest, ResolveConfiguredSourceVideoStreamSpecRejectsUnknownConfiguredSourceName)
{
  const VideoStreamConfig stream_config = makeDefaultVideoStreamConfig();

  try {
    (void)resolveConfiguredSourceVideoStreamSpec(stream_config, "sources/missing");
    FAIL() << "Expected invalid_argument";
  } catch (const std::invalid_argument & exc) {
    EXPECT_STREQ(exc.what(), "Unknown configured video source 'sources/missing'.");
  }
}

}  // namespace
}  // namespace livekit_ros2_bridge
