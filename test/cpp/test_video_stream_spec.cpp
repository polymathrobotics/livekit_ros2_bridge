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

#include <stdexcept>
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

RosVideoTopicRule makeRule(const char * rule_id, const char * pattern, const char * transform_fragment)
{
  RosVideoTopicRule rule;
  rule.rule_id = rule_id;
  rule.pattern = pattern;
  rule.transform_fragment = transform_fragment;
  return rule;
}

template <typename Exception, typename Callable>
void expectThrowsWithMessage(Callable && callable, const char * expected_message)
{
  try {
    std::forward<Callable>(callable)();
    FAIL() << "Expected exception";
  } catch (const Exception & exc) {
    EXPECT_STREQ(exc.what(), expected_message);
  }
}

TEST(VideoStreamSpecTest, ClassifyRosVideoIngestModeOnlyAcceptsSupportedExactStrings)
{
  EXPECT_EQ(classifyRosVideoIngestMode(kImageInterfaceType), kRawImageIngestMode);

  EXPECT_EQ(classifyRosVideoIngestMode(kCompressedImageInterfaceType), kCompressedImageIngestMode);
  EXPECT_FALSE(classifyRosVideoIngestMode(" sensor_msgs/msg/Image").has_value());
}

TEST(VideoStreamSpecTest, ResolveRosVideoTopicSpecUsesBuiltInDefaultSelectionForSupportedTypes)
{
  const auto config = makeDefaultVideoStreamConfig();

  const auto raw_spec = resolveRosVideoTopicSpec(config, "/camera/front/image", kImageInterfaceType);
  EXPECT_EQ(raw_spec.stream_key, "topic:/camera/front/image");
  EXPECT_EQ(raw_spec.track_name, "ros.video.camera.front.image");
  EXPECT_EQ(raw_spec.ros_topic, "/camera/front/image");
  EXPECT_EQ(raw_spec.interface_type, kImageInterfaceType);
  EXPECT_EQ(raw_spec.input_kind, VideoInputKind::RosTopic);
  EXPECT_EQ(raw_spec.config_id, "default_ros");
  EXPECT_EQ(raw_spec.ingest_mode, kRawImageIngestMode);
  expectPublishConfigEq(raw_spec.publish_config, config.default_publish_config);

  const auto compressed_spec =
    resolveRosVideoTopicSpec(config, "/camera/front/image/compressed", kCompressedImageInterfaceType);
  EXPECT_EQ(compressed_spec.stream_key, "topic:/camera/front/image/compressed");
  EXPECT_EQ(compressed_spec.track_name, "ros.video.camera.front.image.compressed");
  EXPECT_EQ(compressed_spec.config_id, "default_ros");
  EXPECT_EQ(compressed_spec.ingest_mode, kCompressedImageIngestMode);
  expectPublishConfigEq(compressed_spec.publish_config, config.default_publish_config);
}

TEST(VideoStreamSpecTest, ResolveRosVideoTopicSpecNormalizesTopicForMatchingAndIdentifiers)
{
  VideoStreamConfig config = makeDefaultVideoStreamConfig();

  RosVideoTopicRule normalized_rule = makeRule("normalized", "/camera/front/*", "videoconvert ! normalized-filter");
  normalized_rule.publish_config =
    makePublishConfig(VideoPublishCodec::H264, 900000, 12.0, VideoPublishSimulcast::Disabled);
  config.ros_topic_rules.insert(config.ros_topic_rules.begin(), normalized_rule);

  const auto spec = resolveRosVideoTopicSpec(config, "  camera//front/image/  ", kImageInterfaceType);

  EXPECT_EQ(spec.stream_key, "topic:/camera/front/image");
  EXPECT_EQ(spec.track_name, "ros.video.camera.front.image");
  EXPECT_EQ(spec.ros_topic, "/camera/front/image");
  EXPECT_EQ(spec.config_id, "normalized");
  EXPECT_EQ(spec.transform_fragment, "videoconvert ! normalized-filter");
  expectPublishConfigEq(spec.publish_config, normalized_rule.publish_config);
}

TEST(VideoStreamSpecTest, ResolveRosVideoTopicSpecUsesUnnamedTrackNameForRootTopic)
{
  const auto config = makeDefaultVideoStreamConfig();

  const auto spec = resolveRosVideoTopicSpec(config, "/", kImageInterfaceType);

  EXPECT_EQ(spec.stream_key, "topic:/");
  EXPECT_EQ(spec.track_name, "ros.video.unnamed");
  EXPECT_EQ(spec.ros_topic, "/");
}

TEST(VideoStreamSpecTest, ResolveRosVideoTopicSpecRejectsInvalidInput)
{
  const auto config = makeDefaultVideoStreamConfig();

  expectThrowsWithMessage<std::invalid_argument>(
    [&]() { (void)resolveRosVideoTopicSpec(config, "/camera/front/image", ""); },
    "ROS topic is not a supported video type.");
  expectThrowsWithMessage<std::invalid_argument>(
    [&]() { (void)resolveRosVideoTopicSpec(config, " \t\n ", kImageInterfaceType); }, "Invalid ROS topic.");
}

TEST(VideoStreamSpecTest, ResolveRosVideoTopicSpecRejectsWhenNoRuleMatches)
{
  const VideoStreamConfig config;

  expectThrowsWithMessage<std::runtime_error>(
    [&]() { (void)resolveRosVideoTopicSpec(config, "/camera/front/image", kImageInterfaceType); },
    "no matching video rule for topic '/camera/front/image'");
}

TEST(VideoStreamSpecTest, ResolveRosVideoTopicSpecUsesLongestMatch)
{
  VideoStreamConfig config = makeDefaultVideoStreamConfig();

  RosVideoTopicRule broad_rule = makeRule("broad", "/camera/*", "videoconvert ! broad-filter");
  broad_rule.publish_config = makePublishConfig(VideoPublishCodec::Vp8, 500000, 30.0, VideoPublishSimulcast::Disabled);
  RosVideoTopicRule specific_rule = makeRule("specific", "/camera/front/*", "videoconvert ! specific-filter");
  specific_rule.publish_config =
    makePublishConfig(VideoPublishCodec::H264, 800000, 15.0, VideoPublishSimulcast::Enabled);

  config.ros_topic_rules.insert(config.ros_topic_rules.begin(), broad_rule);
  config.ros_topic_rules.insert(config.ros_topic_rules.begin(), specific_rule);

  const auto spec = resolveRosVideoTopicSpec(config, "/camera/front/image", kImageInterfaceType);

  EXPECT_EQ(spec.config_id, "specific");
  expectPublishConfigEq(spec.publish_config, specific_rule.publish_config);
}

TEST(VideoStreamSpecTest, ResolveRosVideoTopicSpecSameLengthUsesFirstDeclared)
{
  VideoStreamConfig config = makeDefaultVideoStreamConfig();

  const RosVideoTopicRule first_rule = makeRule("first", "/camera/front/*", "videoconvert ! first-filter");
  const RosVideoTopicRule second_rule = makeRule("second", "/camera/front/*", "videoconvert ! second-filter");

  config.ros_topic_rules.insert(config.ros_topic_rules.begin(), first_rule);
  config.ros_topic_rules.insert(config.ros_topic_rules.end() - 1, second_rule);

  const auto spec = resolveRosVideoTopicSpec(config, "/camera/front/image", kImageInterfaceType);

  EXPECT_EQ(spec.config_id, "first");
}

TEST(VideoStreamSpecTest, ResolveRosVideoTopicSpecDoesNotInterpolateTopicPlaceholders)
{
  VideoStreamConfig config = makeDefaultVideoStreamConfig();

  config.ros_topic_rules.insert(config.ros_topic_rules.begin(), makeRule("front", "/camera/front/*", "{topic}"));

  const auto spec = resolveRosVideoTopicSpec(config, "/camera/front/image", kImageInterfaceType);

  EXPECT_EQ(spec.transform_fragment, "{topic}");
}

TEST(VideoStreamSpecTest, ResolveConfiguredVideoSourceSpecTrimsConfiguredSourceName)
{
  VideoStreamConfig config = makeDefaultVideoStreamConfig();
  const auto expected_publish_config =
    makePublishConfig(VideoPublishCodec::H265, 1200000, 10.0, VideoPublishSimulcast::Disabled);

  ConfiguredVideoStreamSource source_config;
  source_config.ingress_fragment = "videotestsrc is-live=true pattern=black";
  source_config.transform_fragment = "videobalance saturation=0.0";
  source_config.publish_config = expected_publish_config;

  config.configured_sources.emplace("front_camera", std::move(source_config));

  const auto spec = resolveConfiguredVideoSourceSpec(config, "  front_camera  ");

  EXPECT_EQ(spec.stream_key, "configured_source:front_camera");
  EXPECT_EQ(spec.track_name, "ros.video.configured_source.front_camera");
  EXPECT_EQ(spec.source_name, "front_camera");
  EXPECT_EQ(spec.input_kind, VideoInputKind::ConfiguredSource);
  EXPECT_EQ(spec.ingest_mode, kConfiguredSourceIngestMode);
  EXPECT_EQ(spec.ingress_fragment, "videotestsrc is-live=true pattern=black");
  EXPECT_EQ(spec.transform_fragment, "videobalance saturation=0.0");
  expectPublishConfigEq(spec.publish_config, expected_publish_config);
}

TEST(VideoStreamSpecTest, ResolveConfiguredVideoSourceSpecPercentEncodesTrackNameSuffix)
{
  VideoStreamConfig config = makeDefaultVideoStreamConfig();

  ConfiguredVideoStreamSource source_config;
  source_config.ingress_fragment = "videotestsrc is-live=true pattern=black";

  config.configured_sources.emplace("/sources/front:rgb%", std::move(source_config));

  const auto spec = resolveConfiguredVideoSourceSpec(config, "/sources/front:rgb%");

  EXPECT_EQ(spec.track_name, "ros.video.configured_source.%2Fsources%2Ffront%3Argb%25");
}

TEST(VideoStreamSpecTest, ResolveConfiguredVideoSourceSpecRejectsInvalidNames)
{
  const VideoStreamConfig config = makeDefaultVideoStreamConfig();

  expectThrowsWithMessage<std::invalid_argument>(
    [&]() { (void)resolveConfiguredVideoSourceSpec(config, "sources/missing"); },
    "Unknown configured video source 'sources/missing'.");
  expectThrowsWithMessage<std::invalid_argument>(
    [&]() { (void)resolveConfiguredVideoSourceSpec(config, " \t\n "); }, "Invalid configured source name.");
}

}  // namespace
}  // namespace livekit_ros2_bridge
