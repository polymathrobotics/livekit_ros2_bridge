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

RosVideoTopicRule makeRosVideoTopicRule(const char * rule_id, const char * pattern, const char * transform_fragment)
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

TEST(VideoStreamSpecTest, ClassifyRosVideoInterfaceTypeOnlyAcceptsSupportedExactStrings)
{
  const auto raw_classification = classifyRosVideoInterfaceType(kImageInterfaceType);
  ASSERT_TRUE(raw_classification.has_value());
  EXPECT_EQ(raw_classification->ingest_mode, kRawImageIngestMode);

  const auto compressed_classification = classifyRosVideoInterfaceType(kCompressedImageInterfaceType);
  ASSERT_TRUE(compressed_classification.has_value());
  EXPECT_EQ(compressed_classification->ingest_mode, kCompressedImageIngestMode);
  EXPECT_FALSE(classifyRosVideoInterfaceType(" sensor_msgs/msg/Image").has_value());
}

TEST(VideoStreamSpecTest, ResolveRosVideoStreamSpecUsesBuiltInDefaultSelectionForSupportedTypes)
{
  const auto stream_config = makeDefaultVideoStreamConfig();

  const auto raw_spec = resolveRosVideoStreamSpec(stream_config, "/camera/front/image", kImageInterfaceType);
  EXPECT_EQ(raw_spec.stream_key, "topic:/camera/front/image");
  EXPECT_EQ(raw_spec.track_name, "ros.video.camera.front.image");
  EXPECT_EQ(raw_spec.ros_topic, "/camera/front/image");
  EXPECT_EQ(raw_spec.interface_type, kImageInterfaceType);
  EXPECT_EQ(raw_spec.input_kind, VideoInputKind::RosTopic);
  EXPECT_EQ(raw_spec.selected_config_id, "default_ros");
  EXPECT_EQ(raw_spec.ingest_mode, kRawImageIngestMode);
  expectPublishConfigEq(raw_spec.publish_config, stream_config.default_publish_config);

  const auto compressed_spec =
    resolveRosVideoStreamSpec(stream_config, "/camera/front/image/compressed", kCompressedImageInterfaceType);
  EXPECT_EQ(compressed_spec.stream_key, "topic:/camera/front/image/compressed");
  EXPECT_EQ(compressed_spec.track_name, "ros.video.camera.front.image.compressed");
  EXPECT_EQ(compressed_spec.ros_topic, "/camera/front/image/compressed");
  EXPECT_EQ(compressed_spec.interface_type, kCompressedImageInterfaceType);
  EXPECT_EQ(compressed_spec.input_kind, VideoInputKind::RosTopic);
  EXPECT_EQ(compressed_spec.selected_config_id, "default_ros");
  EXPECT_EQ(compressed_spec.ingest_mode, kCompressedImageIngestMode);
  expectPublishConfigEq(compressed_spec.publish_config, stream_config.default_publish_config);
}

TEST(VideoStreamSpecTest, ResolveRosVideoStreamSpecNormalizesTopicForMatchingAndIdentifiers)
{
  VideoStreamConfig stream_config = makeDefaultVideoStreamConfig();

  RosVideoTopicRule normalized_rule =
    makeRosVideoTopicRule("normalized", "/camera/front/*", "videoconvert ! normalized-filter");
  normalized_rule.publish_config =
    makePublishConfig(VideoPublishCodec::H264, 900000, 12.0, VideoPublishSimulcast::Disabled);
  stream_config.ros_topic_rules.insert(stream_config.ros_topic_rules.begin(), normalized_rule);

  const auto spec = resolveRosVideoStreamSpec(stream_config, "  camera//front/image/  ", kImageInterfaceType);

  EXPECT_EQ(spec.stream_key, "topic:/camera/front/image");
  EXPECT_EQ(spec.track_name, "ros.video.camera.front.image");
  EXPECT_EQ(spec.ros_topic, "/camera/front/image");
  EXPECT_EQ(spec.interface_type, kImageInterfaceType);
  EXPECT_EQ(spec.selected_config_id, "normalized");
  EXPECT_EQ(spec.transform_fragment, "videoconvert ! normalized-filter");
  expectPublishConfigEq(spec.publish_config, normalized_rule.publish_config);
}

TEST(VideoStreamSpecTest, ResolveRosVideoStreamSpecRejectsInvalidInput)
{
  const auto stream_config = makeDefaultVideoStreamConfig();

  expectThrowsWithMessage<std::invalid_argument>(
    [&]() { (void)resolveRosVideoStreamSpec(stream_config, "/camera/front/image", ""); },
    "ROS topic is not a supported video type.");
  expectThrowsWithMessage<std::invalid_argument>(
    [&]() { (void)resolveRosVideoStreamSpec(stream_config, " \t\n ", kImageInterfaceType); }, "Invalid ROS topic.");
}

TEST(VideoStreamSpecTest, ResolveRosVideoStreamSpecRejectsWhenNoRuleMatches)
{
  const VideoStreamConfig stream_config;

  expectThrowsWithMessage<std::runtime_error>(
    [&]() { (void)resolveRosVideoStreamSpec(stream_config, "/camera/front/image", kImageInterfaceType); },
    "no matching video rule for topic '/camera/front/image'");
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
  EXPECT_EQ(spec.transform_fragment, "videoconvert ! specific-filter");
  expectPublishConfigEq(spec.publish_config, specific_rule.publish_config);
}

TEST(VideoStreamSpecTest, ResolveRosVideoStreamSpecExactPatternBeatsSubtreeRuleOnParentTopic)
{
  VideoStreamConfig stream_config;

  RosVideoTopicRule subtree_rule = makeRosVideoTopicRule("subtree", "/camera/front/*", "videoconvert ! subtree-filter");
  subtree_rule.publish_config =
    makePublishConfig(VideoPublishCodec::Vp8, 500000, 30.0, VideoPublishSimulcast::Disabled);

  RosVideoTopicRule exact_rule = makeRosVideoTopicRule("exact", "/camera/front", "videoconvert ! exact-filter");
  exact_rule.publish_config = makePublishConfig(VideoPublishCodec::H264, 800000, 15.0, VideoPublishSimulcast::Enabled);

  stream_config.ros_topic_rules = {subtree_rule, exact_rule};

  const auto spec = resolveRosVideoStreamSpec(stream_config, "/camera/front", kImageInterfaceType);

  EXPECT_EQ(spec.stream_key, "topic:/camera/front");
  EXPECT_EQ(spec.track_name, "ros.video.camera.front");
  EXPECT_EQ(spec.selected_config_id, "exact");
  EXPECT_EQ(spec.transform_fragment, "videoconvert ! exact-filter");
  expectPublishConfigEq(spec.publish_config, exact_rule.publish_config);
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
}

TEST(VideoStreamSpecTest, ResolveRosVideoStreamSpecDoesNotInterpolateTopicPlaceholders)
{
  VideoStreamConfig stream_config = makeDefaultVideoStreamConfig();

  stream_config.ros_topic_rules.insert(
    stream_config.ros_topic_rules.begin(), makeRosVideoTopicRule("front", "/camera/front/*", "{topic}"));

  const auto spec = resolveRosVideoStreamSpec(stream_config, "/camera/front/image", kImageInterfaceType);

  EXPECT_EQ(spec.transform_fragment, "{topic}");
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
  const auto expected_publish_config =
    makePublishConfig(VideoPublishCodec::H265, 1200000, 10.0, VideoPublishSimulcast::Disabled);

  ConfiguredVideoStreamSource configured_source;
  configured_source.ingress_fragment = "videotestsrc is-live=true pattern=black";
  configured_source.transform_fragment = "videobalance saturation=0.0";
  configured_source.publish_config = expected_publish_config;

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
  expectPublishConfigEq(spec.publish_config, expected_publish_config);
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

TEST(VideoStreamSpecTest, ResolveConfiguredSourceVideoStreamSpecRejectsInvalidNames)
{
  const VideoStreamConfig stream_config = makeDefaultVideoStreamConfig();

  expectThrowsWithMessage<std::invalid_argument>(
    [&]() { (void)resolveConfiguredSourceVideoStreamSpec(stream_config, "sources/missing"); },
    "Unknown configured video source 'sources/missing'.");
  expectThrowsWithMessage<std::invalid_argument>(
    [&]() { (void)resolveConfiguredSourceVideoStreamSpec(stream_config, " \t\n "); },
    "Invalid configured source name.");
}

}  // namespace
}  // namespace livekit_ros2_bridge
