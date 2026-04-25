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

#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "gtest/gtest.h"
#include "rclcpp/expand_topic_or_service_name.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "utils/ros_resource_name_utils.hpp"
#include "video_stream_spec.hpp"

namespace livekit_ros2_bridge
{
namespace
{

livekit::VideoCodec videoCodec(int value)
{
  return static_cast<livekit::VideoCodec>(value);
}

livekit::TrackPublishOptions makeTrackPublishOptions(
  std::optional<livekit::VideoCodec> video_codec,
  std::optional<livekit::VideoEncodingOptions> video_encoding,
  std::optional<bool> simulcast)
{
  livekit::TrackPublishOptions options;
  options.video_codec = video_codec;
  options.video_encoding = video_encoding;
  options.simulcast = simulcast;
  return options;
}

void expectVideoEncodingEq(
  const std::optional<livekit::VideoEncodingOptions> & actual,
  const std::optional<livekit::VideoEncodingOptions> & expected)
{
  EXPECT_EQ(actual.has_value(), expected.has_value());
  if (!actual.has_value() || !expected.has_value()) {
    return;
  }
  EXPECT_EQ(actual->max_bitrate, expected->max_bitrate);
  EXPECT_DOUBLE_EQ(actual->max_framerate, expected->max_framerate);
}

void expectTrackPublishOptionsEq(
  const livekit::TrackPublishOptions & actual, const livekit::TrackPublishOptions & expected)
{
  EXPECT_EQ(actual.video_codec, expected.video_codec);
  expectVideoEncodingEq(actual.video_encoding, expected.video_encoding);
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

const char * imageInterfaceType()
{
  return rosidl_generator_traits::name<sensor_msgs::msg::Image>();
}

const char * compressedImageInterfaceType()
{
  return rosidl_generator_traits::name<sensor_msgs::msg::CompressedImage>();
}

std::string expandRosTopicName(std::string_view name)
{
  return rclcpp::expand_topic_or_service_name(
    std::string{name},
    ros_resource_name_utils_detail::kResourceNameExpansionNode,
    ros_resource_name_utils_detail::kResourceNameExpansionNamespace);
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
  EXPECT_EQ(classifyRosVideoIngestMode(imageInterfaceType()), RosVideoIngestMode::RawImage);

  EXPECT_EQ(classifyRosVideoIngestMode(compressedImageInterfaceType()), RosVideoIngestMode::CompressedImage);
  EXPECT_FALSE(classifyRosVideoIngestMode(std::string{" "} + imageInterfaceType()).has_value());
}

TEST(VideoStreamSpecTest, ResolveRosVideoTopicSpecUsesBuiltInDefaultSelectionForSupportedTypes)
{
  const auto config = makeDefaultVideoStreamConfig();

  const auto raw_spec = resolveRosVideoTopicSpec(config, "/camera/front/image", imageInterfaceType());
  const auto & raw_input = requireRosVideoInput(raw_spec);
  EXPECT_EQ(raw_spec.stream_key, "topic:/camera/front/image");
  EXPECT_EQ(raw_spec.track_name, "lkros.video.camera.front.image");
  EXPECT_EQ(raw_input.topic, "/camera/front/image");
  EXPECT_EQ(raw_input.interface_type, imageInterfaceType());
  EXPECT_EQ(raw_input.rule_id, "default_ros");
  EXPECT_EQ(raw_input.ingest_mode, RosVideoIngestMode::RawImage);
  expectTrackPublishOptionsEq(raw_spec.publish_config, config.default_publish_config);

  const auto compressed_spec =
    resolveRosVideoTopicSpec(config, "/camera/front/image/compressed", compressedImageInterfaceType());
  const auto & compressed_input = requireRosVideoInput(compressed_spec);
  EXPECT_EQ(compressed_spec.stream_key, "topic:/camera/front/image/compressed");
  EXPECT_EQ(compressed_spec.track_name, "lkros.video.camera.front.image.compressed");
  EXPECT_EQ(compressed_input.rule_id, "default_ros");
  EXPECT_EQ(compressed_input.ingest_mode, RosVideoIngestMode::CompressedImage);
  expectTrackPublishOptionsEq(compressed_spec.publish_config, config.default_publish_config);
}

TEST(VideoStreamSpecTest, ResolveRosVideoTopicSpecExpandsRelativeTopicForMatchingAndIdentifiers)
{
  VideoStreamConfig config = makeDefaultVideoStreamConfig();

  RosVideoTopicRule normalized_rule = makeRule("normalized", "/camera/front/*", "videoconvert ! normalized-filter");
  normalized_rule.publish_config =
    makeTrackPublishOptions(videoCodec(1), livekit::VideoEncodingOptions{900000U, 12.0}, false);
  config.ros_topic_rules.insert(config.ros_topic_rules.begin(), normalized_rule);

  const auto spec = resolveRosVideoTopicSpec(config, "  camera/front/image  ", imageInterfaceType());
  const auto & input = requireRosVideoInput(spec);
  const std::string expected_topic = expandRosTopicName("camera/front/image");

  EXPECT_EQ(spec.stream_key, "topic:" + expected_topic);
  EXPECT_EQ(spec.track_name, "lkros.video.camera.front.image");
  EXPECT_EQ(input.topic, expected_topic);
  EXPECT_EQ(input.rule_id, "normalized");
  EXPECT_EQ(input.transform_fragment, "videoconvert ! normalized-filter");
  expectTrackPublishOptionsEq(spec.publish_config, normalized_rule.publish_config);
}

TEST(VideoStreamSpecTest, ResolveRosVideoTopicSpecRejectsRootTopic)
{
  const auto config = makeDefaultVideoStreamConfig();

  expectThrowsWithMessage<std::invalid_argument>(
    [&]() { (void)resolveRosVideoTopicSpec(config, "/", imageInterfaceType()); }, "Invalid ROS topic.");
}

TEST(VideoStreamSpecTest, ResolveRosVideoTopicSpecRejectsInvalidInput)
{
  const auto config = makeDefaultVideoStreamConfig();

  expectThrowsWithMessage<std::invalid_argument>(
    [&]() { (void)resolveRosVideoTopicSpec(config, "/camera/front/image", ""); },
    "ROS topic is not a supported video type.");
  expectThrowsWithMessage<std::invalid_argument>(
    [&]() { (void)resolveRosVideoTopicSpec(config, " \t\n ", imageInterfaceType()); }, "Invalid ROS topic.");
}

TEST(VideoStreamSpecTest, ResolveRosVideoTopicSpecRejectsWhenNoRuleMatches)
{
  const VideoStreamConfig config;

  expectThrowsWithMessage<std::runtime_error>(
    [&]() { (void)resolveRosVideoTopicSpec(config, "/camera/front/image", imageInterfaceType()); },
    "no matching video rule for topic '/camera/front/image'");
}

TEST(VideoStreamSpecTest, ResolveRosVideoTopicSpecUsesLongestMatch)
{
  VideoStreamConfig config = makeDefaultVideoStreamConfig();

  RosVideoTopicRule broad_rule = makeRule("broad", "/camera/*", "videoconvert ! broad-filter");
  broad_rule.publish_config =
    makeTrackPublishOptions(videoCodec(0), livekit::VideoEncodingOptions{500000U, 30.0}, false);
  RosVideoTopicRule specific_rule = makeRule("specific", "/camera/front/*", "videoconvert ! specific-filter");
  specific_rule.publish_config =
    makeTrackPublishOptions(videoCodec(1), livekit::VideoEncodingOptions{800000U, 15.0}, true);

  config.ros_topic_rules.insert(config.ros_topic_rules.begin(), broad_rule);
  config.ros_topic_rules.insert(config.ros_topic_rules.begin(), specific_rule);

  const auto spec = resolveRosVideoTopicSpec(config, "/camera/front/image", imageInterfaceType());

  EXPECT_EQ(requireRosVideoInput(spec).rule_id, "specific");
  expectTrackPublishOptionsEq(spec.publish_config, specific_rule.publish_config);
}

TEST(VideoStreamSpecTest, ResolveRosVideoTopicSpecSameLengthUsesFirstDeclared)
{
  VideoStreamConfig config = makeDefaultVideoStreamConfig();

  const RosVideoTopicRule first_rule = makeRule("first", "/camera/front/*", "videoconvert ! first-filter");
  const RosVideoTopicRule second_rule = makeRule("second", "/camera/front/*", "videoconvert ! second-filter");

  config.ros_topic_rules.insert(config.ros_topic_rules.begin(), first_rule);
  config.ros_topic_rules.insert(config.ros_topic_rules.end() - 1, second_rule);

  const auto spec = resolveRosVideoTopicSpec(config, "/camera/front/image", imageInterfaceType());

  EXPECT_EQ(requireRosVideoInput(spec).rule_id, "first");
}

TEST(VideoStreamSpecTest, ResolveRosVideoTopicSpecDoesNotInterpolateTopicPlaceholders)
{
  VideoStreamConfig config = makeDefaultVideoStreamConfig();

  config.ros_topic_rules.insert(config.ros_topic_rules.begin(), makeRule("front", "/camera/front/*", "{topic}"));

  const auto spec = resolveRosVideoTopicSpec(config, "/camera/front/image", imageInterfaceType());

  EXPECT_EQ(requireRosVideoInput(spec).transform_fragment, "{topic}");
}

TEST(VideoStreamSpecTest, ResolveOtherVideoSourceSpecTrimsOtherVideoSourceName)
{
  VideoStreamConfig config = makeDefaultVideoStreamConfig();
  const auto expected_publish_config =
    makeTrackPublishOptions(videoCodec(4), livekit::VideoEncodingOptions{1200000U, 10.0}, false);

  OtherVideoSource source_config;
  source_config.ingress_fragment = "videotestsrc is-live=true pattern=black";
  source_config.transform_fragment = "videobalance saturation=0.0";
  source_config.publish_config = expected_publish_config;

  config.other_video_sources.emplace("front_camera", std::move(source_config));

  const auto spec = resolveOtherVideoSourceSpec(config, "  front_camera  ");
  const auto & input = requireOtherVideoInput(spec);

  EXPECT_EQ(spec.stream_key, "other_video:front_camera");
  EXPECT_EQ(spec.track_name, "lkros.video.other.front_camera");
  EXPECT_EQ(input.name, "front_camera");
  EXPECT_EQ(input.ingress_fragment, "videotestsrc is-live=true pattern=black");
  EXPECT_EQ(input.transform_fragment, "videobalance saturation=0.0");
  expectTrackPublishOptionsEq(spec.publish_config, expected_publish_config);
}

TEST(VideoStreamSpecTest, ResolveOtherVideoSourceSpecPercentEncodesTrackNameSuffix)
{
  VideoStreamConfig config = makeDefaultVideoStreamConfig();

  OtherVideoSource source_config;
  source_config.ingress_fragment = "videotestsrc is-live=true pattern=black";

  config.other_video_sources.emplace("/sources/front:rgb%", std::move(source_config));

  const auto spec = resolveOtherVideoSourceSpec(config, "/sources/front:rgb%");

  EXPECT_EQ(spec.track_name, "lkros.video.other.%2Fsources%2Ffront%3Argb%25");
}

TEST(VideoStreamSpecTest, ResolveOtherVideoSourceSpecRejectsInvalidNames)
{
  const VideoStreamConfig config = makeDefaultVideoStreamConfig();

  expectThrowsWithMessage<std::invalid_argument>(
    [&]() { (void)resolveOtherVideoSourceSpec(config, "sources/missing"); },
    "Unknown other video source 'sources/missing'.");
  expectThrowsWithMessage<std::invalid_argument>(
    [&]() { (void)resolveOtherVideoSourceSpec(config, " \t\n "); }, "Invalid other video name.");
}

}  // namespace
}  // namespace livekit_ros2_bridge
