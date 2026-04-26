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
#include "video/stream_spec.hpp"

namespace livekit_ros2_bridge::video
{
namespace
{

livekit::VideoCodec videoCodec(int value)
{
  return static_cast<livekit::VideoCodec>(value);
}

livekit::TrackPublishOptions makeTrackPublishOptions(
  std::optional<livekit::VideoCodec> codec,
  std::optional<livekit::VideoEncodingOptions> encoding,
  std::optional<bool> simulcast)
{
  livekit::TrackPublishOptions options;
  options.video_codec = codec;
  options.video_encoding = encoding;
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

RosTopicRule makeRule(const char * id, const char * pattern, const char * transform)
{
  RosTopicRule rule;
  rule.rule_id = id;
  rule.pattern = RosResourcePattern::fromCanonical(pattern);
  rule.transform_fragment = transform;
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
  return rclcpp::expand_topic_or_service_name(std::string{name}, "livekit_ros2_bridge_resource_name", "/");
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

TEST(StreamSpecTest, ClassifyRosIngestModeOnlyAcceptsSupportedExactStrings)
{
  EXPECT_EQ(classifyRosIngestMode(imageInterfaceType()), RosIngestMode::RawImage);

  EXPECT_EQ(classifyRosIngestMode(compressedImageInterfaceType()), RosIngestMode::CompressedImage);
  EXPECT_FALSE(classifyRosIngestMode(std::string{" "} + imageInterfaceType()).has_value());
}

TEST(StreamSpecTest, ResolveRosTopicSpecUsesBuiltInDefaultSelectionForSupportedTypes)
{
  const auto config = makeDefaultConfig();

  const auto raw_spec = resolveRosTopicSpec(config, "/camera/front/image", imageInterfaceType());
  const auto & raw_input = requireRosInput(raw_spec);
  EXPECT_EQ(raw_spec.stream_key, "topic:/camera/front/image");
  EXPECT_EQ(raw_spec.track_name, "lkros.video.camera.front.image");
  EXPECT_EQ(raw_input.topic, "/camera/front/image");
  EXPECT_EQ(raw_input.interface_type, imageInterfaceType());
  EXPECT_EQ(raw_input.rule_id, "default_ros");
  EXPECT_EQ(raw_input.ingest_mode, RosIngestMode::RawImage);
  expectTrackPublishOptionsEq(raw_spec.publish_options, config.default_publish_options);

  const auto compressed_spec =
    resolveRosTopicSpec(config, "/camera/front/image/compressed", compressedImageInterfaceType());
  const auto & compressed_input = requireRosInput(compressed_spec);
  EXPECT_EQ(compressed_spec.stream_key, "topic:/camera/front/image/compressed");
  EXPECT_EQ(compressed_spec.track_name, "lkros.video.camera.front.image.compressed");
  EXPECT_EQ(compressed_input.rule_id, "default_ros");
  EXPECT_EQ(compressed_input.ingest_mode, RosIngestMode::CompressedImage);
  expectTrackPublishOptionsEq(compressed_spec.publish_options, config.default_publish_options);
}

TEST(StreamSpecTest, ResolveRosTopicSpecExpandsRelativeTopicForMatchingAndIdentifiers)
{
  StreamConfig config = makeDefaultConfig();

  RosTopicRule rule = makeRule("normalized", "/camera/front/*", "videoconvert ! normalized-filter");
  rule.publish_options = makeTrackPublishOptions(videoCodec(1), livekit::VideoEncodingOptions{900000U, 12.0}, false);
  config.ros_topic_rules.insert(config.ros_topic_rules.begin(), rule);

  const auto spec = resolveRosTopicSpec(config, "  camera/front/image  ", imageInterfaceType());
  const auto & input = requireRosInput(spec);
  const std::string expected_topic = expandRosTopicName("camera/front/image");

  EXPECT_EQ(spec.stream_key, "topic:" + expected_topic);
  EXPECT_EQ(spec.track_name, "lkros.video.camera.front.image");
  EXPECT_EQ(input.topic, expected_topic);
  EXPECT_EQ(input.rule_id, "normalized");
  EXPECT_EQ(input.transform_fragment, "videoconvert ! normalized-filter");
  expectTrackPublishOptionsEq(spec.publish_options, rule.publish_options);
}

TEST(StreamSpecTest, ResolveRosTopicSpecRejectsRootTopic)
{
  const auto config = makeDefaultConfig();

  expectThrowsWithMessage<std::invalid_argument>(
    [&]() { (void)resolveRosTopicSpec(config, "/", imageInterfaceType()); }, "Invalid ROS topic.");
}

TEST(StreamSpecTest, ResolveRosTopicSpecRejectsInvalidInput)
{
  const auto config = makeDefaultConfig();

  expectThrowsWithMessage<std::invalid_argument>(
    [&]() { (void)resolveRosTopicSpec(config, "/camera/front/image", ""); },
    "ROS topic is not a supported video type.");
  expectThrowsWithMessage<std::invalid_argument>(
    [&]() { (void)resolveRosTopicSpec(config, " \t\n ", imageInterfaceType()); }, "Invalid ROS topic.");
}

TEST(StreamSpecTest, ResolveRosTopicSpecRejectsWhenNoRuleMatches)
{
  const StreamConfig config;

  expectThrowsWithMessage<std::runtime_error>(
    [&]() { (void)resolveRosTopicSpec(config, "/camera/front/image", imageInterfaceType()); },
    "no matching video rule for topic '/camera/front/image'");
}

TEST(StreamSpecTest, ResolveRosTopicSpecUsesLongestMatch)
{
  StreamConfig config = makeDefaultConfig();

  RosTopicRule broad = makeRule("broad", "/camera/*", "videoconvert ! broad-filter");
  broad.publish_options = makeTrackPublishOptions(videoCodec(0), livekit::VideoEncodingOptions{500000U, 30.0}, false);
  RosTopicRule specific = makeRule("specific", "/camera/front/*", "videoconvert ! specific-filter");
  specific.publish_options = makeTrackPublishOptions(videoCodec(1), livekit::VideoEncodingOptions{800000U, 15.0}, true);

  config.ros_topic_rules.insert(config.ros_topic_rules.begin(), broad);
  config.ros_topic_rules.insert(config.ros_topic_rules.begin(), specific);

  const auto spec = resolveRosTopicSpec(config, "/camera/front/image", imageInterfaceType());

  EXPECT_EQ(requireRosInput(spec).rule_id, "specific");
  expectTrackPublishOptionsEq(spec.publish_options, specific.publish_options);
}

TEST(StreamSpecTest, ResolveRosTopicSpecSameLengthUsesFirstDeclared)
{
  StreamConfig config = makeDefaultConfig();

  const RosTopicRule first = makeRule("first", "/camera/front/*", "videoconvert ! first-filter");
  const RosTopicRule second = makeRule("second", "/camera/front/*", "videoconvert ! second-filter");

  config.ros_topic_rules.insert(config.ros_topic_rules.begin(), first);
  config.ros_topic_rules.insert(config.ros_topic_rules.end() - 1, second);

  const auto spec = resolveRosTopicSpec(config, "/camera/front/image", imageInterfaceType());

  EXPECT_EQ(requireRosInput(spec).rule_id, "first");
}

TEST(StreamSpecTest, ResolveRosTopicSpecDoesNotInterpolateTopicPlaceholders)
{
  StreamConfig config = makeDefaultConfig();

  config.ros_topic_rules.insert(config.ros_topic_rules.begin(), makeRule("front", "/camera/front/*", "{topic}"));

  const auto spec = resolveRosTopicSpec(config, "/camera/front/image", imageInterfaceType());

  EXPECT_EQ(requireRosInput(spec).transform_fragment, "{topic}");
}

TEST(StreamSpecTest, ResolveOtherSourceSpecTrimsOtherSourceName)
{
  StreamConfig config = makeDefaultConfig();
  const auto expected_publish_options =
    makeTrackPublishOptions(videoCodec(4), livekit::VideoEncodingOptions{1200000U, 10.0}, false);

  OtherSource source;
  source.source_fragment = "videotestsrc is-live=true pattern=black";
  source.transform_fragment = "videobalance saturation=0.0";
  source.publish_options = expected_publish_options;

  config.other_sources.emplace("front_camera", std::move(source));

  const auto spec = resolveOtherSourceSpec(config, "  front_camera  ");
  const auto & input = requireOtherInput(spec);

  EXPECT_EQ(spec.stream_key, "other_video:front_camera");
  EXPECT_EQ(spec.track_name, "lkros.video.other.front_camera");
  EXPECT_EQ(input.name, "front_camera");
  EXPECT_EQ(input.source_fragment, "videotestsrc is-live=true pattern=black");
  EXPECT_EQ(input.transform_fragment, "videobalance saturation=0.0");
  expectTrackPublishOptionsEq(spec.publish_options, expected_publish_options);
}

TEST(StreamSpecTest, ResolveOtherSourceSpecPercentEncodesTrackNameSuffix)
{
  StreamConfig config = makeDefaultConfig();

  OtherSource source;
  source.source_fragment = "videotestsrc is-live=true pattern=black";

  config.other_sources.emplace("/sources/front:rgb%", std::move(source));

  const auto spec = resolveOtherSourceSpec(config, "/sources/front:rgb%");

  EXPECT_EQ(spec.track_name, "lkros.video.other.%2Fsources%2Ffront%3Argb%25");
}

TEST(StreamSpecTest, ResolveOtherSourceSpecRejectsInvalidNames)
{
  const StreamConfig config = makeDefaultConfig();

  expectThrowsWithMessage<std::invalid_argument>(
    [&]() { (void)resolveOtherSourceSpec(config, "sources/missing"); },
    "Unknown other video source 'sources/missing'.");
  expectThrowsWithMessage<std::invalid_argument>(
    [&]() { (void)resolveOtherSourceSpec(config, " \t\n "); }, "Invalid other video name.");
}

}  // namespace
}  // namespace livekit_ros2_bridge::video
