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
#include "video_config.hpp"

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

TEST(VideoConfigTest, DefaultConfigResolvesBuiltInRawImageRule)
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
  EXPECT_EQ(spec.source_kind, VideoSourceKind::RosTopic);
  EXPECT_EQ(spec.ingest_mode, kRawImageIngestMode);
  EXPECT_EQ(spec.selected_config_key, video_defaults::kDefaultRosProfileId);
  EXPECT_EQ(spec.transform_description, video_defaults::kDefaultRosTransform);
  expectPublishConfigEq(spec.publish_config, config.publish);
}

TEST(VideoConfigTest, DefaultConfigResolvesBuiltInCompressedImageRule)
{
  const auto config = makeDefaultVideoConfig();

  const auto spec = resolveRosVideoStreamSpec(config, "/camera/front/image/compressed", kCompressedImageInterfaceType);

  EXPECT_EQ(spec.stream_key, "topic:/camera/front/image/compressed");
  EXPECT_EQ(spec.track_name, "ros.video.camera.front.image.compressed");
  EXPECT_EQ(spec.ros_topic, "/camera/front/image/compressed");
  EXPECT_EQ(spec.interface_type, kCompressedImageInterfaceType);
  EXPECT_EQ(spec.source_kind, VideoSourceKind::RosTopic);
  EXPECT_EQ(spec.ingest_mode, kCompressedImageIngestMode);
  EXPECT_EQ(spec.selected_config_key, video_defaults::kDefaultRosProfileId);
  EXPECT_EQ(spec.transform_description, video_defaults::kDefaultRosTransform);
  expectPublishConfigEq(spec.publish_config, config.publish);
}

TEST(VideoConfigTest, ResolveRosVideoStreamSpecUsesLongestMatch)
{
  VideoConfig config = makeDefaultVideoConfig();

  RosTopicRule broad_rule = makeRosRule("broad", "/camera/*", "videoconvert ! broad-filter");
  broad_rule.publish = makePublishConfig(VideoPublishCodec::Vp8, 500000, 30.0, VideoPublishSimulcast::Disabled);
  RosTopicRule specific_rule = makeRosRule("specific", "/camera/front/*", "videoconvert ! specific-filter");
  specific_rule.publish = makePublishConfig(VideoPublishCodec::H264, 800000, 15.0, VideoPublishSimulcast::Enabled);

  config.ros_topic_rules.insert(config.ros_topic_rules.begin(), broad_rule);
  config.ros_topic_rules.insert(config.ros_topic_rules.begin(), specific_rule);

  const auto spec = resolveRosVideoStreamSpec(config, "/camera/front/image", kImageInterfaceType);

  EXPECT_EQ(spec.selected_config_key, "specific");
  EXPECT_EQ(spec.ingest_mode, kRawImageIngestMode);
  EXPECT_EQ(spec.transform_description, "videoconvert ! specific-filter");
  expectPublishConfigEq(spec.publish_config, specific_rule.publish);
}

TEST(VideoConfigTest, ResolveRosVideoStreamSpecSameLengthUsesFirstDeclared)
{
  VideoConfig config = makeDefaultVideoConfig();

  const RosTopicRule first_rule = makeRosRule("first", "/camera/front/*", "videoconvert ! first-filter");
  const RosTopicRule second_rule = makeRosRule("second", "/camera/front/*", "videoconvert ! second-filter");

  config.ros_topic_rules.insert(config.ros_topic_rules.begin(), first_rule);
  config.ros_topic_rules.insert(config.ros_topic_rules.end() - 1, second_rule);

  const auto spec = resolveRosVideoStreamSpec(config, "/camera/front/image", kImageInterfaceType);

  EXPECT_EQ(spec.selected_config_key, "first");
  EXPECT_EQ(spec.transform_description, "videoconvert ! first-filter");
  expectPublishConfigEq(spec.publish_config, first_rule.publish);
}

TEST(VideoConfigTest, UserCatchAllOverridesBuiltInDefault)
{
  VideoConfig config = makeDefaultVideoConfig();

  const RosTopicRule user_rule = makeRosRule("user_default", "/*", "videoconvert ! user-filter");
  config.ros_topic_rules.insert(config.ros_topic_rules.begin(), user_rule);

  const auto spec = resolveRosVideoStreamSpec(config, "/camera/front/image", kImageInterfaceType);

  EXPECT_EQ(spec.selected_config_key, "user_default");
  EXPECT_EQ(spec.transform_description, "videoconvert ! user-filter");
  expectPublishConfigEq(spec.publish_config, user_rule.publish);
}

TEST(VideoConfigTest, ResolveRosVideoStreamSpecDoesNotInterpolateTopicPlaceholders)
{
  VideoConfig config = makeDefaultVideoConfig();
  config.ros_topic_rules.insert(config.ros_topic_rules.begin(), makeRosRule("front", "/camera/front/*", "{topic}"));

  const auto spec = resolveRosVideoStreamSpec(config, "/camera/front/image", kImageInterfaceType);

  EXPECT_EQ(spec.transform_description, "{topic}");
  expectPublishConfigEq(spec.publish_config, config.ros_topic_rules.front().publish);
}

TEST(VideoConfigTest, ResolveExternalVideoStreamSpecNormalizesExternalName)
{
  VideoConfig config = makeDefaultVideoConfig();

  ConfiguredExternalSource source;
  source.source = "videotestsrc is-live=true pattern=black";
  source.transform = "videobalance saturation=0.0";
  source.publish = makePublishConfig(VideoPublishCodec::H265, 1200000, 10.0, VideoPublishSimulcast::Disabled);
  config.external_sources.emplace("/sources/front", std::move(source));

  const auto spec = resolveExternalVideoStreamSpec(config, "  /sources/front/ ");

  EXPECT_EQ(spec.stream_key, "external:/sources/front");
  EXPECT_EQ(spec.track_name, "ros.video.external.sources.front");
  EXPECT_EQ(spec.external_name, "/sources/front");
  EXPECT_EQ(spec.source_kind, VideoSourceKind::External);
  EXPECT_EQ(spec.ingest_mode, kExternalIngestMode);
  EXPECT_EQ(spec.selected_config_key, "/sources/front");
  EXPECT_EQ(spec.source_description, "videotestsrc is-live=true pattern=black");
  EXPECT_EQ(spec.transform_description, "videobalance saturation=0.0");
  expectPublishConfigEq(
    spec.publish_config, makePublishConfig(VideoPublishCodec::H265, 1200000, 10.0, VideoPublishSimulcast::Disabled));
}

TEST(VideoConfigTest, ResolveExternalVideoStreamSpecRejectsUnknownExternalName)
{
  const VideoConfig config = makeDefaultVideoConfig();

  try {
    (void)resolveExternalVideoStreamSpec(config, "/sources/missing");
    FAIL() << "Expected invalid_argument";
  } catch (const std::invalid_argument & exc) {
    EXPECT_STREQ(exc.what(), "Unknown configured video source '/sources/missing'.");
  }
}

}  // namespace
}  // namespace livekit_ros2_bridge
