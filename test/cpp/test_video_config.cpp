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

#include "gtest/gtest.h"
#include "video_config.hpp"

namespace livekit_ros2_bridge
{
namespace
{

RosTopicRule makeRosRule(const char * id, const char * pattern, const char * pipeline, const char * alias = "image")
{
  RosTopicRule rule;
  rule.id = id;
  rule.pattern = pattern;
  rule.pipelines = {{alias, pipeline}};
  return rule;
}

TEST(VideoConfigTest, DefaultConfigResolvesBuiltInRawImageRule)
{
  const auto config = makeDefaultVideoConfig();

  ASSERT_EQ(config.ros_topic_rules.size(), 1U);
  const auto & rule = config.ros_topic_rules.front();
  EXPECT_EQ(rule.pattern, "/*");
  EXPECT_EQ(rule.id, video_defaults::kDefaultRosProfileId);
  ASSERT_EQ(rule.pipelines.size(), 2U);
  EXPECT_EQ(rule.pipelines.at("image"), video_defaults::kDefaultImagePipeline);
  EXPECT_EQ(rule.pipelines.at("compressed_image"), video_defaults::kDefaultCompressedImagePipeline);

  const auto spec = resolveRosVideoLaunchSpec(config, "/camera/front/image", "sensor_msgs/msg/Image");

  EXPECT_EQ(spec.sidecar_key, "topic:/camera/front/image");
  EXPECT_EQ(spec.ros_topic, "/camera/front/image");
  EXPECT_EQ(spec.interface_type, "sensor_msgs/msg/Image");
  EXPECT_EQ(spec.source_kind, VideoSourceKind::RosTopic);
  EXPECT_EQ(spec.ingest_mode, "raw_image");
  EXPECT_EQ(spec.selected_config_key, video_defaults::kDefaultRosProfileId);
  ASSERT_GE(spec.source_pipeline.size(), 2U);
  EXPECT_EQ(spec.source_pipeline[0], "rosrawimagesrc");
  EXPECT_EQ(spec.source_pipeline[1], "ros-topic=/camera/front/image");
}

TEST(VideoConfigTest, ResolveRosVideoLaunchSpecUsesLongestMatch)
{
  VideoConfig config = makeDefaultVideoConfig();

  const RosTopicRule broad_rule =
    makeRosRule("broad", "/camera/*", "rosrawimagesrc ros-topic={topic} ! vp8enc deadline=1");
  const RosTopicRule specific_rule = makeRosRule(
    "specific", "/camera/front/*", "rosrawimagesrc ros-topic={topic} ! vp8enc deadline=1 target-bitrate=2000000");

  // Declare broad rule first — but longer pattern should still win.
  config.ros_topic_rules.insert(config.ros_topic_rules.begin(), broad_rule);
  config.ros_topic_rules.insert(config.ros_topic_rules.begin(), specific_rule);

  const auto spec = resolveRosVideoLaunchSpec(config, "/camera/front/image", "sensor_msgs/msg/Image");

  EXPECT_EQ(spec.selected_config_key, "specific");
  EXPECT_EQ(spec.ingest_mode, "raw_image");
  EXPECT_EQ(spec.source_pipeline.back(), "target-bitrate=2000000");
}

TEST(VideoConfigTest, ResolveRosVideoLaunchSpecSameLengthUsesFirstDeclared)
{
  VideoConfig config = makeDefaultVideoConfig();

  const RosTopicRule first_rule =
    makeRosRule("first", "/camera/front/*", "rosrawimagesrc ros-topic={topic} ! first-encoder");
  const RosTopicRule second_rule =
    makeRosRule("second", "/camera/front/*", "rosrawimagesrc ros-topic={topic} ! second-encoder");

  // Both patterns are the same length — first declared should win.
  config.ros_topic_rules.insert(config.ros_topic_rules.begin(), first_rule);
  config.ros_topic_rules.insert(config.ros_topic_rules.end() - 1, second_rule);

  const auto spec = resolveRosVideoLaunchSpec(config, "/camera/front/image", "sensor_msgs/msg/Image");

  EXPECT_EQ(spec.selected_config_key, "first");
  EXPECT_EQ(spec.source_pipeline.back(), "first-encoder");
}

TEST(VideoConfigTest, UserCatchAllOverridesBuiltInDefault)
{
  VideoConfig config = makeDefaultVideoConfig();

  const RosTopicRule user_rule = makeRosRule("user_default", "/*", "rosrawimagesrc ros-topic={topic} ! user-encoder");

  // Insert before built-in catch-all, same pattern length — first-declared wins.
  config.ros_topic_rules.insert(config.ros_topic_rules.begin(), user_rule);

  const auto spec = resolveRosVideoLaunchSpec(config, "/camera/front/image", "sensor_msgs/msg/Image");

  EXPECT_EQ(spec.selected_config_key, "user_default");
  EXPECT_EQ(spec.source_pipeline.back(), "user-encoder");
}

TEST(VideoConfigTest, ResolveRosVideoLaunchSpecFallsBackToDefaultAlias)
{
  VideoConfig config;

  RosTopicRule rule = makeRosRule("catch_all", "/*", "rosrawimagesrc ros-topic={topic} ! fallback-encoder", "default");
  config.ros_topic_rules.push_back(std::move(rule));

  const auto spec = resolveRosVideoLaunchSpec(config, "/camera/front/image", "sensor_msgs/msg/Image");

  const std::vector<std::string> expected_pipeline = {
    "rosrawimagesrc", "ros-topic=/camera/front/image", "!", "fallback-encoder"};
  EXPECT_EQ(spec.selected_config_key, "catch_all");
  EXPECT_EQ(spec.ingest_mode, "raw_image");
  EXPECT_EQ(spec.source_pipeline, expected_pipeline);
}

TEST(VideoConfigTest, ResolveConfiguredVideoLaunchSpecNormalizesExternalName)
{
  VideoConfig config = makeDefaultVideoConfig();

  config.pipeline_sources.emplace(
    "/sources/front", ConfiguredPipelineSource{"/sources/front", "v4l2src device=/dev/video0 do-timestamp=true"});

  const auto spec = resolvePipelineVideoLaunchSpec(config, "  /sources/front/ ");

  EXPECT_EQ(spec.sidecar_key, "external:/sources/front");
  EXPECT_EQ(spec.external_name, "/sources/front");
  EXPECT_EQ(spec.source_kind, VideoSourceKind::Pipeline);
  EXPECT_EQ(spec.ingest_mode, "pipeline");
  EXPECT_EQ(spec.selected_config_key, "/sources/front");
  const std::vector<std::string> expected_pipeline = {"v4l2src", "device=/dev/video0", "do-timestamp=true"};
  EXPECT_EQ(spec.source_pipeline, expected_pipeline);
}

TEST(VideoConfigTest, ResolveConfiguredVideoLaunchSpecRejectsUnknownExternalName)
{
  const VideoConfig config = makeDefaultVideoConfig();

  try {
    (void)resolvePipelineVideoLaunchSpec(config, "/sources/missing");
    FAIL() << "Expected invalid_argument";
  } catch (const std::invalid_argument & exc) {
    EXPECT_STREQ(exc.what(), "Unknown configured video source '/sources/missing'.");
  }
}

}  // namespace
}  // namespace livekit_ros2_bridge
