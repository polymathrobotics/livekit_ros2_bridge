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

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "ros_test_support.hpp"
#include "runtime_config.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr char kLivekitTokenEnvVar[] = "LIVEKIT_TOKEN";

rclcpp::NodeOptions makeBaseOptions()
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("livekit.url", "ws://test:7880");
  return options;
}

rclcpp::NodeOptions makeStaticTokenOptions()
{
  auto options = makeBaseOptions();
  options.append_parameter_override("livekit.token", "static-token");
  return options;
}

RuntimeConfig loadRuntimeConfigForNode(const std::string & node_name, const rclcpp::NodeOptions & options)
{
  auto node = std::make_shared<rclcpp::Node>(node_name, options);
  return loadRuntimeConfig(node->get_node_parameters_interface());
}

void expectConfigError(const std::string & node_name, const rclcpp::NodeOptions & options, const char * expected_error)
{
  try {
    (void)loadRuntimeConfigForNode(node_name, options);
    FAIL() << "Expected loadRuntimeConfig to throw '" << expected_error << "'";
  } catch (const std::runtime_error & error) {
    EXPECT_STREQ(error.what(), expected_error);
  }
}

void expectConfigErrorContains(
  const std::string & node_name, const rclcpp::NodeOptions & options, const char * expected_error_fragment)
{
  try {
    (void)loadRuntimeConfigForNode(node_name, options);
    FAIL() << "Expected loadRuntimeConfig to throw an error containing '" << expected_error_fragment << "'";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(std::string(error.what()).find(expected_error_fragment), std::string::npos)
      << "actual error: " << error.what();
  }
}

void appendVideoPublishOverrides(
  rclcpp::NodeOptions & options, const char * codec, int max_bitrate_bps, double max_framerate, const char * simulcast)
{
  options.append_parameter_override("video.publish.codec", codec);
  options.append_parameter_override("video.publish.max_bitrate_bps", max_bitrate_bps);
  options.append_parameter_override("video.publish.max_framerate", max_framerate);
  options.append_parameter_override("video.publish.simulcast", simulcast);
}

void expectPublishConfigEq(
  const VideoPublishConfig & actual,
  VideoPublishCodec codec,
  std::uint64_t max_bitrate_bps,
  double max_framerate,
  VideoPublishSimulcast simulcast)
{
  EXPECT_EQ(actual.codec, codec);
  EXPECT_EQ(actual.max_bitrate_bps, max_bitrate_bps);
  EXPECT_DOUBLE_EQ(actual.max_framerate, max_framerate);
  EXPECT_EQ(actual.simulcast, simulcast);
}

void expectPublishConfigEq(const VideoPublishConfig & actual, const VideoPublishConfig & expected)
{
  expectPublishConfigEq(actual, expected.codec, expected.max_bitrate_bps, expected.max_framerate, expected.simulcast);
}

void expectSubscriptionQosOverrideEq(
  const TopicSubscriptionQosOverride & actual,
  const char * id,
  const char * pattern,
  SubscriptionQosReliabilityMode reliability,
  SubscriptionQosDurabilityMode durability)
{
  EXPECT_EQ(actual.id, id);
  EXPECT_EQ(actual.pattern, pattern);
  EXPECT_EQ(actual.reliability, reliability);
  EXPECT_EQ(actual.durability, durability);
}

}  // namespace

class RuntimeConfigTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    static test_support::ScopedRclcppInit rclcpp_init;
  }
};

TEST_F(RuntimeConfigTest, StaticTokenStartupLoadsConnectionSettings)
{
  const RuntimeConfig config = loadRuntimeConfigForNode("startup_config_static_token", makeStaticTokenOptions());

  EXPECT_EQ(config.livekit.url, "ws://test:7880");
  EXPECT_EQ(config.livekit.access_token, "static-token");
}

TEST_F(RuntimeConfigTest, FallsBackToEnvironmentTokenWhenParameterIsUnset)
{
  test_support::ScopedEnvironmentVariable env_token(kLivekitTokenEnvVar, std::string("env-token"));

  const RuntimeConfig config = loadRuntimeConfigForNode("startup_config_env_token", makeBaseOptions());

  EXPECT_EQ(config.livekit.access_token, "env-token");
}

TEST_F(RuntimeConfigTest, ParameterTokenWinsOverEnvironmentToken)
{
  test_support::ScopedEnvironmentVariable env_token(kLivekitTokenEnvVar, std::string("env-token"));

  const RuntimeConfig config =
    loadRuntimeConfigForNode("startup_config_parameter_token_precedence", makeStaticTokenOptions());

  EXPECT_EQ(config.livekit.access_token, "static-token");
}

TEST_F(RuntimeConfigTest, DefaultVideoConfigAddsBuiltInCatchAllRosRule)
{
  const RuntimeConfig config =
    loadRuntimeConfigForNode("startup_config_default_video_rule_shape", makeStaticTokenOptions());

  ASSERT_EQ(config.video_stream.ros_topic_rules.size(), 1U);
  const auto & rule = config.video_stream.ros_topic_rules.front();
  EXPECT_EQ(rule.rule_id, "default_ros");
  EXPECT_EQ(rule.pattern, "/*");
  EXPECT_EQ(rule.transform_fragment, "");
  expectPublishConfigEq(rule.publish_config, config.video_stream.default_publish_config);
}

TEST_F(RuntimeConfigTest, WatchdogDefaultsAndOverridesLoadFromParameters)
{
  const RuntimeConfig default_config =
    loadRuntimeConfigForNode("startup_config_watchdog_defaults", makeStaticTokenOptions());

  EXPECT_TRUE(default_config.health.watchdog_enabled);
  EXPECT_EQ(default_config.health.watchdog_recovery_timeout, std::chrono::minutes(10));

  auto options = makeStaticTokenOptions();
  options.append_parameter_override("health.watchdog.enabled", false);
  options.append_parameter_override("health.watchdog.recovery_timeout_seconds", 12.5);

  const RuntimeConfig overridden_config = loadRuntimeConfigForNode("startup_config_watchdog_overrides", options);

  EXPECT_FALSE(overridden_config.health.watchdog_enabled);
  EXPECT_EQ(overridden_config.health.watchdog_recovery_timeout, std::chrono::milliseconds(12500));
}

TEST_F(RuntimeConfigTest, AccessRulesLoadIntoRuntimeAccessPolicy)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("access.rules.publish.allow", std::vector<std::string>{"/camera/*"});
  options.append_parameter_override("access.rules.publish.deny", std::vector<std::string>{"/camera/private/*"});
  options.append_parameter_override("access.rules.subscribe.allow", std::vector<std::string>{"*"});
  options.append_parameter_override("access.rules.subscribe.deny", std::vector<std::string>{"/private/*"});
  options.append_parameter_override("access.rules.service.allow", std::vector<std::string>{"/robot/reset"});

  const RuntimeConfig config = loadRuntimeConfigForNode("startup_config_access_rules", options);

  EXPECT_TRUE(config.access_policy.allows(AccessOperation::Publish, "/camera/front"));
  EXPECT_FALSE(config.access_policy.allows(AccessOperation::Publish, "/camera/private/front"));
  EXPECT_TRUE(config.access_policy.allows(AccessOperation::Subscribe, "/camera/front"));
  EXPECT_FALSE(config.access_policy.allows(AccessOperation::Subscribe, "/private/diagnostics"));
  EXPECT_TRUE(config.access_policy.allows(AccessOperation::CallService, "/robot/reset"));
  EXPECT_FALSE(config.access_policy.allows(AccessOperation::CallService, "/robot/status"));
}

TEST_F(RuntimeConfigTest, VideoProfilingDefaultsAndOverridesLoadFromParameters)
{
  const RuntimeConfig default_config =
    loadRuntimeConfigForNode("startup_config_video_profiling_defaults", makeStaticTokenOptions());

  EXPECT_FALSE(default_config.video_profiling.enabled);
  EXPECT_EQ(default_config.video_profiling.summary_interval, kVideoProfilingDefaultSummaryInterval);
  EXPECT_EQ(default_config.video_profiling.trace_file, kVideoProfilingDefaultTraceFile);
  EXPECT_EQ(default_config.video_profiling.trace_max_events, kVideoProfilingDefaultTraceMaxEvents);

  auto options = makeStaticTokenOptions();
  options.append_parameter_override("debug.video_profiling.enabled", true);
  options.append_parameter_override("debug.video_profiling.summary_interval_ms", 1200);
  options.append_parameter_override("debug.video_profiling.trace_file", "/tmp/bridge.trace.json");
  options.append_parameter_override("debug.video_profiling.trace_max_events", 4096);

  const RuntimeConfig overridden_config = loadRuntimeConfigForNode("startup_config_video_profiling_overrides", options);

  EXPECT_TRUE(overridden_config.video_profiling.enabled);
  EXPECT_EQ(overridden_config.video_profiling.summary_interval, std::chrono::milliseconds(1200));
  EXPECT_EQ(overridden_config.video_profiling.trace_file, "/tmp/bridge.trace.json");
  EXPECT_EQ(overridden_config.video_profiling.trace_max_events, 4096U);
}

TEST_F(RuntimeConfigTest, NullParametersInterfaceIsRejected)
{
  try {
    (void)loadRuntimeConfig(nullptr);
    FAIL() << "Expected loadRuntimeConfig to reject a null parameters interface";
  } catch (const std::invalid_argument & error) {
    EXPECT_STREQ(error.what(), "parameters_interface is required");
  }
}

TEST_F(RuntimeConfigTest, MissingRequiredStartupParametersThrow)
{
  test_support::ScopedEnvironmentVariable env_token(kLivekitTokenEnvVar, std::nullopt);
  expectConfigError(
    "startup_config_missing_token",
    makeBaseOptions(),
    "LiveKit startup token is required; set livekit.token or LIVEKIT_TOKEN");

  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("livekit.token", "static-token");

    expectConfigErrorContains("startup_config_missing_url", options, "Parameter 'livekit.url' cannot be empty");
  }
}

TEST_F(RuntimeConfigTest, GeneratedVideoEntriesLoadFromSplitParams)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("video_topic_rule_ids", std::vector<std::string>{"front_camera"});
  options.append_parameter_override("video_configured_source_ids", std::vector<std::string>{"front_rtsp"});
  options.append_parameter_override("video.topic_rules.front_camera.pattern", "/camera/front/*");
  options.append_parameter_override(
    "video.topic_rules.front_camera.transform", "videoconvert ! videoscale ! video/x-raw,width=640,height=360");
  options.append_parameter_override(
    "video.configured_sources.front_rtsp.source", "videotestsrc is-live=true pattern=ball");
  options.append_parameter_override("video.configured_sources.front_rtsp.transform", "videobalance saturation=0.0");

  const RuntimeConfig config = loadRuntimeConfigForNode("startup_config_video_params", options);

  ASSERT_EQ(config.video_stream.ros_topic_rules.size(), 2U);
  const auto & front_rule = config.video_stream.ros_topic_rules.front();
  EXPECT_EQ(front_rule.rule_id, "front_camera");
  EXPECT_EQ(front_rule.pattern, "/camera/front/*");
  EXPECT_EQ(front_rule.transform_fragment, "videoconvert ! videoscale ! video/x-raw,width=640,height=360");
  const auto & fallback_rule = config.video_stream.ros_topic_rules.back();
  EXPECT_EQ(fallback_rule.rule_id, "default_ros");
  ASSERT_EQ(config.video_stream.configured_sources.size(), 1U);
  EXPECT_EQ(
    config.video_stream.configured_sources.at("front_rtsp").ingress_fragment, "videotestsrc is-live=true pattern=ball");
  EXPECT_EQ(config.video_stream.configured_sources.at("front_rtsp").transform_fragment, "videobalance saturation=0.0");
}

TEST_F(RuntimeConfigTest, VideoPublishConfigLoadsFromUnifiedParams)
{
  auto options = makeStaticTokenOptions();
  appendVideoPublishOverrides(options, "h264", 900000, 24.0, "enabled");

  const RuntimeConfig config = loadRuntimeConfigForNode("startup_config_video_publish_params", options);

  expectPublishConfigEq(
    config.video_stream.default_publish_config, VideoPublishCodec::H264, 900000U, 24.0, VideoPublishSimulcast::Enabled);
}

TEST_F(RuntimeConfigTest, GeneratedSubscriptionQosOverridesLoadFromUnifiedParams)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("subscription_qos_overrides_ids", std::vector<std::string>{"camera", "front"});
  options.append_parameter_override("subscription.qos_overrides.camera.pattern", "/camera/*");
  options.append_parameter_override("subscription.qos_overrides.camera.reliability", "best_effort");
  options.append_parameter_override("subscription.qos_overrides.camera.durability", "auto");
  options.append_parameter_override("subscription.qos_overrides.front.pattern", "/camera/front");
  options.append_parameter_override("subscription.qos_overrides.front.reliability", "auto");
  options.append_parameter_override("subscription.qos_overrides.front.durability", "transient_local");

  const RuntimeConfig config = loadRuntimeConfigForNode("startup_config_subscription_qos_params", options);

  ASSERT_EQ(config.subscription_qos.topic_overrides.size(), 2U);
  expectSubscriptionQosOverrideEq(
    config.subscription_qos.topic_overrides[0],
    "camera",
    "/camera/*",
    SubscriptionQosReliabilityMode::kBestEffort,
    SubscriptionQosDurabilityMode::kAuto);
  expectSubscriptionQosOverrideEq(
    config.subscription_qos.topic_overrides[1],
    "front",
    "/camera/front",
    SubscriptionQosReliabilityMode::kAuto,
    SubscriptionQosDurabilityMode::kTransientLocal);
}

TEST_F(RuntimeConfigTest, SubscriptionQosOverrideNormalizesPattern)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("subscription_qos_overrides_ids", std::vector<std::string>{"front"});
  options.append_parameter_override("subscription.qos_overrides.front.pattern", " //camera/front/ ");
  options.append_parameter_override("subscription.qos_overrides.front.reliability", "auto");
  options.append_parameter_override("subscription.qos_overrides.front.durability", "auto");

  const RuntimeConfig config =
    loadRuntimeConfigForNode("startup_config_subscription_qos_pattern_normalization", options);

  ASSERT_EQ(config.subscription_qos.topic_overrides.size(), 1U);
  EXPECT_EQ(config.subscription_qos.topic_overrides.front().pattern, "/camera/front");
}

TEST_F(RuntimeConfigTest, DuplicateSubscriptionQosOverrideIdReportsSectionSpecificError)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("subscription_qos_overrides_ids", std::vector<std::string>{"camera", "camera"});
  options.append_parameter_override("subscription.qos_overrides.camera.pattern", "/camera/*");
  options.append_parameter_override("subscription.qos_overrides.camera.reliability", "auto");
  options.append_parameter_override("subscription.qos_overrides.camera.durability", "auto");

  expectConfigError(
    "startup_config_duplicate_subscription_qos_override_id",
    options,
    "duplicate subscription QoS override id 'camera'");
}

TEST_F(RuntimeConfigTest, SubscriptionQosOverrideRejectsEmptyPattern)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("subscription_qos_overrides_ids", std::vector<std::string>{"camera"});
  options.append_parameter_override("subscription.qos_overrides.camera.pattern", "   ");
  options.append_parameter_override("subscription.qos_overrides.camera.reliability", "auto");
  options.append_parameter_override("subscription.qos_overrides.camera.durability", "auto");

  expectConfigError(
    "startup_config_empty_subscription_qos_pattern", options, "subscription.qos_overrides pattern must not be empty");
}

TEST_F(RuntimeConfigTest, UnsupportedSubscriptionQosReliabilityIsRejectedByParameterLibrary)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("subscription_qos_overrides_ids", std::vector<std::string>{"camera"});
  options.append_parameter_override("subscription.qos_overrides.camera.pattern", "/camera/*");
  options.append_parameter_override("subscription.qos_overrides.camera.reliability", "sometimes");
  options.append_parameter_override("subscription.qos_overrides.camera.durability", "auto");

  expectConfigErrorContains(
    "startup_config_invalid_subscription_qos_reliability",
    options,
    "Parameter 'subscription.qos_overrides.camera.reliability' with the value 'sometimes' is not in the set");
}

TEST_F(RuntimeConfigTest, RosVideoEntryWithoutTransformUsesEmptyTransform)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("video_topic_rule_ids", std::vector<std::string>{"front"});
  options.append_parameter_override("video.topic_rules.front.pattern", "/camera/front/*");

  const RuntimeConfig config = loadRuntimeConfigForNode("startup_config_ros_empty_transform", options);

  ASSERT_EQ(config.video_stream.ros_topic_rules.size(), 2U);
  const auto & rule = config.video_stream.ros_topic_rules.front();
  EXPECT_EQ(rule.rule_id, "front");
  EXPECT_EQ(rule.pattern, "/camera/front/*");
  EXPECT_EQ(rule.transform_fragment, "");
  expectPublishConfigEq(rule.publish_config, config.video_stream.default_publish_config);
}

TEST_F(RuntimeConfigTest, VideoPublishOverrideCanSetSingleFieldWithoutTransformForEachEntryType)
{
  {
    auto options = makeStaticTokenOptions();
    appendVideoPublishOverrides(options, "h264", 900000, 24.0, "enabled");
    options.append_parameter_override("video_topic_rule_ids", std::vector<std::string>{"front"});
    options.append_parameter_override("video.topic_rules.front.pattern", "/camera/front/*");
    options.append_parameter_override("video.topic_rules.front.publish.max_framerate", 15.0);

    const RuntimeConfig config = loadRuntimeConfigForNode("startup_config_ros_publish_override", options);

    ASSERT_FALSE(config.video_stream.ros_topic_rules.empty());
    expectPublishConfigEq(
      config.video_stream.ros_topic_rules.front().publish_config,
      VideoPublishCodec::H264,
      900000U,
      15.0,
      VideoPublishSimulcast::Enabled);
  }

  {
    auto options = makeStaticTokenOptions();
    appendVideoPublishOverrides(options, "vp8", 500000, 30.0, "disabled");
    options.append_parameter_override("video_configured_source_ids", std::vector<std::string>{"front"});
    options.append_parameter_override("video.configured_sources.front.source", "videotestsrc pattern=ball");
    options.append_parameter_override("video.configured_sources.front.publish.codec", "h265");

    const RuntimeConfig config = loadRuntimeConfigForNode("startup_config_configured_source_publish_override", options);

    expectPublishConfigEq(
      config.video_stream.configured_sources.at("front").publish_config,
      VideoPublishCodec::H265,
      500000U,
      30.0,
      VideoPublishSimulcast::Disabled);
  }
}

TEST_F(RuntimeConfigTest, EntryPublishOverrideCanResetFieldsToSdkDefaults)
{
  auto options = makeStaticTokenOptions();
  appendVideoPublishOverrides(options, "h264", 900000, 24.0, "enabled");
  options.append_parameter_override("video_topic_rule_ids", std::vector<std::string>{"front"});
  options.append_parameter_override("video.topic_rules.front.pattern", "/camera/front/*");
  options.append_parameter_override("video.topic_rules.front.publish.codec", "auto");
  options.append_parameter_override("video.topic_rules.front.publish.max_bitrate_bps", 0);
  options.append_parameter_override("video.topic_rules.front.publish.max_framerate", 0.0);
  options.append_parameter_override("video.topic_rules.front.publish.simulcast", "auto");

  const RuntimeConfig config = loadRuntimeConfigForNode("startup_config_publish_override_reset", options);

  ASSERT_FALSE(config.video_stream.ros_topic_rules.empty());
  expectPublishConfigEq(
    config.video_stream.ros_topic_rules.front().publish_config,
    VideoPublishCodec::Auto,
    0U,
    0.0,
    VideoPublishSimulcast::Auto);
}

TEST_F(RuntimeConfigTest, MissingGeneratedVideoParametersAreRejectedByParameterLibrary)
{
  {
    auto options = makeStaticTokenOptions();
    options.append_parameter_override("video_topic_rule_ids", std::vector<std::string>{"front"});

    expectConfigErrorContains(
      "startup_config_missing_video_entry_params",
      options,
      "parameter 'video.topic_rules.front.pattern' is not initialized");
  }

  {
    auto options = makeStaticTokenOptions();
    options.append_parameter_override("video_configured_source_ids", std::vector<std::string>{"front"});

    expectConfigErrorContains(
      "startup_config_missing_video_configured_source_params",
      options,
      "parameter 'video.configured_sources.front.source' is not initialized");
  }
}

TEST_F(RuntimeConfigTest, ConfiguredSourceRejectsWhitespaceOnlyIngressFragment)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("video_configured_source_ids", std::vector<std::string>{"front"});
  options.append_parameter_override("video.configured_sources.front.source", " \t\n ");

  expectConfigError(
    "startup_config_empty_configured_source_ingress",
    options,
    "video configured source 'front' requires a non-empty source");
}

TEST_F(RuntimeConfigTest, BridgeManagedEndpointsAreRejectedInVideoFragments)
{
  {
    auto options = makeStaticTokenOptions();
    options.append_parameter_override("video_topic_rule_ids", std::vector<std::string>{"front"});
    options.append_parameter_override("video.topic_rules.front.pattern", "/camera/front/*");
    options.append_parameter_override("video.topic_rules.front.transform", "videoconvert ! appsink");

    expectConfigError(
      "startup_config_ros_transform_appsink_rejected",
      options,
      "video topic rule 'front' transform must not define appsrc/appsink endpoints; the bridge owns them");
  }

  {
    auto options = makeStaticTokenOptions();
    options.append_parameter_override("video_configured_source_ids", std::vector<std::string>{"front"});
    options.append_parameter_override("video.configured_sources.front.source", "appsrc ! videoconvert");

    expectConfigError(
      "startup_config_configured_source_appsrc_rejected",
      options,
      "video configured source 'front' must not define appsrc/appsink endpoints; the bridge owns them");
  }
}

TEST_F(RuntimeConfigTest, InvalidVideoTransformSyntaxIsRejected)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("video_topic_rule_ids", std::vector<std::string>{"front"});
  options.append_parameter_override("video.topic_rules.front.pattern", "/camera/front/*");
  options.append_parameter_override("video.topic_rules.front.transform", "videoconvert ! )");

  expectConfigErrorContains(
    "startup_config_invalid_video_transform_syntax",
    options,
    "video topic rule 'front' transform has invalid GStreamer syntax");
}

TEST_F(RuntimeConfigTest, DuplicateVideoIdsReportSectionSpecificErrors)
{
  {
    auto options = makeStaticTokenOptions();
    options.append_parameter_override("video_topic_rule_ids", std::vector<std::string>{"front", "front"});
    options.append_parameter_override("video.topic_rules.front.pattern", "/camera/front/*");

    expectConfigError("startup_config_duplicate_video_entry_id", options, "duplicate video topic rule id 'front'");
  }

  {
    auto options = makeStaticTokenOptions();
    options.append_parameter_override("video_configured_source_ids", std::vector<std::string>{"front", "front"});
    options.append_parameter_override("video.configured_sources.front.source", "videotestsrc pattern=ball");

    expectConfigError(
      "startup_config_duplicate_video_configured_source_id", options, "duplicate video configured source id 'front'");
  }
}

TEST_F(RuntimeConfigTest, SlashVariantsLoadAsDistinctConfiguredSources)
{
  const std::string node_name = "startup_config_distinct_slash_configured_sources";
  const auto params_path =
    std::filesystem::temp_directory_path() / "livekit_ros2_bridge_distinct_slash_configured_sources_params.yaml";

  {
    std::ofstream params_file(params_path);
    ASSERT_TRUE(params_file.is_open());
    params_file << node_name << ":\n";
    params_file << "  ros__parameters:\n";
    params_file << "    livekit.url: ws://test:7880\n";
    params_file << "    livekit.token: static-token\n";
    params_file << "    video_configured_source_ids: ['/front_rtsp', '/front_rtsp/']\n";
    params_file << "    \"video.configured_sources./front_rtsp.source\": 'videotestsrc is-live=true pattern=ball'\n";
    params_file << "    \"video.configured_sources./front_rtsp/.source\": 'videotestsrc is-live=true pattern=smpte'\n";
  }

  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", params_path.string()});
  const RuntimeConfig config = loadRuntimeConfigForNode(node_name, options);

  ASSERT_EQ(config.video_stream.configured_sources.size(), 2U);
  EXPECT_EQ(
    config.video_stream.configured_sources.at("/front_rtsp").ingress_fragment,
    "videotestsrc is-live=true pattern=ball");
  EXPECT_EQ(
    config.video_stream.configured_sources.at("/front_rtsp/").ingress_fragment,
    "videotestsrc is-live=true pattern=smpte");

  std::filesystem::remove(params_path);
}

}  // namespace livekit_ros2_bridge
