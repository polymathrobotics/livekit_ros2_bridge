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
#include "rclcpp/rclcpp.hpp"
#include "runtime_config.hpp"

namespace livekit_ros2_bridge
{

namespace
{

class ScopedRclcppInit
{
public:
  ScopedRclcppInit()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  ~ScopedRclcppInit()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

bool usesDerivedIdentity(const std::string & node_name, const std::string & identity)
{
  return identity == node_name || identity.rfind(node_name + "-", 0) == 0;
}

const char * invalidTokenTtlError()
{
  return "livekit.token_ttl_seconds must be > 0 when livekit.api_key/livekit.api_secret mint bridge tokens";
}

const char * invalidApiCredentialPairError()
{
  return "livekit.api_key and livekit.api_secret must be both set or both unset";
}

const char * missingTokenConfigError()
{
  return "Either livekit.token or livekit.api_key + livekit.api_secret must be set";
}

rclcpp::NodeOptions makeBaseOptions()
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("livekit.url", "ws://test:7880");
  options.append_parameter_override("livekit.room", "robot-room");
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
  return loadRuntimeConfig(node->get_node_parameters_interface(), node->get_name());
}

void expectRuntimeConfigError(
  const std::string & node_name, const rclcpp::NodeOptions & options, const char * expected_error)
{
  try {
    (void)loadRuntimeConfigForNode(node_name, options);
    FAIL() << "Expected loadRuntimeConfig to throw '" << expected_error << "'";
  } catch (const std::runtime_error & error) {
    EXPECT_STREQ(error.what(), expected_error);
  }
}

void expectRuntimeConfigErrorContains(
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

}  // namespace

class RuntimeConfigTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    static ScopedRclcppInit rclcpp_init;
  }
};

TEST_F(RuntimeConfigTest, StaticTokenStartupKeepsConfiguredIdentity)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("livekit.identity", "bridge-id");
  const RuntimeConfig startup_config = loadRuntimeConfigForNode("startup_config_static_token", options);

  EXPECT_EQ(startup_config.connect_config.url, "ws://test:7880");
  EXPECT_EQ(startup_config.connect_config.room, "robot-room");
  EXPECT_EQ(startup_config.connect_config.identity, "bridge-id");

  const auto token = startup_config.token_source->getToken(startup_config.connect_config);
  EXPECT_EQ(token.value, "static-token");
  EXPECT_FALSE(token.refreshable);
}

TEST_F(RuntimeConfigTest, ApiMintedTokenModesRejectZeroTokenTtl)
{
  struct Case
  {
    const char * name;
    rclcpp::NodeOptions options;
  };

  std::vector<Case> cases;
  {
    auto options = makeBaseOptions();
    options.append_parameter_override("livekit.api_key", "api-key");
    options.append_parameter_override("livekit.api_secret", "api-secret");
    options.append_parameter_override("livekit.token_ttl_seconds", 0);
    cases.push_back({"api_key_and_secret", options});
  }
  {
    auto options = makeStaticTokenOptions();
    options.append_parameter_override("livekit.api_key", "api-key");
    options.append_parameter_override("livekit.api_secret", "api-secret");
    options.append_parameter_override("livekit.token_ttl_seconds", 0);
    cases.push_back({"static_token_with_api_credentials", options});
  }

  for (const auto & c : cases) {
    SCOPED_TRACE(c.name);
    expectRuntimeConfigError(std::string("startup_config_zero_ttl_") + c.name, c.options, invalidTokenTtlError());
  }
}

TEST_F(RuntimeConfigTest, GeneratedTokenStartupBuildsRefreshableTokenSource)
{
  auto options = makeBaseOptions();
  options.append_parameter_override("livekit.api_key", "api-key");
  options.append_parameter_override("livekit.api_secret", "api-secret");
  options.append_parameter_override("livekit.token_ttl_seconds", 600);
  options.append_parameter_override("livekit.token_refresh_margin_seconds", 120);
  const RuntimeConfig startup_config = loadRuntimeConfigForNode("startup_config_api_key", options);

  EXPECT_TRUE(usesDerivedIdentity("startup_config_api_key", startup_config.connect_config.identity));

  const auto token = startup_config.token_source->getToken(startup_config.connect_config);
  EXPECT_TRUE(token.refreshable);
  EXPECT_FALSE(token.value.empty());
}

TEST_F(RuntimeConfigTest, StaticTokenWithApiCredentialsStillUsesStaticTokenSource)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("livekit.api_key", "api-key");
  options.append_parameter_override("livekit.api_secret", "api-secret");
  options.append_parameter_override("livekit.token_ttl_seconds", 600);
  const RuntimeConfig startup_config = loadRuntimeConfigForNode("startup_config_static_token_with_api", options);

  const auto token = startup_config.token_source->getToken(startup_config.connect_config);
  EXPECT_EQ(token.value, "static-token");
  EXPECT_FALSE(token.refreshable);
}

TEST_F(RuntimeConfigTest, RejectsMalformedApiCredentialPair)
{
  struct Case
  {
    const char * name;
    const char * param;
    const char * value;
    bool with_static_token;
  };

  const Case cases[] = {
    {"api_key_without_secret", "livekit.api_key", "api-key", false},
    {"api_secret_without_key", "livekit.api_secret", "api-secret", false},
    {"api_key_without_secret_with_token", "livekit.api_key", "api-key", true},
  };

  for (const auto & c : cases) {
    SCOPED_TRACE(c.name);
    auto options = makeBaseOptions();
    if (c.with_static_token) {
      options.append_parameter_override("livekit.token", "static-token");
    }
    options.append_parameter_override(c.param, c.value);
    expectRuntimeConfigError(std::string("credential_pair_") + c.name, options, invalidApiCredentialPairError());
  }
}

TEST_F(RuntimeConfigTest, MissingTokenConfigurationThrows)
{
  expectRuntimeConfigError("startup_config_missing_token", makeBaseOptions(), missingTokenConfigError());
}

TEST_F(RuntimeConfigTest, GeneratedVideoEntriesLoadFromSplitParams)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("video_topic_rule_ids", std::vector<std::string>{"front_camera"});
  options.append_parameter_override("video_custom_source_ids", std::vector<std::string>{"front_rtsp"});
  options.append_parameter_override("video.topic_rules.front_camera.pattern", "/camera/front/*");
  options.append_parameter_override(
    "video.topic_rules.front_camera.transform", "videoconvert ! videoscale ! video/x-raw,width=640,height=360");
  options.append_parameter_override("video.custom_sources.front_rtsp.source", "videotestsrc is-live=true pattern=ball");
  options.append_parameter_override("video.custom_sources.front_rtsp.transform", "videobalance saturation=0.0");

  const RuntimeConfig startup_config = loadRuntimeConfigForNode("startup_config_video_params", options);

  ASSERT_FALSE(startup_config.video_config.ros_topic_rules.empty());
  const auto & front_rule = startup_config.video_config.ros_topic_rules.front();
  EXPECT_EQ(front_rule.pattern, "/camera/front/*");
  EXPECT_EQ(front_rule.id, "front_camera");
  EXPECT_EQ(front_rule.transform, "videoconvert ! videoscale ! video/x-raw,width=640,height=360");
  ASSERT_EQ(startup_config.video_config.external_sources.size(), 1U);
  EXPECT_EQ(
    startup_config.video_config.external_sources.at("/front_rtsp").source, "videotestsrc is-live=true pattern=ball");
  EXPECT_EQ(startup_config.video_config.external_sources.at("/front_rtsp").transform, "videobalance saturation=0.0");
}

TEST_F(RuntimeConfigTest, VideoPublishConfigLoadsFromUnifiedParams)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("video.publish.codec", "h264");
  options.append_parameter_override("video.publish.max_bitrate_bps", 900000);
  options.append_parameter_override("video.publish.max_framerate", 24.0);
  options.append_parameter_override("video.publish.simulcast", "enabled");

  const RuntimeConfig startup_config = loadRuntimeConfigForNode("startup_config_video_publish_params", options);

  EXPECT_EQ(startup_config.video_config.publish.codec, VideoPublishCodec::H264);
  EXPECT_EQ(startup_config.video_config.publish.max_bitrate_bps, 900000U);
  EXPECT_DOUBLE_EQ(startup_config.video_config.publish.max_framerate, 24.0);
  EXPECT_EQ(startup_config.video_config.publish.simulcast, VideoPublishSimulcast::Enabled);
}

TEST_F(RuntimeConfigTest, GeneratedSubscriptionQosOverridesLoadFromUnifiedParams)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("subscription_qos_override_ids", std::vector<std::string>{"camera", "front"});
  options.append_parameter_override("subscribe.qos_overrides.camera.pattern", "/camera/*");
  options.append_parameter_override("subscribe.qos_overrides.camera.reliability", "best_effort");
  options.append_parameter_override("subscribe.qos_overrides.camera.durability", "auto");
  options.append_parameter_override("subscribe.qos_overrides.front.pattern", " //camera/front/ ");
  options.append_parameter_override("subscribe.qos_overrides.front.reliability", "auto");
  options.append_parameter_override("subscribe.qos_overrides.front.durability", "transient_local");

  const RuntimeConfig startup_config = loadRuntimeConfigForNode("startup_config_subscription_qos_params", options);

  ASSERT_EQ(startup_config.subscription_qos_config.topic_overrides.size(), 2U);
  EXPECT_EQ(startup_config.subscription_qos_config.topic_overrides[0].id, "camera");
  EXPECT_EQ(startup_config.subscription_qos_config.topic_overrides[0].pattern, "/camera/*");
  EXPECT_EQ(
    startup_config.subscription_qos_config.topic_overrides[0].reliability, SubscriptionQosReliabilityMode::kBestEffort);
  EXPECT_EQ(startup_config.subscription_qos_config.topic_overrides[0].durability, SubscriptionQosDurabilityMode::kAuto);
  EXPECT_EQ(startup_config.subscription_qos_config.topic_overrides[1].id, "front");
  EXPECT_EQ(startup_config.subscription_qos_config.topic_overrides[1].pattern, "/camera/front");
  EXPECT_EQ(
    startup_config.subscription_qos_config.topic_overrides[1].reliability, SubscriptionQosReliabilityMode::kAuto);
  EXPECT_EQ(
    startup_config.subscription_qos_config.topic_overrides[1].durability,
    SubscriptionQosDurabilityMode::kTransientLocal);
}

TEST_F(RuntimeConfigTest, DuplicateSubscriptionQosOverrideIdReportsSectionSpecificError)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("subscription_qos_override_ids", std::vector<std::string>{"camera", "camera"});
  options.append_parameter_override("subscribe.qos_overrides.camera.pattern", "/camera/*");
  options.append_parameter_override("subscribe.qos_overrides.camera.reliability", "auto");
  options.append_parameter_override("subscribe.qos_overrides.camera.durability", "auto");

  expectRuntimeConfigError(
    "startup_config_duplicate_subscription_qos_override_id",
    options,
    "duplicate subscription QoS override id 'camera'");
}

TEST_F(RuntimeConfigTest, SubscriptionQosOverrideRejectsEmptyPattern)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("subscription_qos_override_ids", std::vector<std::string>{"camera"});
  options.append_parameter_override("subscribe.qos_overrides.camera.pattern", "   ");
  options.append_parameter_override("subscribe.qos_overrides.camera.reliability", "auto");
  options.append_parameter_override("subscribe.qos_overrides.camera.durability", "auto");

  expectRuntimeConfigError(
    "startup_config_empty_subscription_qos_pattern", options, "subscribe.qos_overrides pattern must not be empty");
}

TEST_F(RuntimeConfigTest, UnsupportedSubscriptionQosReliabilityIsRejectedByParameterLibrary)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("subscription_qos_override_ids", std::vector<std::string>{"camera"});
  options.append_parameter_override("subscribe.qos_overrides.camera.pattern", "/camera/*");
  options.append_parameter_override("subscribe.qos_overrides.camera.reliability", "sometimes");
  options.append_parameter_override("subscribe.qos_overrides.camera.durability", "auto");

  expectRuntimeConfigErrorContains(
    "startup_config_invalid_subscription_qos_reliability",
    options,
    "Parameter 'subscribe.qos_overrides.camera.reliability' with the value 'sometimes' is not in the set");
}

TEST_F(RuntimeConfigTest, RosVideoEntryWithoutTransformUsesEmptyTransform)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("video_topic_rule_ids", std::vector<std::string>{"front"});
  options.append_parameter_override("video.topic_rules.front.pattern", "/camera/front/*");

  const RuntimeConfig startup_config = loadRuntimeConfigForNode("startup_config_ros_empty_transform", options);

  ASSERT_FALSE(startup_config.video_config.ros_topic_rules.empty());
  EXPECT_EQ(startup_config.video_config.ros_topic_rules.front().id, "front");
  EXPECT_EQ(startup_config.video_config.ros_topic_rules.front().transform, "");
  expectPublishConfigEq(
    startup_config.video_config.ros_topic_rules.front().publish,
    VideoPublishCodec::Auto,
    0U,
    0.0,
    VideoPublishSimulcast::Auto);
}

TEST_F(RuntimeConfigTest, RosVideoPublishOverrideCanSetSingleFieldWithoutTransform)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("video.publish.codec", "h264");
  options.append_parameter_override("video.publish.max_bitrate_bps", 900000);
  options.append_parameter_override("video.publish.max_framerate", 24.0);
  options.append_parameter_override("video.publish.simulcast", "enabled");
  options.append_parameter_override("video_topic_rule_ids", std::vector<std::string>{"front"});
  options.append_parameter_override("video.topic_rules.front.pattern", "/camera/front/*");
  options.append_parameter_override("video.topic_rules.front.publish.max_framerate", 15.0);

  const RuntimeConfig startup_config = loadRuntimeConfigForNode("startup_config_ros_publish_override", options);

  ASSERT_FALSE(startup_config.video_config.ros_topic_rules.empty());
  const auto & rule = startup_config.video_config.ros_topic_rules.front();
  EXPECT_EQ(rule.id, "front");
  EXPECT_EQ(rule.transform, "");
  expectPublishConfigEq(rule.publish, VideoPublishCodec::H264, 900000U, 15.0, VideoPublishSimulcast::Enabled);
}

TEST_F(RuntimeConfigTest, CustomVideoPublishOverrideCanSetSingleFieldWithoutTransform)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("video.publish.codec", "vp8");
  options.append_parameter_override("video.publish.max_bitrate_bps", 500000);
  options.append_parameter_override("video.publish.max_framerate", 30.0);
  options.append_parameter_override("video.publish.simulcast", "disabled");
  options.append_parameter_override("video_custom_source_ids", std::vector<std::string>{"front"});
  options.append_parameter_override("video.custom_sources.front.source", "videotestsrc pattern=ball");
  options.append_parameter_override("video.custom_sources.front.publish.codec", "h265");

  const RuntimeConfig startup_config = loadRuntimeConfigForNode("startup_config_custom_publish_override", options);

  ASSERT_EQ(startup_config.video_config.external_sources.size(), 1U);
  const auto & source = startup_config.video_config.external_sources.at("/front");
  EXPECT_EQ(source.transform, "");
  expectPublishConfigEq(source.publish, VideoPublishCodec::H265, 500000U, 30.0, VideoPublishSimulcast::Disabled);
}

TEST_F(RuntimeConfigTest, EntryPublishOverrideCanResetFieldsToSdkDefaults)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("video.publish.codec", "h264");
  options.append_parameter_override("video.publish.max_bitrate_bps", 900000);
  options.append_parameter_override("video.publish.max_framerate", 24.0);
  options.append_parameter_override("video.publish.simulcast", "enabled");
  options.append_parameter_override("video_topic_rule_ids", std::vector<std::string>{"front"});
  options.append_parameter_override("video.topic_rules.front.pattern", "/camera/front/*");
  options.append_parameter_override("video.topic_rules.front.publish.codec", "auto");
  options.append_parameter_override("video.topic_rules.front.publish.max_bitrate_bps", 0);
  options.append_parameter_override("video.topic_rules.front.publish.max_framerate", 0.0);
  options.append_parameter_override("video.topic_rules.front.publish.simulcast", "auto");

  const RuntimeConfig startup_config = loadRuntimeConfigForNode("startup_config_publish_override_reset", options);

  ASSERT_FALSE(startup_config.video_config.ros_topic_rules.empty());
  expectPublishConfigEq(
    startup_config.video_config.ros_topic_rules.front().publish,
    VideoPublishCodec::Auto,
    0U,
    0.0,
    VideoPublishSimulcast::Auto);
}

TEST_F(RuntimeConfigTest, DuplicateVideoEntryIdReportsSectionSpecificError)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("video_topic_rule_ids", std::vector<std::string>{"front", "front"});
  options.append_parameter_override("video.topic_rules.front.pattern", "/camera/front/*");

  expectRuntimeConfigError("startup_config_duplicate_video_entry_id", options, "duplicate video topic rule id 'front'");
}

TEST_F(RuntimeConfigTest, MissingGeneratedVideoEntryParametersAreRejectedByParameterLibrary)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("video_topic_rule_ids", std::vector<std::string>{"front"});

  expectRuntimeConfigErrorContains(
    "startup_config_missing_video_entry_params",
    options,
    "parameter 'video.topic_rules.front.pattern' is not initialized");
}

TEST_F(RuntimeConfigTest, MissingGeneratedVideoCustomSourceParametersAreRejectedByParameterLibrary)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("video_custom_source_ids", std::vector<std::string>{"front"});

  expectRuntimeConfigErrorContains(
    "startup_config_missing_video_custom_source_params",
    options,
    "parameter 'video.custom_sources.front.source' is not initialized");
}

TEST_F(RuntimeConfigTest, RosVideoTransformRejectsBridgeManagedAppsink)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("video_topic_rule_ids", std::vector<std::string>{"front"});
  options.append_parameter_override("video.topic_rules.front.pattern", "/camera/front/*");
  options.append_parameter_override("video.topic_rules.front.transform", "videoconvert ! appsink");

  expectRuntimeConfigError(
    "startup_config_ros_transform_appsink_rejected",
    options,
    "video topic rule 'front' transform must not define appsrc/appsink endpoints; the bridge owns them");
}

TEST_F(RuntimeConfigTest, ExternalVideoSourceRejectsBridgeManagedAppsrc)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("video_custom_source_ids", std::vector<std::string>{"front"});
  options.append_parameter_override("video.custom_sources.front.source", "appsrc ! videoconvert");

  expectRuntimeConfigError(
    "startup_config_external_source_appsrc_rejected",
    options,
    "video custom source 'front' must not define appsrc/appsink endpoints; the bridge owns them");
}

TEST_F(RuntimeConfigTest, DuplicateVideoCustomSourceIdReportsSectionSpecificError)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("video_custom_source_ids", std::vector<std::string>{"front", "front"});
  options.append_parameter_override("video.custom_sources.front.source", "videotestsrc pattern=ball");

  expectRuntimeConfigError(
    "startup_config_duplicate_video_custom_source_id", options, "duplicate video custom source id 'front'");
}

TEST_F(RuntimeConfigTest, DuplicateConfiguredSourceIdReportsSectionSpecificError)
{
  const std::string node_name = "startup_config_duplicate_pipeline_source";
  const auto params_path =
    std::filesystem::temp_directory_path() / "livekit_ros2_bridge_duplicate_pipeline_source_params.yaml";

  {
    std::ofstream params_file(params_path);
    ASSERT_TRUE(params_file.is_open());
    params_file << node_name << ":\n";
    params_file << "  ros__parameters:\n";
    params_file << "    livekit.url: ws://test:7880\n";
    params_file << "    livekit.room: robot-room\n";
    params_file << "    livekit.token: static-token\n";
    params_file << "    video_custom_source_ids: ['/front_rtsp', '/front_rtsp/']\n";
    params_file << "    \"video.custom_sources./front_rtsp.source\": 'videotestsrc is-live=true pattern=ball'\n";
    params_file << "    \"video.custom_sources./front_rtsp/.source\": 'videotestsrc is-live=true pattern=smpte'\n";
  }

  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", params_path.string()});
  expectRuntimeConfigError(node_name, options, "duplicate configured video external name '/front_rtsp'");

  std::filesystem::remove(params_path);
}

}  // namespace livekit_ros2_bridge
