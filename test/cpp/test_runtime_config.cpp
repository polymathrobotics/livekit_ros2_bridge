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
  return "livekit.token_ttl_seconds must be > 0 when livekit.api_key/livekit.api_secret mint bridge or "
         "video sidecar tokens";
}

const char * invalidApiCredentialPairError()
{
  return "livekit.api_key and livekit.api_secret must be both set or both unset";
}

const char * missingTokenConfigError()
{
  return "Either livekit.token or livekit.api_key + livekit.api_secret must be set";
}

const char * missingRosPipelineConfigError()
{
  return "video entry 'front' (ros kind) requires at least one pipeline alias from [image, compressed_image, "
         "default]";
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

void expectSupervisorConfig(
  const RuntimeConfig & startup_config,
  const std::chrono::seconds token_ttl,
  const std::chrono::seconds token_refresh_margin)
{
  ASSERT_TRUE(startup_config.video_sidecar_config.has_value());
  EXPECT_EQ(startup_config.video_sidecar_config->bridge_identity, startup_config.connect_config.identity);
  EXPECT_EQ(startup_config.video_sidecar_config->token_ttl, token_ttl);
  EXPECT_EQ(startup_config.video_sidecar_config->token_refresh_margin, token_refresh_margin);
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
  EXPECT_FALSE(startup_config.video_sidecar_config.has_value());

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

TEST_F(RuntimeConfigTest, GeneratedTokenStartupBuildsSupervisorConfig)
{
  auto options = makeBaseOptions();
  options.append_parameter_override("livekit.api_key", "api-key");
  options.append_parameter_override("livekit.api_secret", "api-secret");
  options.append_parameter_override("livekit.token_ttl_seconds", 600);
  options.append_parameter_override("livekit.token_refresh_margin_seconds", 120);
  const RuntimeConfig startup_config = loadRuntimeConfigForNode("startup_config_api_key", options);

  EXPECT_TRUE(usesDerivedIdentity("startup_config_api_key", startup_config.connect_config.identity));
  expectSupervisorConfig(startup_config, std::chrono::seconds(600), std::chrono::seconds(120));

  const auto token = startup_config.token_source->getToken(startup_config.connect_config);
  EXPECT_TRUE(token.refreshable);
  EXPECT_FALSE(token.value.empty());
}

TEST_F(RuntimeConfigTest, StaticTokenSidecarSetupBuildsSupervisorConfig)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("livekit.api_key", "api-key");
  options.append_parameter_override("livekit.api_secret", "api-secret");
  options.append_parameter_override("livekit.token_ttl_seconds", 600);
  options.append_parameter_override("livekit.token_refresh_margin_seconds", 120);
  const RuntimeConfig startup_config = loadRuntimeConfigForNode("startup_config_static_token_sidecar", options);

  expectSupervisorConfig(startup_config, std::chrono::seconds(600), std::chrono::seconds(120));

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
    // A static token does not bypass the credential-pair validation.
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

TEST_F(RuntimeConfigTest, GeneratedVideoEntriesLoadFromUnifiedParams)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("videos.ids", std::vector<std::string>{"front_camera", "front_rtsp"});
  options.append_parameter_override("videos.front_camera.kind", "ros");
  options.append_parameter_override("videos.front_camera.pattern", "/camera/front/*");
  options.append_parameter_override(
    "videos.front_camera.pipelines",
    std::vector<std::string>{
      "image=rosrawimagesrc ros-topic={topic} ros-reliable=true ! vp8enc deadline=1 target-bitrate=2000000"});
  options.append_parameter_override("videos.front_rtsp.kind", "pipeline");
  options.append_parameter_override(
    "videos.front_rtsp.pipelines",
    std::vector<std::string>{
      "default=uridecodebin uri=rtsp://127.0.0.1:8554/front source::latency=0 ! vp8enc deadline=1"});

  const RuntimeConfig startup_config = loadRuntimeConfigForNode("startup_config_video_params", options);

  ASSERT_FALSE(startup_config.video_config.ros_topic_rules.empty());
  const auto & front_rule = startup_config.video_config.ros_topic_rules.front();
  EXPECT_EQ(front_rule.pattern, "/camera/front/*");
  EXPECT_EQ(front_rule.id, "front_camera");
  ASSERT_EQ(front_rule.pipelines.size(), 1U);
  EXPECT_EQ(
    front_rule.pipelines.at("image"),
    "rosrawimagesrc ros-topic={topic} ros-reliable=true ! vp8enc deadline=1 target-bitrate=2000000");
  ASSERT_EQ(startup_config.video_config.pipeline_sources.size(), 1U);
  EXPECT_EQ(
    startup_config.video_config.pipeline_sources.at("/front_rtsp").pipeline,
    "uridecodebin uri=rtsp://127.0.0.1:8554/front source::latency=0 ! vp8enc deadline=1");
}

TEST_F(RuntimeConfigTest, RosVideoEntryAcceptsDefaultPipelineAlias)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("videos.ids", std::vector<std::string>{"front"});
  options.append_parameter_override("videos.front.kind", "ros");
  options.append_parameter_override("videos.front.pattern", "/camera/front/*");
  options.append_parameter_override(
    "videos.front.pipelines", std::vector<std::string>{"default=rosrawimagesrc ros-topic={topic} ! vp8enc deadline=1"});

  const RuntimeConfig startup_config = loadRuntimeConfigForNode("startup_config_ros_default_pipeline", options);

  ASSERT_FALSE(startup_config.video_config.ros_topic_rules.empty());
  EXPECT_EQ(startup_config.video_config.ros_topic_rules.front().id, "front");
  ASSERT_EQ(startup_config.video_config.ros_topic_rules.front().pipelines.size(), 1U);
  EXPECT_EQ(
    startup_config.video_config.ros_topic_rules.front().pipelines.at("default"),
    "rosrawimagesrc ros-topic={topic} ! vp8enc deadline=1");
}

TEST_F(RuntimeConfigTest, DuplicateVideoEntryIdReportsSectionSpecificError)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("videos.ids", std::vector<std::string>{"front", "front"});
  options.append_parameter_override("videos.front.kind", "ros");
  options.append_parameter_override("videos.front.pattern", "/camera/front/*");
  options.append_parameter_override(
    "videos.front.pipelines", std::vector<std::string>{"default=rosrawimagesrc ros-topic={topic} ! vp8enc"});

  expectRuntimeConfigError("startup_config_duplicate_video_entry_id", options, "duplicate video entry id 'front'");
}

TEST_F(RuntimeConfigTest, MissingGeneratedVideoEntryParametersAreRejectedByParameterLibrary)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("videos.ids", std::vector<std::string>{"front"});

  expectRuntimeConfigErrorContains(
    "startup_config_missing_video_entry_params", options, "parameter 'videos.front.kind' is not initialized");
}

TEST_F(RuntimeConfigTest, RosVideoEntryWithoutPipelinesIsRejected)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("videos.ids", std::vector<std::string>{"front"});
  options.append_parameter_override("videos.front.kind", "ros");
  options.append_parameter_override("videos.front.pattern", "/camera/front/*");

  expectRuntimeConfigError("startup_config_missing_ros_pipeline", options, missingRosPipelineConfigError());
}

TEST_F(RuntimeConfigTest, RosVideoEntryRejectsUnsupportedPipelineAlias)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("videos.ids", std::vector<std::string>{"front"});
  options.append_parameter_override("videos.front.kind", "ros");
  options.append_parameter_override("videos.front.pattern", "/camera/front/*");
  options.append_parameter_override(
    "videos.front.pipelines", std::vector<std::string>{"foo=rosrawimagesrc ros-topic={topic} ! vp8enc"});

  expectRuntimeConfigError(
    "startup_config_unsupported_ros_pipeline_alias",
    options,
    "video entry 'front' (ros kind) has unsupported pipeline alias 'foo'");
}

TEST_F(RuntimeConfigTest, VideoEntryRejectsDuplicatePipelineAliases)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("videos.ids", std::vector<std::string>{"front"});
  options.append_parameter_override("videos.front.kind", "ros");
  options.append_parameter_override("videos.front.pattern", "/camera/front/*");
  options.append_parameter_override(
    "videos.front.pipelines",
    std::vector<std::string>{
      "image=rosrawimagesrc ros-topic={topic} ! first-encoder",
      "image=rosrawimagesrc ros-topic={topic} ! second-encoder",
    });

  expectRuntimeConfigError(
    "startup_config_duplicate_pipeline_alias", options, "video entry 'front' has duplicate pipeline alias 'image'");
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
    params_file << "    videos.ids: ['/front_rtsp', '/front_rtsp/']\n";
    params_file << "    \"videos./front_rtsp.kind\": pipeline\n";
    params_file
      << "    \"videos./front_rtsp.pipelines\": ['default=uridecodebin uri=rtsp://127.0.0.1:8554/front ! vp8enc']\n";
    params_file << "    \"videos./front_rtsp/.kind\": pipeline\n";
    params_file << "    \"videos./front_rtsp/.pipelines\": ['default=uridecodebin uri=rtsp://127.0.0.1:8554/front-copy "
                   "! vp8enc']\n";
  }

  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", params_path.string()});
  expectRuntimeConfigError(node_name, options, "duplicate configured video external name '/front_rtsp'");

  std::filesystem::remove(params_path);
}

TEST_F(RuntimeConfigTest, PipelineVideoEntryRejectsMissingDefaultAlias)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("videos.ids", std::vector<std::string>{"front"});
  options.append_parameter_override("videos.front.kind", "pipeline");

  expectRuntimeConfigError(
    "startup_config_missing_pipeline_default",
    options,
    "video entry 'front' (pipeline kind) requires a 'default' pipeline key");
}

TEST_F(RuntimeConfigTest, UnsupportedVideoEntryKindIsRejectedByParameterLibrary)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("videos.ids", std::vector<std::string>{"front"});
  options.append_parameter_override("videos.front.kind", "gstream");
  options.append_parameter_override(
    "videos.front.pipelines", std::vector<std::string>{"default=videotestsrc pattern=ball ! vp8enc"});

  expectRuntimeConfigErrorContains(
    "startup_config_unsupported_video_kind",
    options,
    "Parameter 'videos.front.kind' with the value 'gstream' is not in the set '{ros, pipeline}'");
}

}  // namespace livekit_ros2_bridge
