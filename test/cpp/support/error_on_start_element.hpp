// Copyright 2025 Polymath Robotics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <gst/gst.h>

#include <atomic>
#include <cstddef>
#include <stdexcept>

#include "utils/gstreamer_resources.hpp"

namespace livekit_ros2_bridge::test_support
{

// A test-only element, "lkroserroronstart": an identity that posts an error each
// time it goes to PAUSED, on the thread changing the pipeline's state, yet lets
// that state change succeed. It stands in for a source whose failure arrives
// while a stream's start() runs without making it throw, like rtspsrc reporting
// a refused connection from its own thread.
class ErrorOnStartElement final
{
public:
  static constexpr char kFactoryName[] = "lkroserroronstart";

  // Registers the element factory once per process.
  static void registerFactory()
  {
    static const bool registered = []() {
      utils::ensureGStreamerInitialized();
      const utils::GstObjectPtr<GstElementFactory> identity_factory(gst_element_factory_find("identity"));
      if (identity_factory == nullptr) {
        return false;
      }
      const utils::GstObjectPtr<GstPluginFeature> loaded_feature(
        gst_plugin_feature_load(GST_PLUGIN_FEATURE(identity_factory.get())));
      if (loaded_feature == nullptr) {
        return false;
      }

      const GType parent_type = gst_element_factory_get_element_type(GST_ELEMENT_FACTORY(loaded_feature.get()));
      GTypeQuery parent_query;
      g_type_query(parent_type, &parent_query);
      const GType type = g_type_register_static_simple(
        parent_type,
        "LkrosErrorOnStartElement",
        parent_query.class_size,
        &ErrorOnStartElement::initClass,
        parent_query.instance_size,
        nullptr,
        static_cast<GTypeFlags>(0));
      return gst_element_register(nullptr, kFactoryName, GST_RANK_NONE, type) == TRUE;
    }();
    if (!registered) {
      throw std::runtime_error("Failed to register the lkroserroronstart test element.");
    }
  }

  // How often any instance has gone to PAUSED, and so posted its error.
  static std::size_t startCount()
  {
    return startCounter().load(std::memory_order_acquire);
  }

private:
  using ChangeState = GstStateChangeReturn (*)(GstElement * element, GstStateChange transition);

  static void initClass(gpointer element_class, gpointer)
  {
    // The class struct starts as a copy of identity's, so this is its change_state.
    parentChangeState() = GST_ELEMENT_CLASS(element_class)->change_state;
    GST_ELEMENT_CLASS(element_class)->change_state = &ErrorOnStartElement::changeState;
  }

  static GstStateChangeReturn changeState(GstElement * element, GstStateChange transition)
  {
    const GstStateChangeReturn result = parentChangeState()(element, transition);
    if (transition != GST_STATE_CHANGE_READY_TO_PAUSED || result == GST_STATE_CHANGE_FAILURE) {
      return result;
    }

    startCounter().fetch_add(1U, std::memory_order_acq_rel);
    const utils::GErrorPtr error(
      g_error_new_literal(GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_FAILED, "simulated failure on start"));
    gst_element_post_message(element, gst_message_new_error(GST_OBJECT(element), error.get(), nullptr));
    return result;
  }

  static ChangeState & parentChangeState()
  {
    static ChangeState instance = nullptr;
    return instance;
  }

  static std::atomic<std::size_t> & startCounter()
  {
    static std::atomic<std::size_t> instance{0U};
    return instance;
  }
};

}  // namespace livekit_ros2_bridge::test_support
