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

// Test-only "lkroscontrolledstart": an identity whose start (READY->PAUSED) a test steers
// through a process-wide mode, to make a stream's start() throw or to report a failure
// while it runs. Tests that use it set the mode first.
class ControlledStartElement final
{
public:
  enum class Mode
  {
    // Starts like identity.
    Normal,
    // Posts an error but lets the start succeed, like rtspsrc reporting a refused connection
    // from its own thread while start() runs.
    PostErrorOnStart,
    // Fails the start without posting an error, so start() throws.
    FailStart,
  };

  static constexpr char kFactoryName[] = "lkroscontrolledstart";

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
        "LkrosControlledStartElement",
        parent_query.class_size,
        &ControlledStartElement::initClass,
        parent_query.instance_size,
        nullptr,
        static_cast<GTypeFlags>(0));
      return gst_element_register(nullptr, kFactoryName, GST_RANK_NONE, type) == TRUE;
    }();
    if (!registered) {
      throw std::runtime_error("Failed to register the lkroscontrolledstart test element.");
    }
  }

  // Applies from the next start of any instance on.
  static void setMode(Mode mode)
  {
    modeSlot().store(mode, std::memory_order_release);
  }

  // How many instances have tried to start, whatever the mode; one per pipeline start.
  static std::size_t startAttempts()
  {
    return startAttemptCounter().load(std::memory_order_acquire);
  }

private:
  using ChangeState = GstStateChangeReturn (*)(GstElement * element, GstStateChange transition);

  static void initClass(gpointer element_class, gpointer)
  {
    // The class struct starts as a copy of identity's, so this is its change_state.
    parentChangeState() = GST_ELEMENT_CLASS(element_class)->change_state;
    GST_ELEMENT_CLASS(element_class)->change_state = &ControlledStartElement::changeState;
  }

  static GstStateChangeReturn changeState(GstElement * element, GstStateChange transition)
  {
    if (transition != GST_STATE_CHANGE_READY_TO_PAUSED) {
      return parentChangeState()(element, transition);
    }

    const Mode mode = startMode(element);
    if (mode == Mode::FailStart) {
      return GST_STATE_CHANGE_FAILURE;
    }

    const GstStateChangeReturn result = parentChangeState()(element, transition);
    if (mode != Mode::PostErrorOnStart || result == GST_STATE_CHANGE_FAILURE) {
      return result;
    }

    const utils::GErrorPtr error(
      g_error_new_literal(GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_FAILED, "simulated failure on start"));
    gst_element_post_message(element, gst_message_new_error(GST_OBJECT(element), error.get(), nullptr));
    return result;
  }

  // GstBin can retry a child's failed state change within one set_state, so each instance
  // counts one attempt and keeps the mode it saw first. The mode is read before counting, so
  // a test that switches modes once it sees an attempt affects only later ones.
  static Mode startMode(GstElement * element)
  {
    static const GQuark mode_quark = g_quark_from_static_string("lkros-controlled-start-mode");
    const gpointer stored_mode = g_object_get_qdata(G_OBJECT(element), mode_quark);
    if (stored_mode != nullptr) {
      return static_cast<Mode>(GPOINTER_TO_INT(stored_mode) - 1);
    }

    const Mode mode = modeSlot().load(std::memory_order_acquire);
    g_object_set_qdata(G_OBJECT(element), mode_quark, GINT_TO_POINTER(static_cast<int>(mode) + 1));
    startAttemptCounter().fetch_add(1U, std::memory_order_acq_rel);
    return mode;
  }

  static ChangeState & parentChangeState()
  {
    static ChangeState instance = nullptr;
    return instance;
  }

  static std::atomic<Mode> & modeSlot()
  {
    static std::atomic<Mode> instance{Mode::Normal};
    return instance;
  }

  static std::atomic<std::size_t> & startAttemptCounter()
  {
    static std::atomic<std::size_t> instance{0U};
    return instance;
  }
};

}  // namespace livekit_ros2_bridge::test_support
