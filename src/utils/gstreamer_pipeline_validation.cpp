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

#include "utils/gstreamer_pipeline_validation.hpp"

#include <stdexcept>
#include <string>
#include <string_view>

#include "utils/gstreamer_resources.hpp"

namespace livekit_ros2_bridge::utils
{

namespace
{

EndpointCounts countEndpoints(const std::string & context, GstElement * pipeline, const EndpointLayout & layout)
{
  EndpointCounts counts;

  GstIteratorPtr iterator(gst_bin_iterate_recurse(GST_BIN(pipeline)));
  GValueSlot item;
  while (true) {
    const GstIteratorResult result = gst_iterator_next(iterator.get(), item.get());
    if (result == GST_ITERATOR_DONE) {
      break;
    }
    if (result == GST_ITERATOR_RESYNC) {
      gst_iterator_resync(iterator.get());
      continue;
    }
    if (result != GST_ITERATOR_OK) {
      throw std::runtime_error(context + " could not inspect parsed GStreamer elements");
    }

    auto * element = GST_ELEMENT(g_value_get_object(item.get()));
    const GstElementFactory * factory = gst_element_get_factory(element);
    const std::string_view factory_name = factory == nullptr ? "" : GST_OBJECT_NAME(factory);
    const std::string_view element_name = GST_ELEMENT_NAME(element);

    const bool is_appsrc = factory_name == "appsrc";
    const bool is_appsink = factory_name == "appsink";
    if (is_appsrc) {
      ++counts.appsrc;
    }
    if (is_appsink) {
      ++counts.appsink;
    }

    if (layout.bridge_appsrc_name != nullptr && element_name == layout.bridge_appsrc_name) {
      if (!is_appsrc) {
        throw std::runtime_error(context + " must not reuse reserved element name '" + layout.bridge_appsrc_name + "'");
      }
      ++counts.bridge_appsrc;
    }
    if (layout.bridge_appsink_name != nullptr && element_name == layout.bridge_appsink_name) {
      if (!is_appsink) {
        throw std::runtime_error(
          context + " must not reuse reserved element name '" + layout.bridge_appsink_name + "'");
      }
      ++counts.bridge_appsink;
    }

    item.reset();
  }

  return counts;
}

}  // namespace

bool EndpointLayout::matches(const EndpointCounts & counts) const noexcept
{
  return counts.appsrc == appsrc && counts.appsink == appsink && counts.bridge_appsrc == bridge_appsrc &&
         counts.bridge_appsink == bridge_appsink;
}

bool EndpointLayout::hasUserEndpoint(const EndpointCounts & counts) const noexcept
{
  if (
    counts.appsrc > appsrc || counts.appsink > appsink || counts.bridge_appsrc > bridge_appsrc ||
    counts.bridge_appsink > bridge_appsink)
  {
    return true;
  }

  return (counts.appsrc != 0U && bridge_appsrc == 0U) || (counts.appsink != 0U && bridge_appsink == 0U);
}

void validatePipeline(const std::string & context, const std::string & description, const EndpointLayout & layout)
{
  ensureGStreamerInitialized();

  GError * raw_error = nullptr;
  GstElementPtr pipeline(gst_parse_launch(description.c_str(), &raw_error));
  GErrorPtr error(raw_error);
  // Prefer the endpoint-ownership error when a partial parse already shows it.
  if (error != nullptr && pipeline != nullptr) {
    const EndpointCounts counts = countEndpoints(context, pipeline.get(), layout);
    if (layout.hasUserEndpoint(counts)) {
      throw std::runtime_error(context + " must not define appsrc/appsink endpoints; the bridge owns them");
    }
  }
  if (error != nullptr) {
    throw std::runtime_error(context + " has invalid GStreamer syntax: " + error->message);
  }
  if (pipeline == nullptr) {
    throw std::runtime_error(context + " has invalid GStreamer syntax: gst_parse_launch returned null");
  }

  if (!GST_IS_BIN(pipeline.get())) {
    throw std::runtime_error(context + " must parse to a GstBin");
  }

  const EndpointCounts counts = countEndpoints(context, pipeline.get(), layout);
  if (!layout.matches(counts)) {
    throw std::runtime_error(context + " must not define appsrc/appsink endpoints; the bridge owns them");
  }
}

}  // namespace livekit_ros2_bridge::utils
