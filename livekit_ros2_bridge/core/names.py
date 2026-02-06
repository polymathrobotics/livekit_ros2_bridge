# Copyright (c) 2025-present Polymath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from __future__ import annotations

import re


def normalize_ros_topic(ros_topic: str | None) -> str:
    topic = (ros_topic or "").strip()
    if not topic:
        return ""
    topic = re.sub(r"/+", "/", topic)
    if not topic.startswith("/"):
        topic = f"/{topic}"
    if topic.endswith("/") and topic != "/":
        topic = topic.rstrip("/")
    return topic


__all__ = ["normalize_ros_topic"]
