^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package livekit_ros2_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (Unreleased)
------------------
* Rewrite the bridge from Python to C++ to reduce runtime overhead and make the package fully ROS/CMake native.
* Introduce protocol v2 with `ros2.*` request surfaces and `lkros.*` control-plane messages.
* Replace hard-state `subscribe` / `unsubscribe` RPCs with heartbeat-driven soft-state subscription leases and `lkros.status` updates.
* Move non-video subscription delivery to dedicated LiveKit data tracks carrying raw ROS CDR payloads instead of JSON on a shared topic.
* Add LiveKit video-track delivery for ROS image topics and configured non-ROS video sources.
* Expand client discovery and interoperability with topic/service listing and interface-definition lookup.

0.0.1 (Unreleased)
------------------
* Initial package scaffolding
