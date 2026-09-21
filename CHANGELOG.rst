^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package livekit_ros2_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.0 (Unreleased)
------------------
* Add configured non-ROS audio sources as mono LiveKit tracks.
* Rename non-ROS source concepts from "other" to "external": subscription kinds `other_video` /
  `other_audio` become `external_video` / `external_audio`, stream keys and `lkros.*` track-name
  prefixes change to match, and config params `video_other_ids` / `audio_other_ids` plus the
  `video.other.*` / `audio.other.*` maps become `video_external_ids` / `audio_external_ids` and
  `video.external.*` / `audio.external.*` with no aliases (breaking). Serialized output always uses
  the new names, and the heartbeat kind parser accepts the old `other_*` kinds as deprecated
  aliases for one release so deployed clients keep working.

0.2.0 (Unreleased)
------------------
* Bump the LiveKit C++ SDK to 1.6.0, which adds a libcurl runtime dependency. Publish options
  now use the new `frame_metadata_features` field in place of `packet_trailer_features`.

0.1.1 (Unreleased)
------------------
* Move watchdog timeout enforcement off the ROS executor and fix stale recovery-deadline handling.
* Improve logging around LiveKit room and track lifecycle events.
* Stop republishing already published data tracks on participant refresh/reconnect.
* Log package version and protocol version at node startup.

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
