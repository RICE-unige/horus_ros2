# Future Improvements

This note tracks non-blocking follow-up work that should be completed to harden `horus_ros2` after the recent `horus_unity_bridge` transport improvements.

## Near-Term Priorities

### 1. Refine outbound message policy for high-rate state topics
- Revisit `horus_unity_bridge` subscription policy classification.
- Evaluate whether `tf2_msgs/TFMessage` and `nav_msgs/Odometry` should use replaceable delivery under backpressure.
- Keep command, task, registration, and service traffic strict.
- Add topic-policy tests for `/tf`, `/odom`, `/goal_status`, `/waypoint_status`, and collision-risk topics.

### 2. Make the package test gate fully green
- Fix `ament_cpplint` issues in `horus_unity_bridge`.
- Fix `ament_uncrustify` formatting drift.
- Fix `ament_lint_cmake` issues in `horus_unity_bridge/CMakeLists.txt`.
- Add missing copyright notices required by the ROS lint pipeline.
- Re-run:
  - `colcon build --packages-select horus_unity_bridge --cmake-args -DENABLE_WEBRTC=ON`
  - `colcon test --packages-select horus_unity_bridge`

## Medium-Term Improvements

### 3. Add realistic backpressure tests
- Add tests that simulate slow Unity clients while high-rate ROS topics are active.
- Verify that:
  - camera and point cloud streams coalesce correctly,
  - strict topics are not dropped,
  - clients do not disconnect under expected steady-state load.

### 4. Improve observability around queue pressure
- Add queue-depth and eviction counters to bridge diagnostics.
- Log when replaceable messages are evicted repeatedly for a client.
- Log when strict queues approach disconnect thresholds.

### 5. Review WebRTC and bridge transport interaction
- Validate that camera transport behavior is still correct when WebRTC is enabled and TCP topic forwarding is active.
- Confirm that vendored dependency test suppression remains scoped to external dependencies only.

## Long-Term Considerations

### 6. Make transport policy configurable
- Add a controlled configuration layer for topic-class transport behavior.
- Prefer explicit type/topic rules over broad prefix matching.
- Keep safe defaults for current HORUS workflows.

### 7. Add end-to-end MR stress validation
- Run full HORUS MR scenarios with:
  - multi-robot traffic,
  - 3D maps enabled,
  - multi-operator sessions,
  - camera streaming and teleoperation active together.
- Use those runs to validate final queue-policy defaults.

## Current Status

The current bridge changes are a net improvement and are acceptable for continued internal use and feature development.

The remaining work in this document is hardening work for production readiness, not evidence of a functional regression in the current design.
