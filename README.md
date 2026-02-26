# HORUS ROS 2 Infrastructure

[![ROS2](https://img.shields.io/badge/ROS2-Humble%20%7C%20Jazzy-22314E)](https://docs.ros.org)
![Build](https://img.shields.io/badge/Build-colcon-blue)
![C++](https://img.shields.io/badge/C%2B%2B-17-00599C)
[![License](https://img.shields.io/badge/License-Apache--2.0-green.svg)](LICENSE)

<p align="center">
  <img src="docs/horus_logo_black.svg#gh-light-mode-only" alt="HORUS logo" height="90">
  <img src="docs/horus_logo_white.svg#gh-dark-mode-only" alt="HORUS logo" height="90">
</p>

<p align="center"><em>Holistic Operational Reality for Unified Systems</em></p>

> [!TIP]
> ROS 2 infrastructure layer for the HORUS research stack (SDK <-> bridge <-> MR app).

> [!IMPORTANT]
> This repository provides the ROS 2 backend infrastructure for HORUS, including the Unity bridge runtime (`horus_unity_bridge`) used by the MR application.

## :microscope: Research Scope

This repo focuses on runtime infrastructure for mixed-reality robot operations:
- ROS 2 topic/service integration,
- Unity-compatible TCP bridge behavior,
- optional WebRTC camera signaling/media pipeline,
- multi-client Unity bridge fanout/cleanup for multi-operator sessions,
- bridge-authoritative per-robot control lease arbitration and command-topic enforcement,
- backend package boundaries for scalability experiments.

It does not own SDK payload authoring (`horus_sdk`) or Unity scene/runtime UX (`horus`).

## :jigsaw: Packages and Responsibilities

| Package | Responsibility |
|---|---|
| `horus_unity_bridge` | TCP/WebRTC bridge runtime between ROS 2 and Unity client, including multi-client fanout and control-lease enforcement |
| `horus_backend` | Backend registration/state management logic |
| `horus_interfaces` | ROS 2 interfaces used by HORUS components |
| `horus_unity_bridge_test` | Test utilities and simulation helpers |

## :building_construction: Architecture

```text
HORUS SDK (registration + orchestration)
         <->
horus_unity_bridge (ROS 2 node, TCP/WebRTC)
         <->
HORUS MR App (Unity client)
```

Default ingress:
- TCP server: `0.0.0.0:10000`

WebRTC signaling topics:
- `/horus/webrtc/client_signal`
- `/horus/webrtc/server_signal`

Recommended go-to-point task topics:
- `/<robot>/goal_pose` (`geometry_msgs/PoseStamped`)
- `/<robot>/goal_cancel` (`std_msgs/String`, payload: `"cancel"`)
- `/<robot>/goal_status` (`std_msgs/String`)

Recommended waypoint task topics:
- `/<robot>/waypoint_path` (`nav_msgs/Path`)
- `/<robot>/waypoint_status` (`std_msgs/String`, JSON status payload)

Robot-task backend baseline:
- `horus_backend` includes an optional Nav2 action adapter path for `goal_pose` / `goal_cancel` / `goal_status`.
- Adapter auto-enables when both `nav2_msgs` and `rclcpp_action` are available at build time.
- When Nav2 dependencies are missing, backend still builds and runs with adapter disabled.

## :white_check_mark: Prerequisites

- ROS 2 Humble or Jazzy
- `rosdep`
- C++ build toolchain (`colcon`, CMake)
- For WebRTC builds: GStreamer + OpenCV dependencies (see [`horus_unity_bridge/README.md`](horus_unity_bridge/README.md))

## :gear: Build Matrix

| Mode | Command |
|---|---|
| Standard bridge | `colcon build --packages-select horus_unity_bridge` |
| WebRTC-enabled bridge | `colcon build --packages-select horus_unity_bridge --cmake-args -DENABLE_WEBRTC=ON` |
| Backend + bridge (auto Nav2 detect) | `colcon build --packages-select horus_backend horus_unity_bridge --cmake-args -DENABLE_WEBRTC=ON` |

Recommended setup:

```bash
git clone https://github.com/RICE-unige/horus_ros2.git ~/horus_ws/src/horus_ros2
cd ~/horus_ws/src/horus_ros2
rosdep install --from-paths . -y --ignore-src
colcon build --symlink-install --packages-select horus_unity_bridge --cmake-args -DENABLE_WEBRTC=ON
source install/setup.bash
```

## :rocket: Launch

```bash
ros2 launch horus_unity_bridge unity_bridge.launch.py
```

> [!NOTE]
> Runtime defaults are managed in `horus_unity_bridge/config/bridge_config.yaml`.

## :toolbox: Operational Guidance

### Signaling and QoS
- WebRTC signaling uses reliable/volatile semantics for timely negotiation.
- Bridge emits structured diagnostics for negotiation and media startup.
- Bridge recovery policy now uses paced keyframe requests (warmup + cooldown gates) to avoid recovery flapping under load.

### Diagnostics to Monitor
- connection lifecycle events,
- negotiated MID/H264 payload selection,
- first outbound RTP header fields,
- periodic RTP counters,
- per-session stall telemetry (incoming frames, pushed frames, dropped queue frames, last push progress),
- keyframe request reasons and cooldown behavior,
- multi-operator bridge lease events (`control_lease_state`) and denied-command diagnostics when control topics are protected.

### Integration Boundary
- `horus_sdk` should treat this repo as infrastructure service, not payload author.
- `horus` should consume this bridge as transport/runtime endpoint.

### Multi-Operator Bridge Baseline
Current multi-operator infrastructure now integrated in `main` includes:
- multi-client subscriber fanout (prevents last-subscriber-wins behavior),
- client disconnect cleanup for subscriptions/publishers/services/session state,
- per-robot control lease arbitration (`/horus/multi_operator/control_lease_request`),
- protected command-topic enforcement with lease-denied events,
- multi-client publisher ownership refcounting (prevents one client disconnect from tearing down shared publishers).

Lease state snapshots are published on `/horus/multi_operator/control_lease_state` for MR clients (and optional SDK observability).

## :test_tube: Validation Workflow

```bash
# 1) Start bridge
ros2 launch horus_unity_bridge unity_bridge.launch.py

# 2) In SDK repo, run unified fake runtime + typical registration demo
python3 python/examples/fake_tf_ops_suite.py --robot-count 10 --rate 30 --static-camera --publish-compressed-images
python3 python/examples/sdk_typical_ops_demo.py --robot-count 10 --workspace-scale 0.1
```

Expected outcomes:
- bridge accepts Unity client connection,
- registration/ack/heartbeat channels remain stable,
- WebRTC sessions negotiate and stream when requested,
- robots remain static until teleop/task command topics are used,
- active teleop commands can preempt active go-to-point/waypoint execution.

## :warning: Known Constraints

- Performance envelope depends on robot count, camera resolutions, and bridge host decode/encode budget.
- Network topology (WSL/NAT/localhost) can affect ICE behavior in WebRTC mode.
- For occupancy map validation, use transient-local publishers (the SDK fake publisher already does this by default).

## :world_map: Roadmap

> [!NOTE]
> This roadmap is scoped to ROS 2 infrastructure ownership (bridge/runtime reliability, observability, and reproducibility), not Unity UX or SDK dashboard behavior.

| Stream | Status | Current Baseline | Next Milestone |
|---|---|---|---|
| Bridge Runtime Stability | :white_check_mark: Active baseline | Stable TCP bridge path, WebRTC signaling/session flow, multi-client subscriber fanout, and publisher ownership refcounting are integrated in `main`, with negotiation diagnostics available (MID/PT/RTP counters). | Add long-duration soak tests and explicit reconnection stress scenarios. |
| WebRTC Media Reliability | :large_orange_diamond: In progress | Negotiation mismatch fixes plus stall telemetry and paced keyframe recovery are integrated. | Harden edge-case handling for network topology variance and high robot counts. |
| QoS and Session Policy | :large_orange_diamond: In progress | Reliable/volatile signaling defaults in place with current compatibility behavior. | Add topic/session policy profiles for workload-specific tuning. |
| Robot Task Routing | :large_orange_diamond: In progress | Go-to-point/waypoint topic guidance is defined and optional Nav2 adapter flow is integrated with build-time dependency gating. | Add multi-robot task execution stress validation and richer task-status observability. |
| Multi-Operator Control Arbitration | :large_orange_diamond: In progress | Bridge-side per-robot control lease arbiter, protected command-topic enforcement, and lease state snapshots are integrated. | Harden lease telemetry/observability, tune TTL policies, and extend regression coverage for repeated join/rejoin contention scenarios. |
| Integration Automation | :white_circle: Planned | Validation is currently mostly manual (SDK fake publishers + Unity runtime checks). | Add CI-driven integration matrix across SDK, bridge, and Unity harness scenarios. |
| Observability and Metrics | :white_circle: Planned | Logs provide actionable diagnostics during runtime issues. | Export machine-readable metrics for dashboards and regression tracking. |
| Benchmarking and Reproducibility | :white_circle: Planned | Ad-hoc profiling exists across development cycles. | Publish repeatable throughput/latency benchmark suite for research reporting. |

## :link: Related Repositories

- SDK: <https://github.com/RICE-unige/horus_sdk>
- Unity MR app: <https://github.com/RICE-unige/horus>

## :book: Citation

If you use HORUS or ideas from this work in your research, please cite:

O. S. Adekoya, A. Sgorbissa, C. T. Recchiuto. HORUS: A Mixed Reality Interface for Managing Teams of Mobile Robots. arXiv preprint arXiv:2506.02622, 2025.

```bibtex
@misc{adekoya2025horus,
  title         = {HORUS: A Mixed Reality Interface for Managing Teams of Mobile Robots},
  author        = {Adekoya, Omotoye Shamsudeen and Sgorbissa, Antonio and Recchiuto, Carmine Tommaso},
  year          = {2025},
  eprint        = {2506.02622},
  archivePrefix = {arXiv},
  primaryClass  = {cs.RO},
  url           = {https://github.com/RICE-unige/horus},
  pdf           = {https://arxiv.org/abs/2506.02622},
  note          = {arXiv preprint arXiv:2506.02622}
}
```

## :mailbox_with_mail: Contact

- Omotoye Shamsudeen Adekoya
- Email: `omotoye.adekoya@edu.unige.it`

## :bulb: Acknowledgments

This project is part of a PhD research effort at the University of Genoa, under the supervision of:

- Prof. Carmine Recchiuto
- Prof. Antonio Sgorbissa

Developed at **RICE Lab**, University of Genoa.

## :handshake: Contributing

See [`CONTRIBUTING.md`](CONTRIBUTING.md). Include reproducible test steps for behavior changes.

## :page_facing_up: License

Apache-2.0. See [`LICENSE`](LICENSE).
