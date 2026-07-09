# HORUS ROS 2 Infrastructure

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-22314E)](https://docs.ros.org)
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
- Unity-compatible HorusLink dual-lane transport behavior (realtime port `10000` + bulk port `10001`),
- optional WebRTC camera signaling/media pipeline,
- multi-client Unity bridge fanout/cleanup for multi-operator sessions,
- bridge-authoritative per-robot control lease arbitration and command-topic enforcement,
- backend package boundaries for scalability experiments.

It does not own SDK payload authoring (`horus_sdk`) or Unity scene/runtime UX (`horus`).

## :jigsaw: Packages and Responsibilities

| Package | Responsibility |
|---|---|
| `horus_unity_bridge` | HorusLink (dual-lane) / WebRTC bridge runtime between ROS 2 and Unity client, including multi-client fanout and control-lease enforcement |
| `horus_backend` | Backend registration/state management logic |
| `horus_interfaces` | ROS 2 interfaces used by HORUS components |
| `horus_unity_bridge_test` | Test utilities and simulation helpers |

## :building_construction: Architecture

```text
HORUS SDK (registration + orchestration)
         <->
horus_unity_bridge (ROS 2 node, HorusLink dual-lane / WebRTC)
         <->
HORUS MR App (Unity client)
```

Default ingress (HorusLink dual-lane):
- realtime lane: `0.0.0.0:10000` (`tcp_port`)
- bulk lane: `0.0.0.0:10001` (`horuslink_bulk_port`)

WebRTC signaling topics:
- `/horus/webrtc/client_signal`
- `/horus/webrtc/server_signal`

Field-teammate / HoloLens integration:
- HoloLens devices do not connect to `horus_unity_bridge` directly.
- The offboard field-teammate relay connects to the HoloLens, then publishes
  normal ROS 2 TF, status, confidence, FPV, and guidance topics.
- `horus_sdk` registers those topics through the field-teammate payload, and
  the bridge routes them like any other registered sensor/task stream.
- No HoloLens-specific TCP listener or HorusLink client is required in this
  repository for the local/core study path.

### HorusLink Transport

The bridge default transport is HorusLink, a dual-lane framed protocol (`transport_protocol: "horuslink"` in `bridge_config.yaml`):
- realtime lane on `tcp_port` (`10000`) for low-latency control/state traffic,
- bulk lane on `horuslink_bulk_port` (`10001`) for large/chunked payloads (robot descriptions, maps),
- reliable framed delivery with per-lane outbound queues and light codecs (`src/horuslink_*`),
- retained-state replay so late joiners receive the current registry/map state during SDK registry replay windows (`MessageRouter::replay_retained_payloads`).

Transport tooling:
- `horus_unity_bridge/tools/horuslink_frame_inspector.py` decodes/inspects HorusLink frames off the wire,
- golden frame vectors (`horus_unity_bridge/test/golden_vectors`, `horuslink_golden_vectors.hpp`) pin the wire format under the package gtest/CTest suite.

Recommended go-to-point task topics:
- `/<robot>/goal_pose` (`geometry_msgs/PoseStamped`)
- `/<robot>/goal_cancel` (`std_msgs/String`, payload: `"cancel"`)
- `/<robot>/goal_status` (`std_msgs/String`)

Recommended waypoint task topics:
- `/<robot>/waypoint_path` (`nav_msgs/Path`)
- `/<robot>/waypoint_status` (`std_msgs/String`, JSON status payload)

Recommended drone task command topics:
- `/<robot>/takeoff` (`std_msgs/Empty`)
- `/<robot>/land` (`std_msgs/Empty`)

Recommended navigation visualization topics:
- `/<robot>/global_path` (`nav_msgs/Path`)
- `/<robot>/local_path` (`nav_msgs/Path`)
- `/<robot>/odom` (`nav_msgs/Odometry`)
- `/<robot>/collision_risk` (`std_msgs/String`, JSON payload)

Recommended global 3D map topics:
- `/map_3d` (`sensor_msgs/PointCloud2`, global pointcloud map stream)
- `/map_3d_mesh` (`visualization_msgs/Marker`, single `TRIANGLE_LIST` mesh stream, recommended Quest mesh path in current stable workflow)
- `/map_3d_octomap_mesh` (`visualization_msgs/Marker`, `TRIANGLE_LIST` mesh stream for octomap mode)
- `/map_3d_octomap` (`octomap_msgs/Octomap`, optional native octomap metadata/topic for interoperability)

> [!NOTE]
> Mesh conversion flow for `/map_3d -> /map_3d_mesh` is orchestrated by `horus_sdk` tooling (`tools/voxblox_mesh_to_horus_marker.py` for the Voxblox/Cow & Lady workflow, or `tools/ply_to_horus_mesh_marker.py` for the ROS 2-native PLY fallback), with SDK-side map registration via `mesh_map_registration.py` / `pointcloud_map_registration.py` / `occupancy_map_registration.py` / `octomap_registration.py`, and snapshot-first defaults for Quest stability.
> Marker-array transport flags remain accepted in SDK for compatibility but are currently coerced to marker-only runtime behavior with warning logs.
> Replay-time bounded republish bursts are used so late joiners receive map data during SDK registry replay windows.

Recommended robot-description transport topics:
- `/horus/robot_description/request` (`std_msgs/String`, JSON request envelope)
- `/horus/robot_description/chunk_begin` (`std_msgs/String`, JSON begin envelope)
- `/horus/robot_description/chunk_item` (`std_msgs/String`, JSON chunk envelope)
- `/horus/robot_description/chunk_end` (`std_msgs/String`, JSON end envelope)
- `/horus/robot_description/replies/<requester>` (per-requester reply channel; bridge routes chunked replies back to the requesting client)

Robot-task backend baseline:
- `horus_backend` includes an optional Nav2 action adapter path for `goal_pose` / `goal_cancel` / `goal_status`.
- Adapter auto-enables when both `nav2_msgs` and `rclcpp_action` are available at build time.
- When Nav2 dependencies are missing, backend still builds and runs with adapter disabled.
- SDK/MR can now run a flat single-robot workflow without ROS namespace prefixes; bridge routing does not change for this mode when explicit topics are registered.
- `horus_backend` also accepts explicit metadata-driven Nav2 topics for flat-root robots:
  - `horus.backend.nav2_action_topic`
  - `horus.backend.goal_topic`
  - `horus.backend.cancel_topic`
  - `horus.backend.status_topic`
- If these metadata keys are absent, backend behavior remains legacy-prefixed (`/<robot>/...`).

## :white_check_mark: Prerequisites

- ROS 2 Jazzy
- `rosdep`
- C++ build toolchain (`colcon`, CMake)
- For WebRTC builds: GStreamer + OpenCV dependencies (see [`horus_unity_bridge/README.md`](horus_unity_bridge/README.md))

## :gear: Build Matrix

| Mode | Command |
|---|---|
| Standard bridge | `colcon build --packages-select horus_unity_bridge` |
| WebRTC-enabled bridge, system `LibDataChannel` | `colcon build --packages-select horus_unity_bridge --cmake-args -DENABLE_WEBRTC=ON` |
| WebRTC-enabled bridge, pinned source fallback | `colcon build --packages-select horus_unity_bridge --cmake-args -DENABLE_WEBRTC=ON -DHORUS_ALLOW_LIBDATACHANNEL_FETCH=ON` |
| Backend + bridge (auto Nav2 detect) | `colcon build --packages-select horus_backend horus_unity_bridge` |

Recommended setup:

```bash
git clone https://github.com/RICE-unige/horus_ros2.git ~/horus_ws/src/horus_ros2
cd ~/horus_ws/src/horus_ros2
rosdep install --from-paths . -y --ignore-src
colcon build --symlink-install --packages-select horus_unity_bridge
source install/setup.bash
```

WebRTC builds are opt-in so the standard bridge can build without network fetches.
Use `-DENABLE_WEBRTC=ON` with an installed `LibDataChannel` package, or add
`-DHORUS_ALLOW_LIBDATACHANNEL_FETCH=ON` to use the pinned source fallback.

## :rocket: Launch

```bash
ros2 launch horus_unity_bridge unity_bridge.launch.py
```

> [!NOTE]
> Runtime defaults are managed in `horus_unity_bridge/config/bridge_config.yaml`.

### Backend Launch

```bash
# Backend registration/state node
ros2 launch horus_backend horus_backend.launch.py

# Backend + bridge together
ros2 launch horus_backend horus_complete_backend.launch.py
```

The backend node exposes an optional Nav2 action adapter path for `goal_pose` / `goal_cancel` / `goal_status`
(auto-enabled when `nav2_msgs` and `rclcpp_action` are available at build time).

> [!NOTE]
> Backend defaults are managed in `horus_backend/config/backend_config.yaml`. Key parameters:
> - `unity_tcp_port` (default `10000`) - bridge ingress port the backend targets,
> - `enable_unity_bridge` (default `true`) - toggle the Unity bridge integration,
> - `default_robot_type` (default `generic`) - default robot type applied to registered robots.

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
- Workspace sharing mode (`Shared Host`, `Shared Join`, `Private Workspace`) is decided in the Unity app; the bridge remains the shared robot/task transport and lease-arbitration layer for all connected operators.

### Multi-Operator Bridge Baseline
Current multi-operator infrastructure now integrated in `main` includes:
- multi-client subscriber fanout (prevents last-subscriber-wins behavior),
- client disconnect cleanup for subscriptions/publishers/services/session state,
- per-robot control lease arbitration (`/horus/multi_operator/control_lease_request`),
- inactive-holder acquire preemption (if holder has no active `panel_open`/`task_active`/`teleop_active`, acquire is reassigned immediately),
- protected command-topic enforcement with lease-denied events,
- multi-client publisher ownership refcounting (prevents one client disconnect from tearing down shared publishers),
- late-joiner retained-state replay and host/join session replay (`MessageRouter::replay_retained_payloads`, `is_horus_retained_state_topic`).

Lease state snapshots are published on `/horus/multi_operator/control_lease_state` for MR clients (and optional SDK observability).

Multi-operator control/replay topics:
- `/horus/multi_operator/control_lease_request` (`std_msgs/String`, lease acquire/release request)
- `/horus/multi_operator/control_lease_state` (`std_msgs/String`, lease state snapshot)
- `/horus/multi_operator/control_topic_catalog` (`std_msgs/String`, protected control-topic catalog)
- `/horus/multi_operator/sdk_registry_replay_begin` (`std_msgs/String`, retained registry replay begin)
- `/horus/multi_operator/sdk_registry_replay_item` (`std_msgs/String`, retained registry replay item)
- `/horus/multi_operator/sdk_registry_replay_end` (`std_msgs/String`, retained registry replay end)

> [!NOTE]
> Bridge-side lease arbitration applies equally to operators using a shared workspace and operators using a private workspace. Workspace geometry/session sync is an app-level concern, not a bridge-level role distinction.

## :test_tube: Validation Workflow

```bash
# 1) Start bridge
ros2 launch horus_unity_bridge unity_bridge.launch.py

# 2a) In the SDK repo, start the matching fake runtime for TF/camera/task topics
PYTHONPATH=python:$PYTHONPATH python3 python/examples/legacy/fake_tf_ops_suite.py --robot-count 10 --rate 30 --static-camera --publish-compressed-images --task-path-publish-rate 5 --publish-collision-risk --collision-threshold-m 1.2

# 2b) In a second SDK terminal, register a typical ground fleet
PYTHONPATH=python:$PYTHONPATH python3 python/examples/ops_registration.py

# 3) Optional robot-description validation (collision + joints)
#    Fetch pinned URDFs/meshes once, run the fake runtime, then register
python3 python/examples/tools/fetch_robot_description_assets.py --force
PYTHONPATH=python:$PYTHONPATH python3 python/examples/legacy/fake_tf_robot_description_suite.py --robot-profile real_models
PYTHONPATH=python:$PYTHONPATH python3 python/examples/robot_description_registration.py

# 4) Optional drone flow validation (takeoff/land + 3D go-to/waypoint/draw-path in MR)
PYTHONPATH=python:$PYTHONPATH python3 python/examples/legacy/fake_tf_drone_ops_suite.py --min-altitude 0.0 --max-altitude 25.0 --takeoff-altitude 1.2
PYTHONPATH=python:$PYTHONPATH python3 python/examples/drone_registration.py
```

> [!NOTE]
> Most current registration examples run their `register_*()` at module top level and take no CLI flags.
> Run each fake runtime and its matching registration command in separate SDK terminals. Examples that do accept argparse flags
> (e.g. `uav_sim_horus_registration.py`, `fleet_robot_description_registration.py`,
> `hcore_heterogeneous_registration.py`, `mesh_map_registration.py`) support `--dry-run`/`--once` where available.
> Stress-test runtimes, advanced flags, and tutorial/live (Carter/Unitree) flows live in `python/examples/legacy/`
> (see `python/examples/legacy/README.md`).

Expected outcomes:
- bridge accepts Unity client connection,
- registration/ack/heartbeat channels remain stable,
- WebRTC sessions negotiate and stream when requested,
- robots remain static until teleop/task command topics are used,
- active teleop commands can preempt active go-to-point/waypoint execution,
- aerial commands (`takeoff`/`land`) pass through bridge without extra protocol wiring.

## :warning: Known Constraints

- Performance envelope depends on robot count, camera resolutions, and bridge host decode/encode budget.
- Network topology (WSL/NAT/localhost) can affect ICE behavior in WebRTC mode.
- For occupancy map validation, use transient-local publishers (the SDK fake publisher already does this by default).

## :world_map: Roadmap

> [!NOTE]
> This roadmap is scoped to ROS 2 infrastructure ownership (bridge/runtime reliability, observability, and reproducibility), not Unity UX or SDK dashboard behavior. Current baseline aligns with the now-merged HorusLink reliable dual-lane transport and retained-state replay, marker-only mesh transport stability in the integrated SDK/MR workflow, explicit flat single-robot backend-topic compatibility, and bridge-global lease behavior across shared/private app workspace modes.

| Stream | Status | Current Baseline | Next Milestone |
|---|---|---|---|
| Bridge Runtime Stability | :white_check_mark: Active baseline | HorusLink dual-lane transport path, WebRTC signaling/session flow, multi-client subscriber fanout, and publisher ownership refcounting are integrated in `main`, with negotiation diagnostics available (MID/PT/RTP counters). | Add long-duration soak tests and explicit reconnection stress scenarios. |
| HorusLink Transport Reliability | :white_check_mark: Active baseline | HorusLink is the default transport (`transport_protocol: "horuslink"`), with dual-lane realtime/bulk delivery (`10000`/`10001`), reliable framed queues, retained-state replay for late joiners, and a frame inspector + golden wire vectors under the package gtest/CTest suite. | Extend multi-operator soak coverage and add per-lane throughput/backpressure telemetry. |
| WebRTC Media Reliability | :large_orange_diamond: In progress | Negotiation mismatch fixes plus stall telemetry and paced keyframe recovery are integrated (paced-keyframe / stall gating in `src/message_router.cpp`; negotiation and keyframe send in `src/webrtc_manager.cpp`). | Harden topology/scale edge cases for network topology variance and high robot counts. |
| QoS and Session Policy | :large_orange_diamond: In progress | Reliable/volatile signaling defaults in place with current compatibility behavior. | Add topic/session policy profiles for workload-specific tuning. |
| Robot Task Routing | :large_orange_diamond: In progress | Go-to-point/waypoint topic guidance is defined, aerial `takeoff`/`land` command topics are validated in fake runtime workflows, and optional Nav2 adapter flow is integrated with build-time dependency gating. | Add multi-robot task execution stress validation (ground + aerial) and richer task-status observability. |
| Flat Single-Robot Backend Compatibility | :large_orange_diamond: In progress | `horus_backend` can now consume explicit metadata-driven Nav2 topics for single robots that publish flat root topics instead of `/<robot>/...`; bridge-side topic routing remains unchanged when SDK registration already provides explicit topic names. | Add targeted validation docs/tests for mixed prefixed + flat deployments and backend diagnostics for mismatched metadata. |
| Robot Description Transport | :large_orange_diamond: In progress | Bridge pass-through for request/chunk topics plus per-requester reply routing (`/horus/robot_description/replies/<requester>`) is active and used by SDK/MR Robot Description V1 flow. | Add targeted transport diagnostics and replay stress tests for chunked description traffic under multi-robot load. |
| 3D Map Transport Compatibility | :large_orange_diamond: In progress | Marker-based global 3D map topic guidance is aligned across stack (`/map_3d`, `/map_3d_mesh`, `/map_3d_octomap_mesh`) with snapshot-first conversion defaults and replay-window map republish behavior handled in SDK tooling. | Add explicit bridge-level diagnostics and stress tests focused on 3D map traffic during multi-operator join/rejoin windows. |
| Multi-Operator Control Arbitration | :large_orange_diamond: In progress | Bridge-side per-robot control lease arbiter, protected command-topic enforcement, lease state snapshots, and late-joiner retained-state / host-join session replay (`sdk_registry_replay_begin|item|end`, `replay_retained_payloads`) are integrated in `main`. | Harden lease telemetry/observability, tune TTL policies, and extend regression coverage for repeated join/rejoin contention scenarios. |
| Integration Automation | :large_orange_diamond: In progress | Bridge ships a gtest + Python CTest suite (HorusLink codecs/protocol/session, golden wire vectors, `test_horuslink_frame_inspector.py`); remaining validation is manual (SDK registration examples + Unity runtime checks). | Add CI-driven integration matrix across SDK, bridge, and Unity harness scenarios. |
| Observability and Metrics | :white_circle: Planned | Logs provide actionable diagnostics during runtime issues. | Export machine-readable metrics for dashboards and regression tracking. |
| Benchmarking and Reproducibility | :large_orange_diamond: In progress | Golden wire vectors pin the HorusLink frame format, and SDK ships a serializer throughput benchmark (`examples/tools/throughput_benchmark.py`). | Publish repeatable throughput/latency benchmark suite for research reporting. |

## :link: Related Repositories

- SDK: <https://github.com/RICE-unige/horus_sdk> (tri-language SDK: Python / C++ / Rust parity)
- Unity MR app: <https://github.com/RICE-unige/horus>

> [!NOTE]
> Cross-repo context (owned outside this repository):
> - **Tri-language SDK**: the HORUS SDK provides Python, C++, and Rust registration bindings; this repo's transport/runtime is language-agnostic and serves all three.
> - **Map showcases**: SDK map tooling produces mesh / point-cloud / octomap / Gaussian-splat map showcases, which this bridge transports over the HorusLink bulk lane.

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
