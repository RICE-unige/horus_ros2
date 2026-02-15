# HORUS Unity Bridge (`horus_unity_bridge`)

<div align="center">

```text
██╗  ██╗ ██████╗ ██████╗ ██╗   ██╗███████╗
██║  ██║██╔═══██╗██╔══██╗██║   ██║██╔════╝
███████║██║   ██║██████╔╝██║   ██║███████╗
██╔══██║██║   ██║██╔══██╗██║   ██║╚════██║
██║  ██║╚██████╔╝██║  ██║╚██████╔╝███████║
╚═╝  ╚═╝ ╚═════╝ ╚═╝  ╚═╝ ╚═════╝ ╚══════╝
```
</div>

> [!IMPORTANT]
> `horus_unity_bridge` is the ROS 2 runtime bridge between HORUS SDK clients and Unity-based MR applications.
> It preserves Unity ROS-TCP protocol compatibility while adding HORUS-specific operational behavior.

## Operator Scope

This package handles:
- TCP server lifecycle for Unity client connectivity,
- topic/service routing between Unity protocol and ROS 2 graph,
- optional WebRTC signaling + media session runtime,
- bridge diagnostics for integration triage.

## Runtime Components

- `ConnectionManager`: socket lifecycle and client handling
- `MessageRouter`: protocol command routing, publisher/subscriber binding, WebRTC signaling dispatch
- `TopicManager`: ROS 2 topic registry + QoS handling
- `ServiceManager`: ROS/Unity service proxy handling
- `WebRTCManager` (optional): offer/candidate handling, SDP/ICE flow, RTP sender pipeline

## Dependency Matrix

| Dependency | Required | Why |
|---|---|---|
| ROS 2 Humble/Jazzy | Yes | Core runtime |
| `nlohmann-json3-dev` | Yes | Protocol payload serialization |
| `libunwind-dev` | Yes | Runtime/toolchain dependency |
| GStreamer dev libs | WebRTC mode | Media encode/payloader pipeline |
| OpenCV dev libs | WebRTC mode | Image decode/format conversion |
| `libdatachannel` | WebRTC mode | WebRTC signaling/peer connection (fetched by CMake) |

Install baseline + WebRTC deps:

```bash
sudo apt-get update
sudo apt-get install -y \
  nlohmann-json3-dev \
  libunwind-dev \
  pkg-config \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  libgstreamer-plugins-bad1.0-dev \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav \
  libopencv-dev
```

## Build and Launch

### WebRTC enabled

```bash
cd ~/horus_ws/src/horus_ros2
colcon build --packages-select horus_unity_bridge --cmake-args -DENABLE_WEBRTC=ON
source install/setup.bash
ros2 launch horus_unity_bridge unity_bridge.launch.py
```

### WebRTC disabled

```bash
cd ~/horus_ws/src/horus_ros2
colcon build --packages-select horus_unity_bridge --cmake-args -DENABLE_WEBRTC=OFF
source install/setup.bash
ros2 launch horus_unity_bridge unity_bridge.launch.py
```

## Key Runtime Configuration

Primary configuration file:
- `config/bridge_config.yaml`

Important keys:
- network: `tcp_ip`, `tcp_port`, `max_connections`
- performance: `socket_buffer_size`, `message_queue_size`, `worker_threads`
- QoS defaults: publisher/subscriber reliability + durability + depth
- WebRTC block:
  - `enabled`
  - signaling topics
  - session timeout
  - bitrate/framerate defaults
  - encoder and STUN defaults

## WebRTC Negotiation Behavior

### Negotiation strategy
- Parse Unity offer video m-section before applying remote description.
- Select active `mid` from offer.
- Select H264 payload type from offer (prefers `packetization-mode=1` when available).
- Keep SDP payload type and `rtph264pay` payload type aligned.

### Signaling gates
- Emit `ready` only after successful offer handling.
- Emit `error` and clean session on negotiation/setup failure.

> [!TIP]
> This gate avoids the common "session appears connected but white panel" condition caused by negotiation mismatch.

## Diagnostics Decoding Guide

Bridge telemetry includes:
- negotiated video MID,
- selected H264 payload type,
- first outbound RTP packet header (`pt`, `ssrc`, `seq`),
- periodic RTP counters.

Interpretation:
- MID/PT mismatch risk -> inspect offer parse + selected PT logs.
- signaling appears fine but no frames -> compare RTP counters vs Unity inbound stats.
- rapid close after checking -> inspect network/ICE candidate path and host topology.

## Failure Taxonomy

| Symptom | Likely Class | First Check |
|---|---|---|
| No answer/ready | Signaling flow | signaling topic wiring + QoS compatibility |
| ICE checking -> closed | Transport path | candidate reachability / network topology |
| Connected but white panel | Decode/PT mismatch | negotiated MID/PT + inbound RTP stats |
| Frames stall after start | Media pipeline/runtime load | RTP counters + publisher/input rate |

## Deployment Checklist

- [ ] Built with intended mode (`ENABLE_WEBRTC` ON/OFF)
- [ ] `bridge_config.yaml` reviewed for current test environment
- [ ] Signaling topics match SDK/MR settings
- [ ] Unity client can connect to TCP bridge on expected host/port
- [ ] Diagnostic logs enabled for integration runs
- [ ] Repro commands recorded for experiment repeatability

## Test Entry Points

```bash
colcon build --packages-select horus_unity_bridge horus_unity_bridge_test --cmake-args -DENABLE_WEBRTC=ON
source install/setup.bash

ros2 launch horus_unity_bridge unity_bridge.launch.py &
ros2 run horus_unity_bridge_test test_publisher
ros2 run horus_unity_bridge_test unity_client_simulator
```

## Stack Links

- ROS 2 infrastructure repo: <https://github.com/RICE-unige/horus_ros2>
- SDK repo: <https://github.com/RICE-unige/horus_sdk>
- Unity MR app repo: <https://github.com/RICE-unige/horus>

## Acknowledgments

`horus_unity_bridge` design and interoperability strategy were strongly informed by the Unity ROS networking stack:

- Unity ROS TCP Endpoint: <https://github.com/Unity-Technologies/ROS-TCP-Endpoint>
- Unity ROS TCP Connector package <https://github.com/Unity-Technologies/ROS-TCP-Connector>

These references helped guide compatibility decisions while extending the bridge for HORUS-specific runtime and WebRTC features.

## License

Apache-2.0.
