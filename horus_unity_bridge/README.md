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
> It speaks the HORUS HorusLink transport on the Unity-facing side and keeps ROS 2 CDR strictly at the DDS edge.

## Operator Scope

This package handles:
- TCP server lifecycle for Unity client connectivity,
- topic/service routing between HorusLink channels and the ROS 2 graph,
- optional WebRTC signaling + media session runtime,
- bridge diagnostics for integration triage.

## Runtime Components

- `HorusLinkConnectionManager`: dual-lane realtime/bulk socket lifecycle and client handling.
  Each Unity client opens two sockets that are matched into one connection by the `session_id`
  carried in their initial `hello` handshake (session-hello lane pairing). Unpaired sockets are
  held in pending-realtime/pending-bulk queues, pruned on timeout
  (`horuslink_pending_socket_timeout_ms`), and rejected if the two halves arrive too far apart
  (`horuslink_pending_socket_pair_max_skew_ms` skew guard). Per-connection outbound lane queues
  (`horuslink_outbound_lane_queue.cpp` / `horuslink_outbound_frame_router.cpp`) apply
  reliable-overflow backpressure — a `ReliableFifo` frame that overflows triggers a drain-and-retry
  of the lane. On (re)subscribe the manager replays cached retained payloads for the topic so a
  reconnecting client re-syncs deterministically.
- `MessageRouter`: channel routing, publisher/subscriber binding, WebRTC signaling dispatch,
  retained-payload cache and replay
- `TopicManager`: ROS 2 topic registry, QoS handling, and CDR/HorusLink payload conversion
- `ServiceManager`: ROS/Unity service proxy handling
- `WebRTCManager` (optional): offer/candidate handling, SDP/ICE flow, RTP sender pipeline

## Dependency Matrix

| Dependency | Required | Why |
|---|---|---|
| ROS 2 Jazzy | Yes | Core runtime |
| `nlohmann-json3-dev` | Yes | Protocol payload serialization |
| GStreamer dev libs | WebRTC mode | Media encode/payloader pipeline |
| OpenCV dev libs | WebRTC mode | Image decode/format conversion |
| `libdatachannel` | WebRTC mode | WebRTC signaling/peer connection; prefer a system CMake package, or opt into the pinned source fallback |

Install baseline + WebRTC deps:

```bash
sudo apt-get update
sudo apt-get install -y \
  nlohmann-json3-dev \
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

`ENABLE_WEBRTC` is off by default for reproducible offline builds. If your system
does not provide a `LibDataChannel` CMake package, the bridge can use a pinned
source fallback only when explicitly requested with
`-DHORUS_ALLOW_LIBDATACHANNEL_FETCH=ON`.

## Build and Launch

### WebRTC enabled

```bash
cd ~/horus_ws/src/horus_ros2
colcon build --packages-select horus_unity_bridge --cmake-args -DENABLE_WEBRTC=ON
source install/setup.bash
ros2 launch horus_unity_bridge unity_bridge.launch.py
```

If no system `LibDataChannel` package is installed, use the pinned fallback:

```bash
colcon build --packages-select horus_unity_bridge --cmake-args \
  -DENABLE_WEBRTC=ON \
  -DHORUS_ALLOW_LIBDATACHANNEL_FETCH=ON
```

### WebRTC disabled

```bash
cd ~/horus_ws/src/horus_ros2
colcon build --packages-select horus_unity_bridge
source install/setup.bash
ros2 launch horus_unity_bridge unity_bridge.launch.py
```

## Key Runtime Configuration

Primary configuration file:
- `config/bridge_config.yaml`

Important keys:
- network: `tcp_ip`, `tcp_port`, `transport_protocol`, `horuslink_bulk_port`, `max_connections`
- performance: `socket_buffer_size`, `worker_threads`
- TCP/socket tuning: `tcp_nodelay`, `keepalive_interval_ms`
- HorusLink transport:
  - `horuslink_max_payload_size`
  - `horuslink_keepalive_ms`
  - `horuslink_tcp_keepalive`
  - `horuslink_socket_send_timeout_ms`
  - lane-pairing: `horuslink_pending_socket_timeout_ms`, `horuslink_pending_socket_pair_max_skew_ms`
- control leases (multi-operator):
  - `multi_operator.control_lease_ttl_ms`
  - `multi_operator.control_lease_denied_event_rate_limit_ms`
- feature flags: `enable_dynamic_registration`, `enable_service_calls`, `enable_topic_list_query`
- diagnostics: `log_protocol_messages`
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

For HorusLink wire captures, use the checked-in frame inspector:

```bash
cd ~/horus_ws/src/horus_ros2/horus_unity_bridge
tools/horuslink_frame_inspector.py capture.bin --pretty
tools/horuslink_frame_inspector.py --hex capture.hex --pretty
```

The inspector decodes the fixed 16-byte frame header, frame flags, and binary
TLV control payloads. It is useful when comparing Unity and bridge logs for
lane negotiation, subscribe acknowledgments, and service correlation IDs.

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

### Automated suite (CTest)

The package ships a gtest suite plus a Python frame-inspector test, both wired into `BUILD_TESTING`:

- `horus_unity_bridge_tests` — 9 gtest translation units covering the HorusLink protocol,
  channel table, connection manager, control messages, light codecs, outbound frame router,
  outbound lane queue, session, and the bridge node. Wire-format expectations are pinned against
  the golden hex vectors in `test/golden_vectors/`.
- `horuslink_frame_inspector_py` — CTest that runs `test/test_horuslink_frame_inspector.py`
  against the checked-in captures.

```bash
colcon build --packages-select horus_unity_bridge
colcon test --packages-select horus_unity_bridge
colcon test-result --verbose
```

### Manual runtime targets

Manual integration targets live in the companion `horus_unity_bridge_test` package
(`test_publisher`, `unity_client_simulator`, `test_service_server`, `test_service_client`):

```bash
colcon build --packages-select horus_unity_bridge horus_unity_bridge_test
source install/setup.bash

ros2 launch horus_unity_bridge unity_bridge.launch.py &
ros2 run horus_unity_bridge_test test_publisher
ros2 run horus_unity_bridge_test unity_client_simulator
ros2 run horus_unity_bridge_test test_service_server
ros2 run horus_unity_bridge_test test_service_client
```

## Stack Links

- ROS 2 infrastructure repo: <https://github.com/RICE-unige/horus_ros2>
- SDK repo: <https://github.com/RICE-unige/horus_sdk>
- Unity MR app repo: <https://github.com/RICE-unige/horus>

## Acknowledgments

`horus_unity_bridge` is maintained as the HORUS-specific ROS 2 integration
bridge for mixed-reality robot operation, including HorusLink transport,
control leases, service routing, diagnostics, and optional WebRTC media
coordination.

## License

Apache-2.0.
