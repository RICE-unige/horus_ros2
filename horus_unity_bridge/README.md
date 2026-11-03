# HORUS Unity Bridge (`horus_unity_bridge`)

Live HMSH v2 map subscriptions can explicitly set the optional HorusLink
`LiveMap` subscribe TLV (21, one byte, value 1). This requests reliable/volatile
ROS delivery and disables the bridge's historical retained-payload cache for
that subscription, independently of topic names. The SDK adapter owns current
map state and serves manifest-driven repair. Legacy subscriptions omit the TLV
and retain their existing static-map behavior. Clients sharing a topic must
agree on this policy; conflicting requests are rejected. Use the matching MR
`indexed_live` registration option through the SDK's `add_live_mesh()` helper.

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
  (`horuslink_outbound_lane_queue.cpp` / `horuslink_outbound_frame_router.cpp`) have independent
  writer threads. ROS callbacks enqueue owned data and return without waiting for TCP writes.
  A single writer preserves data-frame sequence order within each lane. Latest-state streams
  coalesce per channel; reliable map chunks remain ordered. On (re)subscribe the manager replays
  cached retained payloads. Non-retained incremental map sources must provide snapshot/repair
  replay themselves; reconnect alone cannot reconstruct an arbitrary live map.
- `MessageRouter`: channel routing, publisher/subscriber binding, WebRTC signaling dispatch,
  retained-payload cache and replay
- `TopicManager`: ROS 2 topic registry, QoS handling, and CDR/HorusLink payload conversion
- `ServiceManager`: ROS/Unity service proxy handling
- `WebRTCManager` (optional): offer/candidate handling, SDP/ICE flow, H.264 RTP
  sender pipeline, and bounded remote-render auxiliary data channel

### Load isolation and capacity

- `tf2_msgs/TFMessage` data is always forwarded as reliable FIFO, even when an
  older client requests replace-latest. A TF packet is a partial tree update,
  not a complete state: dropping one for another packet can lose unrelated
  robot/camera/joint transforms. The wire flag is cleared so existing Quest
  clients also preserve these updates in their receive queue. This is bounded
  TCP delivery, not a guarantee against DDS source loss or unlimited input.
- `nav_msgs/OccupancyGrid` subscriptions use a live volatile reader plus a
  reliable transient-local snapshot reader. Publication identities suppress
  duplicate/out-of-order samples from the same publisher. This accepts ordinary
  live map publishers and saved maps published before subscription. One latest
  grid is cached for additional clients and released when the last leaves;
  the source does not need to resend its full map periodically.
- Each client has separate realtime and bulk data queues and writers. A slow client does not
  hold ROS executor workers in socket writes or block another client's sends.
- Defaults in `HorusLinkConnectionManager::Config`: 1,024 queued frames per lane, 8 MiB realtime
  and 64 MiB bulk queued wire bytes. One legal oversized frame may occupy an otherwise empty
  queue, up to `horuslink_max_payload_size`; each writer can additionally hold one in-flight
  frame. These are queue limits, not a bound on total process RSS or kernel socket buffers.
- Bulk scheduling gives a pending replace-latest channel a turn after four reliable frames or
  1 MiB of reliable payload. Reliable frames retain their relative order, including across
  topics. Scheduling is between frames; a large TCP frame cannot be interrupted mid-payload.
  Use bounded source chunks for maps, not one monolithic multi-megabyte update.
- Accepted reliable chunks are never silently replaced. If a slow client's reliable queue
  exhausts its frame/byte budget, its session is explicitly closed for resynchronization.
  Other clients remain active. Admission returns `false` on failure; `true` means **queued**,
  not peer acknowledgement or headset display. Provision source rates below sustained link
  and receiver capacity; no finite queue can guarantee lossless unlimited input.
- ROS callback groups serialize each individual topic while allowing different topics to run
  concurrently. This prevents same-topic map callbacks overtaking during copying/fanout.
- Quest receive queues coalesce only replace-latest data for the same channel, before filling.
  Reliable data uses bounded receive-side backpressure. The main-thread bulk budget still
  applies; this transport change does not lower geometry fidelity or change rendering quality.
- Quest sending also uses bounded background writers, so a stalled bridge cannot hold the
  Unity main thread in a socket write. Shutdown interrupts pending writes before joining.
- Disconnected bridge sessions are reclaimed during the accept loop, without waiting for
  another connection. Topic, router and service counters use atomic storage and return value
  snapshots under concurrent callbacks. Each initial HELLO header/payload read has a total
  deadline and bounded shutdown polling.

The existing periodic bridge statistics now report per-client/per-lane queue frames/bytes,
peak queue bytes, accepted/sent/replaced/rejected counts, sent wire bytes and maximum socket
write time. Sent bytes count completed local writes, not confirmed display. Compare these
with ROS source rate and headset receive/apply timing to distinguish source stalls, slow
Wi-Fi, queue pressure and rendering stalls. Keep protocol-payload logging off during profiling.

## Dependency Matrix

### Service request lifecycle

HorusLink service correlation IDs are local to a connection. The router allocates
an internal ID for each outstanding call and restores the original client ID on
the response. Two operators may therefore use the same wire ID safely; duplicate
outstanding IDs from one operator are rejected instead of overwriting a request.
Service registrations have shared ownership. Unsubscribing or disconnecting one
operator cancels only its pending calls and removes the ROS client only after the
last owner leaves.

Unity-to-ROS calls use Jazzy GenericClient with native request/response objects
constructed from installed C++ type support. They are not casts of serialized
buffers. The matching service package and its type support must be installed.
Conflicting type registrations are rejected. Pending requests expire after ten
seconds of steady time, including while ROS simulation time is paused; timeout,
cancellation and disconnect remove pending ROS state. A failed/expired call is
not sent as a fabricated successful response: the Unity caller's timeout remains
the outward failure signal.

This does not implement arbitrary reverse Unity service servers or generic ROS
actions. Reverse ServiceManager implementations remain limited to AddTwoInts,
Trigger and SetBool. The optional backend Nav2 adapter is a separate capability.

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

When explicitly enabled, missing WebRTC development dependencies fail the build;
support is never silently disabled. The installed package share directory contains
`build_capabilities.json` with the compiled WebRTC capability for local launchers.

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
  - Raw and compressed image readers use best-effort, volatile, keep-last-one
    QoS, matching both best-effort sensor publishers and reliable publishers.
    This is selected by ROS message type, not topic name or robot model.
    HorusLink lane and delivery settings are independent of ROS QoS.
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
- Reuse the cached answer for an identical offer retry; changed offers replace
  the session. Start encoding only after peer/track readiness, not ICE alone.

### Source Media And Encoded Relay

With WebRTC enabled, `horus_media_node` supplies the same media sender without
a TCP server or a second control-lease authority. The connector can manage it
on a remote robot PC and receive H.264 access units on the operator PC without
decoding/re-encoding them. The operator bridge can explicitly accept those local
CompressedImage topics as H.264 even when the SDK camera uses its compatible
`image_type: compressed` value:

```yaml
horus_unity_bridge:
  ros__parameters:
    webrtc:
      preencoded_topics: ["/horus/media/front/h264"]
```

Supply a complete bridge YAML using the launch argument `config_file:=/path/to/config.yaml`;
the SDK's optional `HORUS_BRIDGE_CONFIG` passes that path to its auto-started bridge.
The default configuration and normal raw/JPEG camera behavior are unchanged.
These topics must actually contain Annex-B H.264 access units, not JPEG data.
The sender retains prediction order, uses bounded queues and forwards keyframe
requests to `<topic>/request_keyframe`. Its RTP fragment budget is 1100 bytes;
image pixels, configured bitrate and frame rate are not reduced by packetization.
See the connector's `docs/performance.md` for source/relay configuration and
the distinction between encoded receipt and headset display measurements.

> [!TIP]
> This gate avoids the common "session appears connected but white panel" condition caused by negotiation mismatch.

### Experimental remote-render RGB-D sessions

Remote-render sessions use the regular H.264 video track for color and the
`horus.remote-render.v1` data channel for matching metric depth, source pose,
and projection metadata. This carrier is retained for further development;
the SDK defaults to its validated ROS-compressed RGB-D carrier. The bridge:

- subscribes to both source topics with best-effort, keep-last-one QoS;
- fragments large depth frames into messages below the negotiated SCTP limit;
- drops a complete stale depth frame before transmission when the data-channel
  queue is already full;
- keeps color RTP timestamps tied to actual GStreamer presentation time instead
  of assuming a constant source cadence.

The Quest pairs color and depth by frame sequence. Missing or incomplete depth
chunks therefore discard one remote frame without corrupting the next frame.

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
- periodic RTP counters,
- remote-render auxiliary input, sent, and dropped frame counters.

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
| Color arrives but no spatial map | auxiliary depth path | data-channel open state + auxiliary sent/dropped counters |

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

- `horus_unity_bridge_tests` — 11 gtest translation units covering the HorusLink protocol,
  channel table, connection manager, control messages, light codecs, outbound frame router,
  outbound lane queue, session, remote-render auxiliary data, RTCP NACK handling and the bridge
  node. Wire-format expectations are pinned against
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
