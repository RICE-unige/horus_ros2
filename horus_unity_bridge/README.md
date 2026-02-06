# HORUS Unity Bridge

High-performance C++ ROS 2 node inspired by the **Unity ROS-TCP-Endpoint**.  
It is built to be a drop-in replacement that works natively with the **Unity ROS-TCP-Connector** package on Unity.

One process hosts the epoll-based TCP bridge, topic router, and service manager, delivering sub-millisecond routing while preserving the original Unity protocol semantics.

## Prerequisites

- ROS 2 Humble or Jazzy
- `nlohmann-json3-dev`
- WebRTC build dependencies (Linux bridge host):
  - `pkg-config`
  - `libgstreamer1.0-dev`
  - `libgstreamer-plugins-base1.0-dev`
  - `libopencv-dev`
- Internet access on first configure (CMake fetches `libdatachannel` automatically)

## Quick Start

```bash
# Build inside the HORUS ROS 2 workspace
cd ~/horus_ros2
colcon build --packages-select horus_unity_bridge --cmake-args -DENABLE_WEBRTC=ON
source install/setup.bash

# Run the bridge (listens on tcp/10000 by default)
ros2 launch horus_unity_bridge unity_bridge.launch.py
```

Supply parameters through `config/bridge_config.yaml` or `--ros-args`.  
To build without WebRTC, use:

```bash
colcon build --packages-select horus_unity_bridge --cmake-args -DENABLE_WEBRTC=OFF
```

Important knobs: `tcp_ip`, `tcp_port`, socket buffer sizes, queue depth, worker thread count, and default QoS profiles.

## Using the Bridge

Unity continues to use the standard ROS‑TCP‑Connector API:

```csharp
ROSConnection.GetOrCreateInstance().Connect("bridge-ip", 10000);
ROSConnection.GetOrCreateInstance().Subscribe<PoseMsg>("/robot/pose", callback);
```

System commands (`__subscribe`, `__publish`, `__ros_service`, etc.), binary framing, and keep‑alives all match the upstream protocol.  
Topic data is published via ROS generic publishers/subscriptions and services are proxied through the typed `ServiceManager`.

## Monitoring & Operations

- Structured logs list connections, per‑topic registrations, and every 30 s statistics covering TCP traffic, routed messages, and service counts.
- To change runtime settings, edit `config/bridge_config.yaml` and relaunch (hot reload is not yet supported).
- If the port is busy, free it via `fuser -k 10000/tcp`.

## Testing

Build the optional `horus_unity_bridge_test` package to access ROS publishers, service servers, and Unity client simulators:

```bash
colcon build --packages-select horus_unity_bridge horus_unity_bridge_test --cmake-args -DENABLE_WEBRTC=ON
source install/setup.bash

# Example
ros2 launch horus_unity_bridge unity_bridge.launch.py &
ros2 run horus_unity_bridge_test test_publisher
ros2 run horus_unity_bridge_test unity_client_simulator
```

Use `colcon test --packages-select horus_unity_bridge` to exercise C++ unit/integration tests.

## License

Apache-2.0 (same as the rest of HORUS SDK).

## Research Lab

Developed at **RICE Lab** (Robots and Intelligent Systems for Citizens and the Environment)
University of Genoa, Italy
