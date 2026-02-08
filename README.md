# HORUS ROS 2 Infrastructure

```
██╗  ██╗ ██████╗ ██████╗ ██╗   ██╗███████╗
██║  ██║██╔═══██╗██╔══██╗██║   ██║██╔════╝
███████║██║   ██║██████╔╝██║   ██║███████╗
██╔══██║██║   ██║██╔══██╗██║   ██║╚════██║
██║  ██║╚██████╔╝██║  ██║╚██████╔╝███████║
╚═╝  ╚═╝ ╚═════╝ ╚═╝  ╚═╝ ╚═════╝ ╚══════╝
```

**Mixed Reality Robot Management Backend**

This repository contains the ROS 2 packages for the HORUS system, managing the bridge between ROS 2 robots and the Meta Quest 3 Mixed Reality application.

## 📦 Packages

- **`horus_unity_bridge`**: TCP server node inspired by `ROS-TCP-Endpoint`, optimized for C++ and fully compatible with the Unity `ROS-TCP-Connector` (Unity connects as a client).
- **`horus_interfaces`**: Custom ROS 2 message and service definitions (`RobotConfig`, `RobotStatus`, etc.).
- **`horus_backend`**: Core backend logic for robot registration and state management.
- **`horus_unity_bridge_test`**: Testing utilities and simulation mocks.

## 🏗️ Architecture

```
HORUS MR App (Unity, TCP client)
    ↑ (TCP 10000)
HORUS Unity Bridge (ROS 2 TCP server)
    ↔ (Topics / Services)
HORUS Backend Node
    ↔ (Registration / State)
HORUS SDK (Robot/PC)
```

## 🚀 Building

### Prerequisites
- ROS 2 Humble (or Jazzy)
- `nlohmann-json3-dev`

### Build Instructions

```bash
# Clone the repository
git clone https://github.com/RICE-unige/horus_ros2.git
cd horus_ros2

# Install dependencies
rosdep install --from-paths . -y --ignore-src

# Build
colcon build --symlink-install

# Source
source install/setup.bash
```

## 🛠️ Usage

### Standard Launch

This brings up the Bridge and Backend nodes, ready to accept connections from the HORUS SDK and MR App.

```bash
ros2 launch horus_unity_bridge unity_bridge.launch.py
```

### WebRTC Camera Streaming Notes

- The Unity bridge WebRTC path negotiates outgoing video track `MID` and H264 payload type directly from the Unity offer before applying the remote description.
- `ready` signaling is published only after offer negotiation succeeds; negotiation failures are returned as `type="error"` signaling payloads for deterministic client handling.
- Bridge logs include media-path telemetry for faster diagnosis:
  - negotiated offer values (`mid` + selected H264 payload type),
  - first outbound RTP packet header (`pt`/`ssrc`/`seq`),
  - periodic RTP sent counters.

These additions specifically target the common "ICE connected but white video panel" failure mode by ensuring SDP and RTP payload alignment end-to-end.

### Network Ports

- **10000**: Main Communication Port (SDK Client & MR App)

## 🤝 Integration

The **HORUS SDK** (`python`, `cpp`, `rust`) interacts with this backend to register robots.
Typically, the SDK Client will automatically attempt to launch this bridge if it is installed and sourced.

```bash
# Example manual start from SDK Repo
ros2 launch horus_unity_bridge unity_bridge.launch.py
```

## 🏛️ Research Lab

Developed at **RICE Lab** (Robots and Intelligent Systems for Citizens and the Environment)
University of Genoa, Italy

## 🔗 Links

- [HORUS SDK (Main Repo)](https://github.com/RICE-unige/horus_sdk)
- [Unity ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector) (Unity client compatible with this bridge)
