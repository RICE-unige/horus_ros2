// Copyright 2026 RICE Lab, University of Genoa
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>
#include <functional>
#include <memory>
#include <string>
#include <cstdint>

namespace horus_unity_bridge
{
// ROS is used only for offer/answer/ICE metadata. RTP never enters DDS.
class NativeMediaRelay
{
public:
  using PacketCallback = std::function<void(const uint8_t *, size_t, uint64_t)>;
  static nlohmann::json validate_config(const nlohmann::json & config);
  NativeMediaRelay(rclcpp::Node * node, const std::string & config_path);
  ~NativeMediaRelay();
  bool contains(const std::string & topic) const;
  bool subscribe(const std::string & topic, const std::string & consumer, PacketCallback callback);
  void unsubscribe(const std::string & topic, const std::string & consumer);
  void request_keyframe(const std::string & topic);
  void update_profile(const std::string & topic, const std::string & consumer,
    int bitrate_kbps, int framerate, int priority = 1);
  void update_capacity(const std::string & topic, const std::string & consumer, int bitrate_kbps);
  void tick();
  nlohmann::json statistics() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace horus_unity_bridge
