// Copyright 2026 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "horus_unity_bridge/horuslink_channel_table.hpp"
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <memory>
#include <string>

namespace horus_unity_bridge
{
// Independent of video sessions. Only signaling and status use ROS at a relay.
class ControlLink
{
public:
  using Apply = std::function<bool(int, const horuslink::ChannelDescriptor &,
      const std::vector<uint8_t> &, uint8_t)>;
  using Disconnect = std::function<void(int)>;
  using Snapshot = std::function<void(const std::string &)>;
  using Catalog = std::function<std::vector<std::string>()>;
  ControlLink(rclcpp::Node * node, const std::string & mode, Apply apply, Disconnect disconnect,
    Snapshot snapshot = {}, Catalog catalog = {});
  ~ControlLink();
  bool is_proxy() const;
  bool handles(const std::string & topic) const;
  bool forward(int client, const horuslink::ChannelDescriptor & channel,
    const std::vector<uint8_t> & payload, uint8_t flags, int64_t remaining_input_ms = 200);
  void disconnected(int client);
  void publish_lease_state(const std::string & snapshot);
  void tick();
  nlohmann::json statistics() const;

private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
};
}  // namespace horus_unity_bridge
