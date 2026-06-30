// Copyright 2025 RICE Lab, University of Genoa
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

// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "connection_manager.hpp"
#include "horus_unity_bridge/horuslink_connection_manager.hpp"
#include "message_router.hpp"

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <atomic>

namespace horus_unity_bridge
{

/**
 * @brief Main HORUS Unity Bridge node
 *
 * Integrates connection management and message routing for high-performance
 * Unity-ROS2 communication.
 *
 * Key improvements over ROS-TCP-Endpoint:
 * - Single-node architecture (vs one node per topic)
 * - Epoll-based async I/O (vs Python threading)
 * - Zero-copy optimizations
 * - Configurable performance tuning
 * - Lower latency (~10x improvement expected)
 * - Higher throughput (~5x improvement expected)
 */
class UnityBridgeNode
{
public:
  enum class TransportProtocol
  {
    LegacyConnector,
    HorusLink
  };

  explicit UnityBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~UnityBridgeNode();

  /**
   * @brief Initialize and start the bridge
   */
  bool start();

  /**
   * @brief Stop the bridge and cleanup
   */
  void stop();

  /**
   * @brief Run the bridge (blocking until stopped)
   */
  void run();

  /**
   * @brief Check if bridge is running
   */
  bool is_running() const {return running_.load();}
  const ConnectionManager::Config & connection_config() const {return conn_config_;}
  uint16_t port() const {return conn_config_.port;}
  uint16_t horuslink_bulk_port() const {return horuslink_config_.bulk_port;}
  const std::string & bind_address() const {return conn_config_.bind_address;}
  TransportProtocol transport_protocol() const {return transport_protocol_;}
  const std::string & transport_protocol_name() const {return transport_protocol_name_;}
  int worker_threads() const {return worker_threads_;}
  bool log_protocol_messages() const {return log_protocol_messages_;}
  bool has_service_timer() const {return static_cast<bool>(service_timer_);}
  const std::vector<std::string> & reserved_parameter_warnings() const
  {
    return reserved_parameter_warnings_;
  }

  /**
   * @brief Print comprehensive statistics
   */
  void print_statistics() const;

private:
  // Core components
  std::shared_ptr<MessageRouter> router_;
  std::unique_ptr<ConnectionManager> connection_manager_;
  std::unique_ptr<horuslink::HorusLinkConnectionManager> horuslink_connection_manager_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;

  // Configuration
  ConnectionManager::Config conn_config_;
  horuslink::HorusLinkConnectionManager::Config horuslink_config_;
  TransportProtocol transport_protocol_ = TransportProtocol::HorusLink;
  std::string transport_protocol_name_ = "horuslink";
  int worker_threads_ = 4;
  bool log_protocol_messages_ = true;
  std::vector<std::string> reserved_parameter_warnings_;
  rclcpp::NodeOptions node_options_;

  // State
  std::atomic<bool> running_;
  std::thread spin_thread_;

  // Private methods
  void load_parameters();
  void setup_callbacks();
  void setup_legacy_callbacks();
  void setup_horuslink_callbacks();
  void handle_message_from_unity(int client_fd, const ProtocolMessage & message);
  void handle_client_connected(int client_fd, const std::string & ip, uint16_t port);
  void handle_client_disconnected(int client_fd, const std::string & ip, uint16_t port);
  void handle_horuslink_client_connected(int connection_id, const std::string & ip);
  void handle_horuslink_client_disconnected(int connection_id, const std::string & ip);
  bool handle_horuslink_subscribe(
    int connection_id,
    const horuslink::ChannelDescriptor & channel,
    std::string & error);
  bool handle_horuslink_publisher(
    int connection_id,
    const horuslink::ChannelDescriptor & channel,
    std::string & error);
  bool handle_horuslink_ros_service(
    int connection_id,
    const horuslink::ChannelDescriptor & channel,
    std::string & error);
  void handle_horuslink_channel_close(
    int connection_id,
    horuslink::ChannelRegistrationKind registration_kind,
    const horuslink::ChannelDescriptor & channel);
  void handle_horuslink_frame(
    int connection_id,
    horuslink::Lane lane,
    const horuslink::ChannelDescriptor & channel,
    const horuslink::Frame & frame);

  // Statistics timer
  rclcpp::TimerBase::SharedPtr stats_timer_;
  rclcpp::TimerBase::SharedPtr service_timer_;
  void stats_timer_callback();
};

} // namespace horus_unity_bridge
