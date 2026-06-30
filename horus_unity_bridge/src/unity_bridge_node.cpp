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

// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#include "horus_unity_bridge/unity_bridge_node.hpp"
#include <algorithm>
#include <iostream>
#include <chrono>
#include <cctype>
#include <sstream>

namespace horus_unity_bridge
{

namespace
{

std::string normalize_transport_protocol(std::string value)
{
  std::transform(
    value.begin(),
    value.end(),
    value.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return value;
}

}  // namespace

UnityBridgeNode::UnityBridgeNode(const rclcpp::NodeOptions & options)
: node_options_(options),
  running_(false)
{
  // Initialize ROS if not already done
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  // Create router (ROS node)
  router_ = std::make_shared<MessageRouter>(node_options_);

  // Load parameters
  load_parameters();

  // Create connection manager
  if (transport_protocol_ == TransportProtocol::HorusLink) {
    horuslink_connection_manager_ =
      std::make_unique<horuslink::HorusLinkConnectionManager>(horuslink_config_);
  } else {
    connection_manager_ = std::make_unique<ConnectionManager>(conn_config_);
  }

  // Setup callbacks
  setup_callbacks();

  // Create executor
  rclcpp::ExecutorOptions executor_options;
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
    executor_options, worker_threads_, false
  );
  executor_->add_node(router_);

  if (!reserved_parameter_warnings_.empty()) {
    std::ostringstream warning_stream;
    warning_stream << "Reserved bridge parameters currently ignored: ";
    for (size_t i = 0; i < reserved_parameter_warnings_.size(); ++i) {
      if (i > 0) {
        warning_stream << ", ";
      }
      warning_stream << reserved_parameter_warnings_[i];
    }
    RCLCPP_WARN(router_->get_logger(), "%s", warning_stream.str().c_str());
  }

  RCLCPP_INFO(
    router_->get_logger(),
    "Unity Bridge Node initialized (transport=%s)",
    transport_protocol_name_.c_str());
}

UnityBridgeNode::~UnityBridgeNode()
{
  stop();
}

void UnityBridgeNode::load_parameters()
{
  conn_config_.bind_address = router_->declare_parameter<std::string>("tcp_ip", "0.0.0.0");
  conn_config_.port = static_cast<uint16_t>(router_->declare_parameter<int>("tcp_port", 10000));
  conn_config_.max_connections = router_->declare_parameter<int>("max_connections", 10);
  conn_config_.socket_buffer_size = static_cast<size_t>(
    router_->declare_parameter<int>("socket_buffer_size", 65536));
  conn_config_.message_queue_size = static_cast<size_t>(
    router_->declare_parameter<int>("message_queue_size", 1000));
  conn_config_.tcp_nodelay = router_->declare_parameter<bool>("tcp_nodelay", true);
  conn_config_.connection_timeout_ms = router_->declare_parameter<int>("connection_timeout_ms",
      5000);

  transport_protocol_name_ = normalize_transport_protocol(
    router_->declare_parameter<std::string>("transport_protocol", "legacy"));
  const bool legacy_transport_requested = transport_protocol_name_ == "legacy" ||
    transport_protocol_name_ == "connector" ||
    transport_protocol_name_ == "ros_tcp_connector";
  if (transport_protocol_name_ == "horuslink") {
    transport_protocol_ = TransportProtocol::HorusLink;
  } else if (legacy_transport_requested) {
    transport_protocol_name_ = "legacy";
    transport_protocol_ = TransportProtocol::LegacyConnector;
  } else {
    RCLCPP_WARN(
      router_->get_logger(),
      "Unknown transport_protocol '%s'; falling back to legacy connector mode",
      transport_protocol_name_.c_str());
    transport_protocol_name_ = "legacy";
    transport_protocol_ = TransportProtocol::LegacyConnector;
  }

  horuslink_config_.bind_address = conn_config_.bind_address;
  horuslink_config_.realtime_port = conn_config_.port;
  horuslink_config_.bulk_port = static_cast<uint16_t>(
    router_->declare_parameter<int>("horuslink_bulk_port", 10001));
  horuslink_config_.max_connections = conn_config_.max_connections;
  horuslink_config_.socket_buffer_size = conn_config_.socket_buffer_size;
  horuslink_config_.tcp_nodelay = conn_config_.tcp_nodelay;

  const auto configured_worker_threads = router_->declare_parameter<int>("worker_threads", 4);
  worker_threads_ = std::max<int>(1, static_cast<int>(configured_worker_threads));
  log_protocol_messages_ = router_->declare_parameter<bool>("log_protocol_messages", true);
  router_->set_log_protocol_messages(log_protocol_messages_);

  reserved_parameter_warnings_.clear();
  auto add_reserved_warning = [this](const std::string & name) {
      if (std::find(
          reserved_parameter_warnings_.begin(),
          reserved_parameter_warnings_.end(),
          name) == reserved_parameter_warnings_.end())
      {
        reserved_parameter_warnings_.push_back(name);
      }
    };

  router_->declare_parameter<int>("message_pool_size", 10000);
  router_->declare_parameter<bool>("enable_zero_copy", true);
  add_reserved_warning("message_pool_size");
  add_reserved_warning("enable_zero_copy");

  const std::string publisher_reliability = router_->declare_parameter<std::string>(
    "default_publisher_qos.reliability", "reliable");
  const std::string publisher_durability = router_->declare_parameter<std::string>(
    "default_publisher_qos.durability", "volatile");
  const std::string publisher_history = router_->declare_parameter<std::string>(
    "default_publisher_qos.history", "keep_last");
  const int publisher_depth = router_->declare_parameter<int>("default_publisher_qos.depth", 10);
  (void)publisher_reliability;
  (void)publisher_durability;
  (void)publisher_history;
  (void)publisher_depth;
  add_reserved_warning("default_publisher_qos.*");

  const std::string subscriber_reliability = router_->declare_parameter<std::string>(
    "default_subscriber_qos.reliability", "reliable");
  const std::string subscriber_durability = router_->declare_parameter<std::string>(
    "default_subscriber_qos.durability", "volatile");
  const std::string subscriber_history = router_->declare_parameter<std::string>(
    "default_subscriber_qos.history", "keep_last");
  const int subscriber_depth = router_->declare_parameter<int>("default_subscriber_qos.depth", 10);
  (void)subscriber_reliability;
  (void)subscriber_durability;
  (void)subscriber_history;
  (void)subscriber_depth;
  add_reserved_warning("default_subscriber_qos.*");
}

void UnityBridgeNode::setup_callbacks()
{
  if (transport_protocol_ == TransportProtocol::HorusLink) {
    setup_horuslink_callbacks();
  } else {
    setup_legacy_callbacks();
  }
}

void UnityBridgeNode::setup_legacy_callbacks()
{
  // Message from Unity -> Router
  connection_manager_->set_message_callback(
    [this](int fd, const ProtocolMessage & msg) {
      handle_message_from_unity(fd, msg);
    }
  );

  // Router wants to send to Unity -> Connection Manager
  router_->set_send_callback(
    [this](int fd,
    const std::string & dest,
    const std::vector<uint8_t> & payload,
    OutboundMessagePolicy policy) {
      return connection_manager_->send_to_client(fd, dest, payload, policy);
    }
  );

  router_->set_broadcast_callback(
    [this](const std::string & dest,
    const std::vector<uint8_t> & payload,
    OutboundMessagePolicy policy) {
      connection_manager_->broadcast_message(dest, payload, policy);
    }
  );

  // Connection events
  connection_manager_->set_connection_callback(
    [this](int fd, const std::string & ip, uint16_t port) {
      handle_client_connected(fd, ip, port);
    }
  );

  connection_manager_->set_disconnection_callback(
    [this](int fd, const std::string & ip, uint16_t port) {
      handle_client_disconnected(fd, ip, port);
    }
  );
}

void UnityBridgeNode::setup_horuslink_callbacks()
{
  horuslink_connection_manager_->set_frame_callback(
    [this](
      int connection_id,
      horuslink::Lane lane,
      const horuslink::ChannelDescriptor & channel,
      const horuslink::Frame & frame) {
      handle_horuslink_frame(connection_id, lane, channel, frame);
    });

  horuslink_connection_manager_->set_subscribe_callback(
    [this](
      int connection_id,
      const horuslink::ChannelDescriptor & channel,
      std::string & error) {
      return handle_horuslink_subscribe(connection_id, channel, error);
    });

  horuslink_connection_manager_->set_publisher_callback(
    [this](
      int connection_id,
      const horuslink::ChannelDescriptor & channel,
      std::string & error) {
      return handle_horuslink_publisher(connection_id, channel, error);
    });

  router_->set_send_callback(
    [this](int connection_id,
    const std::string & topic,
    const std::vector<uint8_t> & payload,
    OutboundMessagePolicy) {
      return horuslink_connection_manager_->send_to_topic(connection_id, topic, payload);
    });

  router_->set_broadcast_callback(
    [this](const std::string & topic,
    const std::vector<uint8_t> & payload,
    OutboundMessagePolicy) {
      horuslink_connection_manager_->broadcast_to_topic(topic, payload);
    });

  horuslink_connection_manager_->set_connection_callback(
    [this](int connection_id, const std::string & ip) {
      handle_horuslink_client_connected(connection_id, ip);
    });

  horuslink_connection_manager_->set_disconnection_callback(
    [this](int connection_id, const std::string & ip) {
      handle_horuslink_client_disconnected(connection_id, ip);
    });
}

bool UnityBridgeNode::start()
{
  if (running_.load()) {
    return true;
  }

  const bool connection_started = transport_protocol_ == TransportProtocol::HorusLink ?
    horuslink_connection_manager_->start() :
    connection_manager_->start();
  if (!connection_started) {
    RCLCPP_ERROR(router_->get_logger(), "Failed to start connection manager");
    return false;
  }

  running_ = true;

  // Start ROS executor in separate thread
  spin_thread_ = std::thread([this]() {
        RCLCPP_INFO(router_->get_logger(), "ROS executor thread started");
        executor_->spin();
        RCLCPP_INFO(router_->get_logger(), "ROS executor thread stopped");
  });

  // Create statistics timer
  stats_timer_ = router_->create_wall_timer(
    std::chrono::seconds(30),
    [this]() {stats_timer_callback();}
  );

  // Create service response processing timer (1ms for low latency)
  service_timer_ = router_->create_wall_timer(
    std::chrono::milliseconds(1),
    [this]() {router_->get_service_manager().process_pending_responses();}
  );

  RCLCPP_INFO(router_->get_logger(), "Unity Bridge started successfully");

  return true;
}

void UnityBridgeNode::stop()
{
  if (!running_.load()) {
    return;
  }

  RCLCPP_INFO(router_->get_logger(), "Stopping Unity Bridge...");

  running_ = false;

  // Stop statistics timer
  if (stats_timer_) {
    stats_timer_->cancel();
  }
  if (service_timer_) {
    service_timer_->cancel();
  }

  if (horuslink_connection_manager_) {
    horuslink_connection_manager_->stop();
  }
  if (connection_manager_) {
    connection_manager_->stop();
  }

  // Stop ROS executor
  if (executor_) {
    executor_->cancel();
  }

  // Wait for spin thread
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }

  RCLCPP_INFO(router_->get_logger(), "Unity Bridge stopped");
}

void UnityBridgeNode::run()
{
  // Wait for shutdown signal
  while (running_.load() && rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void UnityBridgeNode::handle_message_from_unity(int client_fd, const ProtocolMessage & message)
{
  // Route message through the router
  router_->route_message(client_fd, message);
}

void UnityBridgeNode::handle_client_connected(int client_fd, const std::string & ip, uint16_t port)
{
  RCLCPP_INFO(router_->get_logger(), "Unity client connected: %s:%u (fd=%d)",
              ip.c_str(), port, client_fd);
  router_->send_handshake(client_fd);
}

void UnityBridgeNode::handle_client_disconnected(
  int client_fd, const std::string & ip,
  uint16_t port)
{
  RCLCPP_INFO(router_->get_logger(), "Unity client disconnected: %s:%u (fd=%d)",
              ip.c_str(), port, client_fd);
  if (router_) {
    router_->on_client_disconnected(client_fd);
  }
}

void UnityBridgeNode::handle_horuslink_client_connected(
  int connection_id,
  const std::string & ip)
{
  RCLCPP_INFO(
    router_->get_logger(),
    "HorusLink client connected: %s (connection=%d)",
    ip.c_str(),
    connection_id);
}

void UnityBridgeNode::handle_horuslink_client_disconnected(
  int connection_id,
  const std::string & ip)
{
  RCLCPP_INFO(
    router_->get_logger(),
    "HorusLink client disconnected: %s (connection=%d)",
    ip.c_str(),
    connection_id);
  if (router_) {
    router_->on_client_disconnected(connection_id);
  }
}

bool UnityBridgeNode::handle_horuslink_subscribe(
  int connection_id,
  const horuslink::ChannelDescriptor & channel,
  std::string & error)
{
  if (!router_->register_horuslink_subscriber(connection_id, channel)) {
    error = "failed to register ROS subscriber";
    return false;
  }

  return true;
}

bool UnityBridgeNode::handle_horuslink_publisher(
  int connection_id,
  const horuslink::ChannelDescriptor & channel,
  std::string & error)
{
  if (!router_->register_horuslink_publisher(connection_id, channel)) {
    error = "failed to register ROS publisher";
    return false;
  }

  return true;
}

void UnityBridgeNode::handle_horuslink_frame(
  int connection_id,
  horuslink::Lane,
  const horuslink::ChannelDescriptor & channel,
  const horuslink::Frame & frame)
{
  ProtocolMessage message;
  message.destination = channel.topic;
  message.payload = frame.payload;
  handle_message_from_unity(connection_id, message);
}

void UnityBridgeNode::stats_timer_callback()
{
  print_statistics();
}

void UnityBridgeNode::print_statistics() const
{
  if (transport_protocol_ == TransportProtocol::HorusLink) {
    RCLCPP_INFO(router_->get_logger(), "=== Unity Bridge Statistics ===");
    RCLCPP_INFO(
      router_->get_logger(),
      "Transport: HorusLink, Connections: %lu active",
      horuslink_connection_manager_ ? horuslink_connection_manager_->connection_count() : 0u);
    auto router_stats = router_->get_statistics();
    auto topic_stats = router_->get_topic_manager().get_statistics();
    RCLCPP_INFO(
      router_->get_logger(),
      "Topics: %lu publishers, %lu subscribers",
      topic_stats.active_publishers,
      topic_stats.active_subscribers);
    RCLCPP_INFO(
      router_->get_logger(),
      "Router: %lu routed, %lu published, %lu commands",
      router_stats.messages_routed,
      router_stats.messages_published,
      router_stats.system_commands_processed);
    return;
  }

  auto conn_stats = connection_manager_->get_statistics();
  auto router_stats = router_->get_statistics();
  auto topic_stats = router_->get_topic_manager().get_statistics();
  auto service_stats = router_->get_service_manager().get_statistics();

  RCLCPP_INFO(router_->get_logger(), "=== Unity Bridge Statistics ===");
  RCLCPP_INFO(router_->get_logger(), "Connections: %lu active, %lu total",
              conn_stats.active_connections, conn_stats.total_connections);
  RCLCPP_INFO(router_->get_logger(), "Messages: %lu sent, %lu received",
              conn_stats.messages_sent, conn_stats.messages_received);
  RCLCPP_INFO(router_->get_logger(), "Bytes: %lu sent, %lu received",
              conn_stats.bytes_sent, conn_stats.bytes_received);
  RCLCPP_INFO(router_->get_logger(), "Topics: %lu publishers, %lu subscribers",
              topic_stats.active_publishers, topic_stats.active_subscribers);
  RCLCPP_INFO(router_->get_logger(), "Router: %lu routed, %lu published, %lu commands",
              router_stats.messages_routed, router_stats.messages_published,
              router_stats.system_commands_processed);
  RCLCPP_INFO(router_->get_logger(),
      "Services: %lu ROS calls, %lu Unity calls, %lu responses, %lu timeouts, %lu errors",
              service_stats.ros_services_called, service_stats.unity_services_called,
              service_stats.responses_sent, service_stats.timeouts, service_stats.errors);

  if (conn_stats.connection_errors > 0 || router_stats.routing_errors > 0) {
    RCLCPP_WARN(router_->get_logger(), "Errors: %lu connection, %lu routing",
                conn_stats.connection_errors, router_stats.routing_errors);
  }
}

} // namespace horus_unity_bridge
