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

  horuslink_connection_manager_ =
    std::make_unique<horuslink::HorusLinkConnectionManager>(horuslink_config_);

  // Setup callbacks
  setup_callbacks();

  // Create executor
  rclcpp::ExecutorOptions executor_options;
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
    executor_options, worker_threads_, false
  );
  executor_->add_node(router_);

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
  horuslink_config_.bind_address = router_->declare_parameter<std::string>("tcp_ip", "0.0.0.0");
  horuslink_config_.realtime_port =
    static_cast<uint16_t>(router_->declare_parameter<int>("tcp_port", 10000));
  horuslink_config_.max_connections = router_->declare_parameter<int>("max_connections", 10);
  horuslink_config_.socket_buffer_size = static_cast<size_t>(
    router_->declare_parameter<int>("socket_buffer_size", 65536));
  horuslink_config_.tcp_nodelay = router_->declare_parameter<bool>("tcp_nodelay", true);

  transport_protocol_name_ = normalize_transport_protocol(
    router_->declare_parameter<std::string>("transport_protocol", "horuslink"));
  if (transport_protocol_name_ == "horuslink") {
    transport_protocol_ = TransportProtocol::HorusLink;
  } else {
    RCLCPP_WARN(
      router_->get_logger(),
      "Unknown transport_protocol '%s'; falling back to horuslink mode",
      transport_protocol_name_.c_str());
    transport_protocol_name_ = "horuslink";
    transport_protocol_ = TransportProtocol::HorusLink;
  }

  horuslink_config_.bulk_port = static_cast<uint16_t>(
    router_->declare_parameter<int>("horuslink_bulk_port", 10001));
  const auto configured_horuslink_max_payload_size = router_->declare_parameter<int>(
    "horuslink_max_payload_size",
    static_cast<int>(horuslink_config_.max_payload_size));
  horuslink_config_.max_payload_size = static_cast<size_t>(
    configured_horuslink_max_payload_size < 1 ? 1 : configured_horuslink_max_payload_size);
  const auto configured_horuslink_keepalive_ms = router_->declare_parameter<int>(
    "horuslink_keepalive_ms",
    static_cast<int>(horuslink_config_.keepalive_ms));
  horuslink_config_.keepalive_ms = static_cast<uint32_t>(
    configured_horuslink_keepalive_ms < 0 ? 0 : configured_horuslink_keepalive_ms);

  const auto configured_worker_threads = router_->declare_parameter<int>("worker_threads", 4);
  worker_threads_ = std::max<int>(1, static_cast<int>(configured_worker_threads));
  log_protocol_messages_ = router_->declare_parameter<bool>("log_protocol_messages", true);
  router_->set_log_protocol_messages(log_protocol_messages_);
}

void UnityBridgeNode::setup_callbacks()
{
  setup_horuslink_callbacks();
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

  horuslink_connection_manager_->set_service_client_callback(
    [this](
      int connection_id,
      const horuslink::ChannelDescriptor & channel,
      std::string & error) {
      return handle_horuslink_ros_service(connection_id, channel, error);
    });

  horuslink_connection_manager_->set_channel_close_callback(
    [this](
      int connection_id,
      horuslink::ChannelRegistrationKind registration_kind,
      const horuslink::ChannelDescriptor & channel) {
      handle_horuslink_channel_close(connection_id, registration_kind, channel);
    });

  horuslink_connection_manager_->set_topic_table_callback(
    [this](int) {
      return router_->get_horuslink_topic_table();
    });

  horuslink_connection_manager_->set_post_subscribe_callback(
    [this](int connection_id, const std::string & topic) {
      router_->replay_retained_payloads(connection_id, topic);
    });

  router_->set_send_callback(
    [this](int connection_id,
    const std::string & topic,
    const std::vector<uint8_t> & payload) {
      return horuslink_connection_manager_->send_to_topic(connection_id, topic, payload);
    });

  router_->set_horuslink_service_response_callback(
    [this](
      int connection_id,
      const std::string & service_name,
      uint32_t corr_id,
      const std::vector<uint8_t> & response) {
      return horuslink_connection_manager_->send_service_response(
        connection_id,
        service_name,
        corr_id,
        response);
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

  const bool connection_started = horuslink_connection_manager_->start();
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

bool UnityBridgeNode::handle_horuslink_ros_service(
  int connection_id,
  const horuslink::ChannelDescriptor & channel,
  std::string & error)
{
  if (!router_->register_horuslink_ros_service(connection_id, channel)) {
    error = "failed to register ROS service";
    return false;
  }

  return true;
}

void UnityBridgeNode::handle_horuslink_channel_close(
  int connection_id,
  horuslink::ChannelRegistrationKind registration_kind,
  const horuslink::ChannelDescriptor & channel)
{
  bool success = false;
  switch (registration_kind) {
    case horuslink::ChannelRegistrationKind::Subscriber:
      success = router_->unregister_horuslink_subscriber(connection_id, channel);
      break;
    case horuslink::ChannelRegistrationKind::Publisher:
      success = router_->unregister_horuslink_publisher(connection_id, channel);
      break;
    case horuslink::ChannelRegistrationKind::ServiceClient:
      success = router_->unregister_horuslink_ros_service(connection_id, channel);
      break;
  }

  if (!success) {
    RCLCPP_WARN(
      router_->get_logger(),
      "HorusLink channel close cleanup ignored: connection=%d channel=%u topic=%s",
      connection_id,
      channel.channel_id,
      channel.topic.c_str());
  }
}

void UnityBridgeNode::handle_horuslink_frame(
  int connection_id,
  horuslink::Lane,
  const horuslink::ChannelDescriptor & channel,
  const horuslink::Frame & frame)
{
  if (frame.header.msg_type == horuslink::MessageType::ServiceRequest) {
    router_->route_horuslink_service_request(
      connection_id,
      channel,
      frame.header.corr_id,
      frame.payload);
    return;
  }

  if (frame.header.msg_type != horuslink::MessageType::Data) {
    return;
  }

  router_->route_horuslink_data_frame(
    connection_id,
    channel,
    frame.payload,
    frame.header.flags);
}

void UnityBridgeNode::stats_timer_callback()
{
  print_statistics();
}

void UnityBridgeNode::print_statistics() const
{
  auto router_stats = router_->get_statistics();
  auto topic_stats = router_->get_topic_manager().get_statistics();
  auto service_stats = router_->get_service_manager().get_statistics();

  RCLCPP_INFO(router_->get_logger(), "=== Unity Bridge Statistics ===");
  RCLCPP_INFO(
    router_->get_logger(),
    "Transport: HorusLink, Connections: %lu active",
    horuslink_connection_manager_ ? horuslink_connection_manager_->connection_count() : 0u);
  RCLCPP_INFO(router_->get_logger(), "Topics: %lu publishers, %lu subscribers",
              topic_stats.active_publishers, topic_stats.active_subscribers);
  RCLCPP_INFO(router_->get_logger(), "Router: %lu routed, %lu published, %lu commands",
              router_stats.messages_routed, router_stats.messages_published,
              router_stats.system_commands_processed);
  RCLCPP_INFO(router_->get_logger(),
      "Services: %lu ROS calls, %lu Unity calls, %lu responses, %lu timeouts, %lu errors",
              service_stats.ros_services_called, service_stats.unity_services_called,
              service_stats.responses_sent, service_stats.timeouts, service_stats.errors);

  if (router_stats.routing_errors > 0) {
    RCLCPP_WARN(router_->get_logger(), "Errors: %lu routing", router_stats.routing_errors);
  }
}

} // namespace horus_unity_bridge
