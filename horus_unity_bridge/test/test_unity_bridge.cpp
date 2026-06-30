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

#include "horus_unity_bridge/connection_manager.hpp"
#include "horus_unity_bridge/horuslink_control_messages.hpp"
#include "horus_unity_bridge/horuslink_protocol.hpp"
#include "horus_unity_bridge/message_router.hpp"
#include "horus_unity_bridge/serialized_payload.hpp"
#include "horus_unity_bridge/unity_bridge_node.hpp"

#include <gtest/gtest.h>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <arpa/inet.h>
#include <array>
#include <chrono>
#include <cstring>
#include <poll.h>
#include <stdexcept>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

namespace horus_unity_bridge
{

namespace
{

class SocketHandle
{
public:
  explicit SocketHandle(int fd = -1)
  : fd_(fd)
  {
  }

  ~SocketHandle()
  {
    reset();
  }

  SocketHandle(const SocketHandle &) = delete;
  SocketHandle & operator=(const SocketHandle &) = delete;

  SocketHandle(SocketHandle && other) noexcept
  : fd_(other.fd_)
  {
    other.fd_ = -1;
  }

  SocketHandle & operator=(SocketHandle && other) noexcept
  {
    if (this != &other) {
      reset();
      fd_ = other.fd_;
      other.fd_ = -1;
    }
    return *this;
  }

  int get() const {return fd_;}

  void reset()
  {
    if (fd_ >= 0) {
      close(fd_);
      fd_ = -1;
    }
  }

private:
  int fd_ = -1;
};

SocketHandle connect_to_port(uint16_t port)
{
  SocketHandle socket_handle(socket(AF_INET, SOCK_STREAM, 0));
  EXPECT_GE(socket_handle.get(), 0);

  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(port);
  inet_pton(AF_INET, "127.0.0.1", &address.sin_addr);
  EXPECT_EQ(
    connect(socket_handle.get(), reinterpret_cast<sockaddr *>(&address), sizeof(address)),
    0);
  return socket_handle;
}

bool recv_exact_with_timeout(int fd, uint8_t * data, size_t size, int timeout_ms)
{
  size_t offset = 0;
  while (offset < size) {
    pollfd poll_fd{};
    poll_fd.fd = fd;
    poll_fd.events = POLLIN;
    if (poll(&poll_fd, 1, timeout_ms) <= 0) {
      return false;
    }
    const ssize_t received = recv(fd, data + offset, size - offset, 0);
    if (received <= 0) {
      return false;
    }
    offset += static_cast<size_t>(received);
  }

  return true;
}

bool recv_horuslink_frame(int fd, horuslink::Frame & frame, int timeout_ms = 2000)
{
  std::array<uint8_t, horuslink::FrameHeader::kSize> header_bytes{};
  if (!recv_exact_with_timeout(fd, header_bytes.data(), header_bytes.size(), timeout_ms)) {
    return false;
  }

  horuslink::FrameHeader header;
  if (!horuslink::decode_header(header_bytes.data(), header_bytes.size(), header)) {
    return false;
  }

  std::vector<uint8_t> payload(header.length);
  if (header.length > 0 &&
    !recv_exact_with_timeout(fd, payload.data(), payload.size(), timeout_ms))
  {
    return false;
  }

  frame = horuslink::Frame{header, std::move(payload)};
  return true;
}

void expect_bridge_hello(
  int fd,
  uint32_t expected_max_payload_bytes = 512u * 1024u * 1024u,
  uint32_t expected_keepalive_ms = 1000u)
{
  horuslink::Frame hello_frame;
  ASSERT_TRUE(recv_horuslink_frame(fd, hello_frame));
  ASSERT_EQ(hello_frame.header.channel_id, 0u);
  ASSERT_EQ(hello_frame.header.msg_type, horuslink::MessageType::Control);
  ASSERT_EQ(hello_frame.header.seq, 1u);
  ASSERT_EQ(hello_frame.header.corr_id, 0u);
  const auto hello = horuslink::decode_hello(
    hello_frame.payload.data(),
    hello_frame.payload.size());
  ASSERT_TRUE(hello.has_value());
  EXPECT_EQ(hello->role, horuslink::EndpointRole::Bridge);
  EXPECT_EQ(hello->max_payload_bytes, expected_max_payload_bytes);
  EXPECT_EQ(hello->keepalive_ms, expected_keepalive_ms);
}

void send_horuslink_frame(int fd, horuslink::Frame frame)
{
  const auto bytes = horuslink::serialize_frame(
    frame.header,
    frame.payload.data(),
    frame.payload.size());
  size_t offset = 0;
  while (offset < bytes.size()) {
    const ssize_t sent = send(fd, bytes.data() + offset, bytes.size() - offset, MSG_NOSIGNAL);
    ASSERT_GT(sent, 0);
    offset += static_cast<size_t>(sent);
  }
}

horuslink::Frame make_horuslink_control_frame(std::vector<uint8_t> payload, uint32_t seq = 1)
{
  horuslink::Frame frame;
  frame.header.channel_id = 0;
  frame.header.msg_type = horuslink::MessageType::Control;
  frame.header.seq = seq;
  frame.header.length = static_cast<uint32_t>(payload.size());
  frame.payload = std::move(payload);
  return frame;
}

horuslink::Frame make_horuslink_data_frame(
  uint16_t channel_id,
  std::vector<uint8_t> payload,
  uint32_t seq = 1,
  uint8_t flags = horuslink::FrameFlags::RawOpaque)
{
  horuslink::Frame frame;
  frame.header.channel_id = channel_id;
  frame.header.msg_type = horuslink::MessageType::Data;
  frame.header.flags = flags;
  frame.header.seq = seq;
  frame.header.length = static_cast<uint32_t>(payload.size());
  frame.payload = std::move(payload);
  return frame;
}

horuslink::Frame make_horuslink_service_request_frame(
  uint16_t channel_id,
  uint32_t corr_id,
  std::vector<uint8_t> payload,
  uint32_t seq = 1)
{
  horuslink::Frame frame;
  frame.header.channel_id = channel_id;
  frame.header.msg_type = horuslink::MessageType::ServiceRequest;
  frame.header.flags = horuslink::FrameFlags::RawOpaque;
  frame.header.seq = seq;
  frame.header.corr_id = corr_id;
  frame.header.length = static_cast<uint32_t>(payload.size());
  frame.payload = std::move(payload);
  return frame;
}

std::vector<uint8_t> serialize_string_message(const std::string & value)
{
  std_msgs::msg::String msg;
  msg.data = value;
  rclcpp::SerializedMessage serialized;
  rclcpp::Serialization<std_msgs::msg::String> serializer;
  serializer.serialize_message(&msg, &serialized);
  const auto & rcl_msg = serialized.get_rcl_serialized_message();
  return std::vector<uint8_t>(rcl_msg.buffer, rcl_msg.buffer + rcl_msg.buffer_length);
}

std::vector<uint8_t> serialize_add_two_ints_request(int64_t a, int64_t b)
{
  example_interfaces::srv::AddTwoInts::Request request;
  request.a = a;
  request.b = b;
  rclcpp::SerializedMessage serialized;
  rclcpp::Serialization<example_interfaces::srv::AddTwoInts::Request> serializer;
  serializer.serialize_message(&request, &serialized);
  const auto & rcl_msg = serialized.get_rcl_serialized_message();
  return std::vector<uint8_t>(rcl_msg.buffer, rcl_msg.buffer + rcl_msg.buffer_length);
}

example_interfaces::srv::AddTwoInts::Response deserialize_add_two_ints_response(
  const std::vector<uint8_t> & payload)
{
  if (payload.empty()) {
    throw std::runtime_error("empty AddTwoInts response payload");
  }

  std::vector<uint8_t> normalized_payload;
  const std::vector<uint8_t> * serialized_payload = &payload;
  if (!detail::has_cdr_header(payload)) {
    normalized_payload = detail::add_cdr_header(payload);
    serialized_payload = &normalized_payload;
  }

  rclcpp::SerializedMessage serialized;
  serialized.reserve(serialized_payload->size());
  auto & rcl_msg = serialized.get_rcl_serialized_message();
  std::memcpy(rcl_msg.buffer, serialized_payload->data(), serialized_payload->size());
  rcl_msg.buffer_length = serialized_payload->size();

  example_interfaces::srv::AddTwoInts::Response response;
  rclcpp::Serialization<example_interfaces::srv::AddTwoInts::Response> serializer;
  serializer.deserialize_message(&serialized, &response);
  return response;
}

}  // namespace

class MessageRouterTestPeer
{
public:
  static OutboundMessagePolicy classify_subscription_policy(
    const std::string & topic,
    const std::string & message_type)
  {
    return MessageRouter::classify_subscription_policy(topic, message_type);
  }

  static bool has_client_state(const MessageRouter & router, int client_fd)
  {
    std::lock_guard<std::mutex> lock(router.pending_service_state_mutex_);
    return router.pending_service_state_.find(client_fd) != router.pending_service_state_.end();
  }

  static MessageRouter::PendingServiceState client_state(
    const MessageRouter & router,
    int client_fd)
  {
    std::lock_guard<std::mutex> lock(router.pending_service_state_mutex_);
    auto it = router.pending_service_state_.find(client_fd);
    if (it == router.pending_service_state_.end()) {
      return {};
    }
    return it->second;
  }
};

class BridgeRuntimeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  static uint16_t find_unused_port()
  {
    const int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
      return 0;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;

    if (bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
      close(fd);
      return 0;
    }

    socklen_t len = sizeof(addr);
    if (getsockname(fd, reinterpret_cast<sockaddr *>(&addr), &len) != 0) {
      close(fd);
      return 0;
    }
    const uint16_t port = ntohs(addr.sin_port);
    close(fd);
    return port;
  }

  static std::vector<uint8_t> bytes(std::initializer_list<uint8_t> data)
  {
    return std::vector<uint8_t>(data);
  }
};

TEST_F(BridgeRuntimeTest, StrictQueueOverflowsDisconnectWhileReplaceableQueueCoalesces)
{
  OutboundMessageQueue queue;

  auto strict_first = queue.enqueue(
    "/horus/registration",
    bytes({0x01}),
    OutboundMessagePolicy::Strict,
    2);
  auto strict_second = queue.enqueue(
    "/horus/registration_ack",
    bytes({0x02}),
    OutboundMessagePolicy::Strict,
    2);
  auto strict_overflow = queue.enqueue(
    "/horus/heartbeat",
    bytes({0x03}),
    OutboundMessagePolicy::Strict,
    2);

  EXPECT_TRUE(strict_first.accepted);
  EXPECT_TRUE(strict_second.accepted);
  EXPECT_FALSE(strict_overflow.accepted);
  EXPECT_TRUE(strict_overflow.should_disconnect);
  EXPECT_EQ(queue.size(), 2u);

  OutboundMessageQueue replaceable_queue;
  auto first_frame = replaceable_queue.enqueue(
    "/camera/image",
    bytes({0x11}),
    OutboundMessagePolicy::Replaceable,
    2);
  auto replaced_frame = replaceable_queue.enqueue(
    "/camera/image",
    bytes({0x22}),
    OutboundMessagePolicy::Replaceable,
    2);

  EXPECT_TRUE(first_frame.accepted);
  EXPECT_TRUE(replaced_frame.accepted);
  EXPECT_TRUE(replaced_frame.replaced_existing);
  ASSERT_EQ(replaceable_queue.size(), 1u);
  auto front_destination = [&replaceable_queue]() {
      const auto & queue_view = static_cast<const OutboundMessageQueue &>(replaceable_queue);
      const auto * front = queue_view.front();
      return front == nullptr ? std::string() : front->destination;
    };
  auto front_first_byte = [&replaceable_queue]() {
      const auto & queue_view = static_cast<const OutboundMessageQueue &>(replaceable_queue);
      const auto * front = queue_view.front();
      return front == nullptr || front->serialized.empty() ? uint8_t{0} : front->serialized.front();
    };
  EXPECT_EQ(front_destination(), "/camera/image");
  EXPECT_EQ(front_first_byte(), 0x22);

  auto second_topic = replaceable_queue.enqueue(
    "/camera/right",
    bytes({0x33}),
    OutboundMessagePolicy::Replaceable,
    2);
  auto third_topic = replaceable_queue.enqueue(
    "/scan",
    bytes({0x44}),
    OutboundMessagePolicy::Replaceable,
    2);

  EXPECT_TRUE(second_topic.accepted);
  EXPECT_TRUE(third_topic.accepted);
  EXPECT_TRUE(third_topic.evicted_old_message);
  ASSERT_EQ(replaceable_queue.size(), 2u);
  EXPECT_EQ(front_destination(), "/camera/right");
}

TEST_F(BridgeRuntimeTest, ReplaceableFrontStaysSupersedableUntilBytesAreSent)
{
  OutboundMessageQueue queue;

  auto initial = queue.enqueue(
    "/camera/image",
    bytes({0x01, 0x02, 0x03}),
    OutboundMessagePolicy::Replaceable,
    4);
  ASSERT_TRUE(initial.accepted);
  ASSERT_TRUE(queue.front_is_replaceable());

  auto replaced = queue.enqueue(
    "/camera/image",
    bytes({0x04, 0x05, 0x06}),
    OutboundMessagePolicy::Replaceable,
    4);
  ASSERT_TRUE(replaced.accepted);
  ASSERT_TRUE(replaced.replaced_existing);
  ASSERT_EQ(queue.size(), 1u);
  ASSERT_NE(queue.front(), nullptr);
  EXPECT_EQ(queue.front()->serialized.front(), 0x04);
  EXPECT_TRUE(queue.front_is_replaceable());

  queue.mark_front_in_flight();
  EXPECT_FALSE(queue.front_is_replaceable());

  auto next_frame = queue.enqueue(
    "/camera/image",
    bytes({0x07, 0x08, 0x09}),
    OutboundMessagePolicy::Replaceable,
    4);
  ASSERT_TRUE(next_frame.accepted);
  ASSERT_EQ(queue.size(), 2u);
  ASSERT_NE(queue.front(), nullptr);
  EXPECT_EQ(queue.front()->serialized.front(), 0x04);

  EXPECT_FALSE(queue.advance_front(1));
  ASSERT_NE(queue.front(), nullptr);
  EXPECT_EQ(queue.front()->serialized.front(), 0x04);
}

TEST_F(BridgeRuntimeTest, SubscriptionPolicyIsStrictByDefaultAndReplaceableOnlyForKnownStreams)
{
  EXPECT_EQ(
    MessageRouterTestPeer::classify_subscription_policy(
      "/robot/goal_status",
      "std_msgs/String"),
    OutboundMessagePolicy::Strict);
  EXPECT_EQ(
    MessageRouterTestPeer::classify_subscription_policy(
      "/robot/waypoint_status",
      "std_msgs/msg/String"),
    OutboundMessagePolicy::Strict);
  EXPECT_EQ(
    MessageRouterTestPeer::classify_subscription_policy(
      "/camera/front/image",
      "sensor_msgs/Image"),
    OutboundMessagePolicy::Replaceable);
  EXPECT_EQ(
    MessageRouterTestPeer::classify_subscription_policy(
      "/scan",
      "sensor_msgs/msg/LaserScan"),
    OutboundMessagePolicy::Replaceable);
  EXPECT_EQ(
    MessageRouterTestPeer::classify_subscription_policy(
      "/horus/registration",
      "std_msgs/String"),
    OutboundMessagePolicy::Strict);

  // 3D-map payloads route to the Bulk lane so they yield to Realtime streams. Whole-map types are
  // BulkReplaceable (coalesce to newest); chunked map types are BulkStrict (preserve order).
  EXPECT_EQ(
    MessageRouterTestPeer::classify_subscription_policy(
      "/map_3d",
      "sensor_msgs/msg/PointCloud2"),
    OutboundMessagePolicy::BulkReplaceable);
  EXPECT_EQ(
    MessageRouterTestPeer::classify_subscription_policy(
      "/map",
      "nav_msgs/msg/OccupancyGrid"),
    OutboundMessagePolicy::BulkReplaceable);
  EXPECT_EQ(
    MessageRouterTestPeer::classify_subscription_policy(
      "/horus/map_mesh",
      "visualization_msgs/msg/Marker"),
    OutboundMessagePolicy::BulkStrict);
  EXPECT_EQ(
    MessageRouterTestPeer::classify_subscription_policy(
      "/horus/gaussian_splat/chunks",
      "std_msgs/String"),
    OutboundMessagePolicy::BulkStrict);
  EXPECT_EQ(
    MessageRouterTestPeer::classify_subscription_policy(
      "/map_3d/chunks_item",
      "std_msgs/String"),
    OutboundMessagePolicy::BulkStrict);
  EXPECT_EQ(
    MessageRouterTestPeer::classify_subscription_policy(
      "/horus/gaussian_splat/manifest",
      "std_msgs/String"),
    OutboundMessagePolicy::Strict);
}

TEST_F(BridgeRuntimeTest, HorusLinkTopicTableIsSortedByTopicAndType)
{
  MessageRouter router;
  auto fixture_node = std::make_shared<rclcpp::Node>("horuslink_topic_table_fixture");
  auto z_publisher = fixture_node->create_publisher<std_msgs::msg::String>(
    "/zz_horuslink_topic_table",
    rclcpp::QoS(1));
  auto a_publisher = fixture_node->create_publisher<std_msgs::msg::String>(
    "/aa_horuslink_topic_table",
    rclcpp::QoS(1));

  std::vector<horuslink::TopicEntry> entries;
  bool observed_fixture_topics = false;
  for (int attempt = 0; attempt < 100 && !observed_fixture_topics; ++attempt) {
    rclcpp::spin_some(fixture_node);
    entries = router.get_horuslink_topic_table();

    bool saw_a_topic = false;
    bool saw_z_topic = false;
    for (const auto & entry : entries) {
      saw_a_topic = saw_a_topic || entry.topic == "/aa_horuslink_topic_table";
      saw_z_topic = saw_z_topic || entry.topic == "/zz_horuslink_topic_table";
    }

    observed_fixture_topics = saw_a_topic && saw_z_topic;
    if (!observed_fixture_topics) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  ASSERT_TRUE(observed_fixture_topics);
  for (size_t i = 1; i < entries.size(); ++i) {
    const auto & previous = entries[i - 1];
    const auto & current = entries[i];
    EXPECT_TRUE(
      previous.topic < current.topic ||
      (previous.topic == current.topic && previous.type_name <= current.type_name))
      << "Topic table was not sorted at index " << i << ": "
      << previous.topic << " [" << previous.type_name << "] before "
      << current.topic << " [" << current.type_name << "]";
  }

  (void)z_publisher;
  (void)a_publisher;
}

TEST_F(BridgeRuntimeTest, PendingServiceStateIsTrackedPerClientAndClearedIndependently)
{
  MessageRouter router;

  auto encode_json = [](const std::string & json_str) {
      return std::vector<uint8_t>(json_str.begin(), json_str.end());
    };

  ProtocolMessage client_one_request{"__request", encode_json(R"({"srv_id":101})")};
  ProtocolMessage client_two_request{"__request", encode_json(R"({"srv_id":202})")};

  EXPECT_TRUE(router.route_message(11, client_one_request));
  EXPECT_TRUE(router.route_message(22, client_two_request));

  auto client_one_state = MessageRouterTestPeer::client_state(router, 11);
  auto client_two_state = MessageRouterTestPeer::client_state(router, 22);
  EXPECT_EQ(client_one_state.pending_request_id, 101u);
  EXPECT_EQ(client_two_state.pending_request_id, 202u);

  ProtocolMessage client_one_payload{"/test_service", bytes({0xAA})};
  EXPECT_FALSE(router.route_message(11, client_one_payload));
  EXPECT_FALSE(MessageRouterTestPeer::has_client_state(router, 11));
  EXPECT_TRUE(MessageRouterTestPeer::has_client_state(router, 22));

  ProtocolMessage client_two_response{"__response", encode_json(R"({"srv_id":303})")};
  EXPECT_TRUE(router.route_message(22, client_two_response));
  client_two_state = MessageRouterTestPeer::client_state(router, 22);
  EXPECT_EQ(client_two_state.pending_response_id, 303u);

  router.on_client_disconnected(22);
  EXPECT_FALSE(MessageRouterTestPeer::has_client_state(router, 22));
}

TEST_F(BridgeRuntimeTest, UnityBridgeNodeLoadsParametersAndKeepsServiceTimerAlive)
{
  const uint16_t port = find_unused_port();
  ASSERT_NE(port, 0);
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("tcp_ip", std::string("127.0.0.1")),
      rclcpp::Parameter("tcp_port", static_cast<int>(port)),
      rclcpp::Parameter("max_connections", 7),
      rclcpp::Parameter("socket_buffer_size", 131072),
      rclcpp::Parameter("message_queue_size", 17),
      rclcpp::Parameter("worker_threads", 3),
      rclcpp::Parameter("connection_timeout_ms", 7000),
      rclcpp::Parameter("tcp_nodelay", false),
      rclcpp::Parameter("log_protocol_messages", false),
      rclcpp::Parameter("message_pool_size", 2048),
      rclcpp::Parameter("enable_zero_copy", false),
      rclcpp::Parameter("default_publisher_qos.depth", 32),
      rclcpp::Parameter("default_subscriber_qos.depth", 32)
  });

  UnityBridgeNode node(options);
  EXPECT_EQ(node.bind_address(), "127.0.0.1");
  EXPECT_EQ(node.transport_protocol(), UnityBridgeNode::TransportProtocol::LegacyConnector);
  EXPECT_EQ(node.transport_protocol_name(), "legacy");
  EXPECT_EQ(node.connection_config().port, port);
  EXPECT_EQ(node.connection_config().max_connections, 7);
  EXPECT_EQ(node.connection_config().socket_buffer_size, 131072u);
  EXPECT_EQ(node.connection_config().message_queue_size, 17u);
  EXPECT_EQ(node.connection_config().connection_timeout_ms, 7000);
  EXPECT_FALSE(node.connection_config().tcp_nodelay);
  EXPECT_EQ(node.worker_threads(), 3);
  EXPECT_FALSE(node.log_protocol_messages());

  const auto & warnings = node.reserved_parameter_warnings();
  EXPECT_NE(std::find(warnings.begin(), warnings.end(), "message_pool_size"), warnings.end());
  EXPECT_NE(std::find(warnings.begin(), warnings.end(), "enable_zero_copy"), warnings.end());
  EXPECT_NE(std::find(warnings.begin(), warnings.end(), "default_publisher_qos.*"), warnings.end());
  EXPECT_NE(std::find(warnings.begin(), warnings.end(), "default_subscriber_qos.*"),
      warnings.end());

  EXPECT_FALSE(node.has_service_timer());
  ASSERT_TRUE(node.start());
  EXPECT_TRUE(node.has_service_timer());
  node.stop();
}

TEST_F(BridgeRuntimeTest, UnityBridgeNodeCanStartHorusLinkTransport)
{
  const uint16_t realtime_port = find_unused_port();
  const uint16_t bulk_port = find_unused_port();
  ASSERT_NE(realtime_port, 0);
  ASSERT_NE(bulk_port, 0);
  ASSERT_NE(realtime_port, bulk_port);

  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("tcp_ip", std::string("127.0.0.1")),
      rclcpp::Parameter("tcp_port", static_cast<int>(realtime_port)),
      rclcpp::Parameter("horuslink_bulk_port", static_cast<int>(bulk_port)),
      rclcpp::Parameter("transport_protocol", std::string("horuslink")),
      rclcpp::Parameter("log_protocol_messages", false)
  });

  UnityBridgeNode node(options);
  EXPECT_EQ(node.transport_protocol(), UnityBridgeNode::TransportProtocol::HorusLink);
  EXPECT_EQ(node.transport_protocol_name(), "horuslink");
  EXPECT_EQ(node.connection_config().port, realtime_port);
  EXPECT_EQ(node.horuslink_bulk_port(), bulk_port);

  EXPECT_FALSE(node.has_service_timer());
  ASSERT_TRUE(node.start());
  EXPECT_TRUE(node.has_service_timer());
  node.stop();
}

TEST_F(BridgeRuntimeTest, HorusLinkTransportAdvertisesConfiguredHello)
{
  const uint16_t realtime_port = find_unused_port();
  const uint16_t bulk_port = find_unused_port();
  ASSERT_NE(realtime_port, 0);
  ASSERT_NE(bulk_port, 0);
  ASSERT_NE(realtime_port, bulk_port);

  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("tcp_ip", std::string("127.0.0.1")),
      rclcpp::Parameter("tcp_port", static_cast<int>(realtime_port)),
      rclcpp::Parameter("horuslink_bulk_port", static_cast<int>(bulk_port)),
      rclcpp::Parameter("horuslink_max_payload_size", 4096),
      rclcpp::Parameter("horuslink_keepalive_ms", 250),
      rclcpp::Parameter("transport_protocol", std::string("horuslink")),
      rclcpp::Parameter("log_protocol_messages", false)
  });

  UnityBridgeNode node(options);
  ASSERT_TRUE(node.start());

  auto realtime = connect_to_port(realtime_port);
  auto bulk = connect_to_port(bulk_port);
  expect_bridge_hello(realtime.get(), 4096u, 250u);

  (void)bulk;
  node.stop();
}

TEST_F(BridgeRuntimeTest, HorusLinkTransportForwardsRosTopicToNegotiatedBulkLane)
{
  const uint16_t realtime_port = find_unused_port();
  const uint16_t bulk_port = find_unused_port();
  ASSERT_NE(realtime_port, 0);
  ASSERT_NE(bulk_port, 0);
  ASSERT_NE(realtime_port, bulk_port);

  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("tcp_ip", std::string("127.0.0.1")),
      rclcpp::Parameter("tcp_port", static_cast<int>(realtime_port)),
      rclcpp::Parameter("horuslink_bulk_port", static_cast<int>(bulk_port)),
      rclcpp::Parameter("transport_protocol", std::string("horuslink")),
      rclcpp::Parameter("log_protocol_messages", false)
  });

  UnityBridgeNode node(options);
  ASSERT_TRUE(node.start());

  auto realtime = connect_to_port(realtime_port);
  auto bulk = connect_to_port(bulk_port);
  expect_bridge_hello(realtime.get());

  const std::string topic = "/horuslink_loopback_string";
  const uint16_t channel_id = 31;
  const horuslink::SubscribeRequest request{
    channel_id,
    topic,
    "std_msgs/msg/String",
    horuslink::Lane::Bulk,
    horuslink::Delivery::ReplaceLatest
  };
  send_horuslink_frame(
    realtime.get(),
    make_horuslink_control_frame(horuslink::encode_subscribe_request(request)));

  horuslink::Frame ack_frame;
  ASSERT_TRUE(recv_horuslink_frame(realtime.get(), ack_frame));
  ASSERT_EQ(ack_frame.header.msg_type, horuslink::MessageType::Control);
  const auto ack = horuslink::decode_subscribe_ack(
    ack_frame.payload.data(),
    ack_frame.payload.size());
  ASSERT_TRUE(ack.has_value());
  ASSERT_EQ(ack->status, horuslink::SubscribeStatus::Accepted);
  ASSERT_EQ(ack->channel_id, channel_id);

  auto publisher_node = std::make_shared<rclcpp::Node>("horuslink_loopback_publisher");
  auto publisher = publisher_node->create_publisher<std_msgs::msg::String>(topic, 10);
  const std::string text = "horuslink-loopback";
  std_msgs::msg::String msg;
  msg.data = text;
  const auto expected_payload = detail::strip_cdr_header(serialize_string_message(text));

  horuslink::Frame received_frame;
  bool received = false;
  for (int attempt = 0; attempt < 20 && !received; ++attempt) {
    publisher->publish(msg);
    received = recv_horuslink_frame(bulk.get(), received_frame, 100);
  }

  ASSERT_TRUE(received);
  EXPECT_EQ(received_frame.header.channel_id, channel_id);
  EXPECT_EQ(received_frame.header.msg_type, horuslink::MessageType::Data);
  EXPECT_EQ(
    received_frame.header.flags,
    horuslink::FrameFlags::RawOpaque | horuslink::FrameFlags::ReplaceLatest);
  EXPECT_EQ(received_frame.header.seq, 1u);
  EXPECT_FALSE(detail::has_cdr_header(received_frame.payload));
  EXPECT_EQ(received_frame.payload, expected_payload);

  node.stop();
}

TEST_F(BridgeRuntimeTest, HorusLinkTransportPublishesDataFramesToRosTopic)
{
  const uint16_t realtime_port = find_unused_port();
  const uint16_t bulk_port = find_unused_port();
  ASSERT_NE(realtime_port, 0);
  ASSERT_NE(bulk_port, 0);
  ASSERT_NE(realtime_port, bulk_port);

  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("tcp_ip", std::string("127.0.0.1")),
      rclcpp::Parameter("tcp_port", static_cast<int>(realtime_port)),
      rclcpp::Parameter("horuslink_bulk_port", static_cast<int>(bulk_port)),
      rclcpp::Parameter("transport_protocol", std::string("horuslink")),
      rclcpp::Parameter("log_protocol_messages", false)
  });

  UnityBridgeNode node(options);
  ASSERT_TRUE(node.start());

  auto realtime = connect_to_port(realtime_port);
  auto bulk = connect_to_port(bulk_port);
  expect_bridge_hello(realtime.get());

  const std::string topic = "/horuslink_unity_publish_string";
  const uint16_t channel_id = 37;
  const horuslink::PublisherRequest request{
    channel_id,
    topic,
    "std_msgs/msg/String",
    horuslink::Lane::Realtime,
    horuslink::Delivery::ReliableFifo
  };
  send_horuslink_frame(
    realtime.get(),
    make_horuslink_control_frame(horuslink::encode_publisher_request(request)));

  horuslink::Frame ack_frame;
  ASSERT_TRUE(recv_horuslink_frame(realtime.get(), ack_frame));
  ASSERT_EQ(ack_frame.header.msg_type, horuslink::MessageType::Control);
  const auto ack = horuslink::decode_subscribe_ack(
    ack_frame.payload.data(),
    ack_frame.payload.size());
  ASSERT_TRUE(ack.has_value());
  ASSERT_EQ(ack->status, horuslink::SubscribeStatus::Accepted);
  ASSERT_EQ(ack->channel_id, channel_id);

  auto subscriber_node = std::make_shared<rclcpp::Node>("horuslink_loopback_subscriber");
  std::string received_text;
  auto subscriber = subscriber_node->create_subscription<std_msgs::msg::String>(
    topic,
    10,
    [&received_text](const std_msgs::msg::String::SharedPtr msg) {
      received_text = msg->data;
    });

  for (int attempt = 0; attempt < 50 && subscriber_node->count_publishers(topic) == 0; ++attempt) {
    rclcpp::spin_some(subscriber_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_GT(subscriber_node->count_publishers(topic), 0u);

  const std::string text = "unity-to-ros";
  const auto payload = detail::strip_cdr_header(serialize_string_message(text));
  ASSERT_FALSE(detail::has_cdr_header(payload));
  send_horuslink_frame(
    realtime.get(),
    make_horuslink_data_frame(channel_id, payload));

  for (int attempt = 0; attempt < 50 && received_text.empty(); ++attempt) {
    rclcpp::spin_some(subscriber_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  EXPECT_EQ(received_text, text);

  (void)subscriber;
  (void)bulk;
  node.stop();
}

TEST_F(BridgeRuntimeTest, HorusLinkTransportCallsRosServiceWithCorrelationId)
{
  const uint16_t realtime_port = find_unused_port();
  const uint16_t bulk_port = find_unused_port();
  ASSERT_NE(realtime_port, 0);
  ASSERT_NE(bulk_port, 0);
  ASSERT_NE(realtime_port, bulk_port);

  const std::string service_name = "/horuslink_add_two_ints";
  auto service_node = std::make_shared<rclcpp::Node>("horuslink_add_two_ints_server");
  auto service = service_node->create_service<example_interfaces::srv::AddTwoInts>(
    service_name,
    [](const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
    std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) {
      response->sum = request->a + request->b;
    });

  rclcpp::executors::SingleThreadedExecutor service_executor;
  service_executor.add_node(service_node);
  std::thread service_thread([&service_executor]() {
      service_executor.spin();
    });

  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("tcp_ip", std::string("127.0.0.1")),
      rclcpp::Parameter("tcp_port", static_cast<int>(realtime_port)),
      rclcpp::Parameter("horuslink_bulk_port", static_cast<int>(bulk_port)),
      rclcpp::Parameter("transport_protocol", std::string("horuslink")),
      rclcpp::Parameter("log_protocol_messages", false)
  });

  UnityBridgeNode node(options);
  bool node_started = node.start();
  EXPECT_TRUE(node_started);

  auto realtime = connect_to_port(realtime_port);
  auto bulk = connect_to_port(bulk_port);
  expect_bridge_hello(realtime.get());

  const uint16_t channel_id = 44;
  const uint32_t corr_id = 1234;
  const horuslink::ServiceClientRequest request{
    channel_id,
    service_name,
    "example_interfaces/srv/AddTwoInts",
    horuslink::Lane::Realtime,
    horuslink::Delivery::ReliableFifo
  };
  send_horuslink_frame(
    realtime.get(),
    make_horuslink_control_frame(horuslink::encode_service_client_request(request)));

  bool ack_ok = false;
  horuslink::Frame ack_frame;
  if (recv_horuslink_frame(realtime.get(), ack_frame)) {
    const auto ack = horuslink::decode_subscribe_ack(
      ack_frame.payload.data(),
      ack_frame.payload.size());
    ack_ok = ack.has_value() &&
      ack->status == horuslink::SubscribeStatus::Accepted &&
      ack->channel_id == channel_id;
  }
  EXPECT_TRUE(ack_ok);

  bool service_client_ready = false;
  for (int attempt = 0; attempt < 100 && !service_client_ready; ++attempt) {
    service_client_ready = service_node->count_clients(service_name) > 0;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  EXPECT_TRUE(service_client_ready);

  bool response_ok = false;
  int64_t response_sum = 0;
  uint32_t response_corr_id = 0;
  uint16_t response_channel_id = 0;
  horuslink::MessageType response_type = horuslink::MessageType::Data;
  if (ack_ok && service_client_ready && node_started) {
    send_horuslink_frame(
      realtime.get(),
      make_horuslink_service_request_frame(
        channel_id,
        corr_id,
        detail::strip_cdr_header(serialize_add_two_ints_request(5, 7))));

    horuslink::Frame response_frame;
    if (recv_horuslink_frame(realtime.get(), response_frame, 3000)) {
      response_corr_id = response_frame.header.corr_id;
      response_channel_id = response_frame.header.channel_id;
      response_type = response_frame.header.msg_type;
      try {
        if (detail::has_cdr_header(response_frame.payload)) {
          throw std::runtime_error("HorusLink service response still has CDR header");
        }
        const auto response = deserialize_add_two_ints_response(response_frame.payload);
        response_sum = response.sum;
        response_ok = true;
      } catch (const std::exception &) {
        response_ok = false;
      }
    }
  }

  service_executor.cancel();
  if (service_thread.joinable()) {
    service_thread.join();
  }
  node.stop();

  EXPECT_TRUE(response_ok);
  EXPECT_EQ(response_channel_id, channel_id);
  EXPECT_EQ(response_type, horuslink::MessageType::ServiceResponse);
  EXPECT_EQ(response_corr_id, corr_id);
  EXPECT_EQ(response_sum, 12);

  (void)service;
  (void)bulk;
}

}  // namespace horus_unity_bridge
