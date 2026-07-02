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

#include "horus_unity_bridge/horuslink_control_messages.hpp"
#include "horus_unity_bridge/horuslink_light_codecs.hpp"
#include "horus_unity_bridge/horuslink_protocol.hpp"
#include "horus_unity_bridge/message_router.hpp"
#include "horus_unity_bridge/serialized_payload.hpp"
#include "horus_unity_bridge/topic_manager.hpp"
#include "horus_unity_bridge/unity_bridge_node.hpp"

#include <gtest/gtest.h>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

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

void send_horuslink_client_hellos(
  int realtime_fd,
  int bulk_fd,
  uint64_t session_id = 0x5060708090A0B0C0ull)
{
  send_horuslink_frame(
    realtime_fd,
    make_horuslink_control_frame(
      horuslink::encode_hello(horuslink::HelloMessage{
        horuslink::EndpointRole::UnityClient,
        512u * 1024u * 1024u,
        1000,
        horuslink::Lane::Realtime,
        session_id}),
      1));
  send_horuslink_frame(
    bulk_fd,
    make_horuslink_control_frame(
      horuslink::encode_hello(horuslink::HelloMessage{
        horuslink::EndpointRole::UnityClient,
        512u * 1024u * 1024u,
        1000,
        horuslink::Lane::Bulk,
        session_id}),
      1));
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

TEST_F(BridgeRuntimeTest, HorusLinkTopicTableUsesNeutralLaneAndDelivery)
{
  MessageRouter router;
  auto fixture_node = std::make_shared<rclcpp::Node>("horuslink_topic_table_route_fixture");
  auto camera_publisher = fixture_node->create_publisher<sensor_msgs::msg::Image>(
    "/camera/front/image",
    rclcpp::QoS(1));
  auto cloud_publisher = fixture_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/map_3d",
    rclcpp::QoS(1));
  auto control_publisher = fixture_node->create_publisher<std_msgs::msg::String>(
    "/horus/registration",
    rclcpp::QoS(1));

  std::vector<horuslink::TopicEntry> entries;
  bool observed_fixture_topics = false;
  for (int attempt = 0; attempt < 100 && !observed_fixture_topics; ++attempt) {
    rclcpp::spin_some(fixture_node);
    entries = router.get_horuslink_topic_table();

    bool saw_camera = false;
    bool saw_cloud = false;
    bool saw_control = false;
    for (const auto & entry : entries) {
      saw_camera = saw_camera || entry.topic == "/camera/front/image";
      saw_cloud = saw_cloud || entry.topic == "/map_3d";
      saw_control = saw_control || entry.topic == "/horus/registration";
    }
    observed_fixture_topics = saw_camera && saw_cloud && saw_control;
    if (!observed_fixture_topics) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  ASSERT_TRUE(observed_fixture_topics);
  auto find_topic = [&entries](const std::string & topic) {
      auto it = std::find_if(
        entries.begin(),
        entries.end(),
        [&topic](const horuslink::TopicEntry & entry) {
          return entry.topic == topic;
        });
      if (it == entries.end()) {
        throw std::runtime_error("missing topic table entry: " + topic);
      }
      return *it;
    };

  const auto camera_entry = find_topic("/camera/front/image");
  EXPECT_EQ(camera_entry.lane, horuslink::Lane::Realtime);
  EXPECT_EQ(camera_entry.delivery, horuslink::Delivery::ReliableFifo);

  const auto cloud_entry = find_topic("/map_3d");
  EXPECT_EQ(cloud_entry.lane, horuslink::Lane::Realtime);
  EXPECT_EQ(cloud_entry.delivery, horuslink::Delivery::ReliableFifo);

  const auto control_entry = find_topic("/horus/registration");
  EXPECT_EQ(control_entry.lane, horuslink::Lane::Realtime);
  EXPECT_EQ(control_entry.delivery, horuslink::Delivery::ReliableFifo);

  (void)camera_publisher;
  (void)cloud_publisher;
  (void)control_publisher;
}

TEST_F(BridgeRuntimeTest, UnityBridgeNodeLoadsParametersAndKeepsServiceTimerAlive)
{
  const uint16_t port = find_unused_port();
  const uint16_t bulk_port = find_unused_port();
  ASSERT_NE(port, 0);
  ASSERT_NE(bulk_port, 0);
  ASSERT_NE(port, bulk_port);
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("tcp_ip", std::string("127.0.0.1")),
      rclcpp::Parameter("tcp_port", static_cast<int>(port)),
      rclcpp::Parameter("horuslink_bulk_port", static_cast<int>(bulk_port)),
      rclcpp::Parameter("max_connections", 7),
      rclcpp::Parameter("socket_buffer_size", 131072),
      rclcpp::Parameter("worker_threads", 3),
      rclcpp::Parameter("tcp_nodelay", false),
      rclcpp::Parameter("log_protocol_messages", false)
  });

  UnityBridgeNode node(options);
  EXPECT_EQ(node.bind_address(), "127.0.0.1");
  EXPECT_EQ(node.transport_protocol(), UnityBridgeNode::TransportProtocol::HorusLink);
  EXPECT_EQ(node.transport_protocol_name(), "horuslink");
  EXPECT_EQ(node.port(), port);
  EXPECT_EQ(node.horuslink_bulk_port(), bulk_port);
  EXPECT_EQ(node.max_connections(), 7);
  EXPECT_EQ(node.socket_buffer_size(), 131072u);
  EXPECT_FALSE(node.tcp_nodelay());
  EXPECT_EQ(node.worker_threads(), 3);
  EXPECT_FALSE(node.log_protocol_messages());

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
  EXPECT_EQ(node.port(), realtime_port);
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
  send_horuslink_client_hellos(realtime.get(), bulk.get());
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
  send_horuslink_client_hellos(realtime.get(), bulk.get());
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

TEST_F(BridgeRuntimeTest, HorusLinkRegistrationSubscriberReceivesRetainedState)
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

  auto publisher_node = std::make_shared<rclcpp::Node>("horuslink_registration_retained_publisher");
  auto retained_qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliability(rclcpp::ReliabilityPolicy::Reliable)
    .durability(rclcpp::DurabilityPolicy::TransientLocal);
  auto publisher = publisher_node->create_publisher<std_msgs::msg::String>(
    "/horus/registration",
    retained_qos);

  const std::string text = R"({"robot_name":"retained_bot","action":"register"})";
  std_msgs::msg::String msg;
  msg.data = text;
  publisher->publish(msg);
  rclcpp::spin_some(publisher_node);

  auto realtime = connect_to_port(realtime_port);
  auto bulk = connect_to_port(bulk_port);
  send_horuslink_client_hellos(realtime.get(), bulk.get());
  expect_bridge_hello(realtime.get());

  const uint16_t channel_id = 91;
  const horuslink::SubscribeRequest request{
    channel_id,
    "/horus/registration",
    "std_msgs/msg/String",
    horuslink::Lane::Realtime,
    horuslink::Delivery::ReliableFifo
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

  const auto expected_payload = detail::strip_cdr_header(serialize_string_message(text));
  horuslink::Frame received_frame;
  bool received = false;
  for (int attempt = 0; attempt < 30 && !received; ++attempt) {
    rclcpp::spin_some(publisher_node);
    horuslink::Frame candidate;
    if (!recv_horuslink_frame(realtime.get(), candidate, 100)) {
      continue;
    }
    if (candidate.header.msg_type == horuslink::MessageType::Data &&
      candidate.header.channel_id == channel_id)
    {
      received_frame = std::move(candidate);
      received = true;
    }
  }

  ASSERT_TRUE(received);
  EXPECT_EQ(received_frame.header.channel_id, channel_id);
  EXPECT_EQ(received_frame.header.msg_type, horuslink::MessageType::Data);
  EXPECT_FALSE(detail::has_cdr_header(received_frame.payload));
  EXPECT_EQ(received_frame.payload, expected_payload);

  (void)bulk;
  node.stop();
}

TEST_F(BridgeRuntimeTest, HorusLinkRegistrationReplayedToLateJoiner)
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

  auto publisher_node =
    std::make_shared<rclcpp::Node>("horuslink_registration_latejoin_publisher");
  auto retained_qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliability(rclcpp::ReliabilityPolicy::Reliable)
    .durability(rclcpp::DurabilityPolicy::TransientLocal);
  auto publisher = publisher_node->create_publisher<std_msgs::msg::String>(
    "/horus/registration",
    retained_qos);

  const std::string text = R"({"robot_name":"latejoin_bot","action":"register"})";
  std_msgs::msg::String msg;
  msg.data = text;
  publisher->publish(msg);
  rclcpp::spin_some(publisher_node);

  const auto expected_payload = detail::strip_cdr_header(serialize_string_message(text));

  // Subscribe on the given socket/channel and confirm both the ack and the retained
  // registration payload arrive (regardless of frame ordering).
  auto subscribe_and_expect_registration =
    [&](int fd, uint16_t channel_id) {
      const horuslink::SubscribeRequest request{
        channel_id,
        "/horus/registration",
        "std_msgs/msg/String",
        horuslink::Lane::Realtime,
        horuslink::Delivery::ReliableFifo
      };
      send_horuslink_frame(
        fd,
        make_horuslink_control_frame(horuslink::encode_subscribe_request(request)));

      bool acked = false;
      bool received = false;
      for (int attempt = 0; attempt < 40 && !(acked && received); ++attempt) {
        rclcpp::spin_some(publisher_node);
        horuslink::Frame candidate;
        if (!recv_horuslink_frame(fd, candidate, 100)) {
          continue;
        }
        if (candidate.header.msg_type == horuslink::MessageType::Control) {
          const auto ack = horuslink::decode_subscribe_ack(
            candidate.payload.data(),
            candidate.payload.size());
          if (ack.has_value() && ack->channel_id == channel_id) {
            EXPECT_EQ(ack->status, horuslink::SubscribeStatus::Accepted);
            acked = true;
          }
          continue;
        }

        if (candidate.header.msg_type == horuslink::MessageType::Data &&
          candidate.header.channel_id == channel_id)
        {
          EXPECT_FALSE(detail::has_cdr_header(candidate.payload));
          EXPECT_EQ(candidate.payload, expected_payload);
          received = true;
        }
      }
      EXPECT_TRUE(acked) << "subscribe was not acknowledged on channel " << channel_id;
      EXPECT_TRUE(received)
        << "retained registration was not delivered on channel " << channel_id;
    };

  // First operator connects, receives the retained registration, and primes the
  // bridge-side retained cache.
  auto realtime_a = connect_to_port(realtime_port);
  auto bulk_a = connect_to_port(bulk_port);
  send_horuslink_client_hellos(realtime_a.get(), bulk_a.get(), 0x1111111111111111ull);
  expect_bridge_hello(realtime_a.get());
  subscribe_and_expect_registration(realtime_a.get(), 91);

  // A late-joining second operator (distinct session) must be re-synced with the same
  // retained registration via deterministic replay -- this is the path that previously
  // relied on destroying/recreating the shared subscription and dropped the re-delivery.
  auto realtime_b = connect_to_port(realtime_port);
  auto bulk_b = connect_to_port(bulk_port);
  send_horuslink_client_hellos(realtime_b.get(), bulk_b.get(), 0x2222222222222222ull);
  expect_bridge_hello(realtime_b.get());
  subscribe_and_expect_registration(realtime_b.get(), 92);

  (void)bulk_a;
  (void)bulk_b;
  node.stop();
}

TEST_F(BridgeRuntimeTest, HorusLinkChunkSequenceReplayedToLateJoiner)
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

  // Simulate the SDK's retained robot-description chunk response.
  auto publisher_node =
    std::make_shared<rclcpp::Node>("horuslink_chunk_latejoin_publisher");
  auto retained_qos = rclcpp::QoS(rclcpp::KeepLast(512))
    .reliability(rclcpp::ReliabilityPolicy::Reliable)
    .durability(rclcpp::DurabilityPolicy::TransientLocal);
  auto chunk_item_pub = publisher_node->create_publisher<std_msgs::msg::String>(
    "/horus/robot_description/chunk_item",
    retained_qos);

  const std::string text =
    R"({"request_id":"RA","robot_name":"chunk_bot","description_id":"desc-1",)"
    R"("chunk_index":0,"total_chunks":1,"chunk_data":"QUFBQQ=="})";
  std_msgs::msg::String msg;
  msg.data = text;
  chunk_item_pub->publish(msg);
  rclcpp::spin_some(publisher_node);

  const auto expected_payload = detail::strip_cdr_header(serialize_string_message(text));

  // Subscribe the chunk_item topic on the Bulk lane (as Unity does); ack arrives on the Realtime
  // lane, the chunk payload on the Bulk lane. Confirm the chunk is delivered on both operators.
  auto subscribe_bulk_and_expect_chunk =
    [&](int realtime_fd, int bulk_fd, uint16_t channel_id) {
      const horuslink::SubscribeRequest request{
        channel_id,
        "/horus/robot_description/chunk_item",
        "std_msgs/msg/String",
        horuslink::Lane::Bulk,
        horuslink::Delivery::ReliableFifo
      };
      send_horuslink_frame(
        realtime_fd,
        make_horuslink_control_frame(horuslink::encode_subscribe_request(request)));

      bool acked = false;
      for (int attempt = 0; attempt < 20 && !acked; ++attempt) {
        horuslink::Frame ack_frame;
        if (!recv_horuslink_frame(realtime_fd, ack_frame, 100)) {
          continue;
        }
        if (ack_frame.header.msg_type != horuslink::MessageType::Control) {
          continue;
        }
        const auto ack = horuslink::decode_subscribe_ack(
          ack_frame.payload.data(),
          ack_frame.payload.size());
        if (ack.has_value() && ack->channel_id == channel_id) {
          EXPECT_EQ(ack->status, horuslink::SubscribeStatus::Accepted);
          acked = true;
        }
      }
      EXPECT_TRUE(acked) << "chunk subscribe not acked on channel " << channel_id;

      bool received = false;
      for (int attempt = 0; attempt < 30 && !received; ++attempt) {
        rclcpp::spin_some(publisher_node);
        horuslink::Frame candidate;
        if (!recv_horuslink_frame(bulk_fd, candidate, 100)) {
          continue;
        }
        if (candidate.header.msg_type == horuslink::MessageType::Data &&
          candidate.header.channel_id == channel_id)
        {
          EXPECT_EQ(candidate.payload, expected_payload);
          received = true;
        }
      }
      EXPECT_TRUE(received)
        << "retained chunk_item not delivered on channel " << channel_id;
    };

  // First operator: fresh ROS subscription (DDS latch) that also primes the bridge chunk cache.
  auto realtime_a = connect_to_port(realtime_port);
  auto bulk_a = connect_to_port(bulk_port);
  send_horuslink_client_hellos(realtime_a.get(), bulk_a.get(), 0x3111111111111111ull);
  expect_bridge_hello(realtime_a.get());
  subscribe_bulk_and_expect_chunk(realtime_a.get(), bulk_a.get(), 61);

  // Late joiner: the shared ROS subscription already exists, so it gets NO DDS latch re-delivery
  // and must be re-synced from the bridge chunk cache replay — the exact path that previously left
  // it with interaction boxes and no robot bodies.
  auto realtime_b = connect_to_port(realtime_port);
  auto bulk_b = connect_to_port(bulk_port);
  send_horuslink_client_hellos(realtime_b.get(), bulk_b.get(), 0x3222222222222222ull);
  expect_bridge_hello(realtime_b.get());
  subscribe_bulk_and_expect_chunk(realtime_b.get(), bulk_b.get(), 62);

  node.stop();
}

TEST_F(BridgeRuntimeTest, DynamicRobotDescriptionReplyTopicUsesTransientQosWithoutSharedReplayCache)
{
  auto subscriber_node =
    std::make_shared<rclcpp::Node>("horuslink_dynamic_reply_subscriber");
  TopicManager manager(subscriber_node.get());

  auto publisher_node =
    std::make_shared<rclcpp::Node>("horuslink_dynamic_reply_publisher");
  auto retained_qos = rclcpp::QoS(rclcpp::KeepLast(512))
    .reliability(rclcpp::ReliabilityPolicy::Reliable)
    .durability(rclcpp::DurabilityPolicy::TransientLocal);

  const std::string topic =
    "/horus/robot_description/replies/app_abc123/chunk_item";
  auto publisher = publisher_node->create_publisher<std_msgs::msg::String>(
    topic,
    retained_qos);

  const std::string text =
    R"({"request_id":"R_DYNAMIC","robot_name":"late_bot","description_id":"desc-dyn",)"
    R"("chunk_index":0,"total_chunks":1,"chunk_data":"QUFBQQ=="})";
  std_msgs::msg::String msg;
  msg.data = text;
  publisher->publish(msg);
  rclcpp::spin_some(publisher_node);

  const auto expected_payload = detail::strip_cdr_header(serialize_string_message(text));
  std::vector<uint8_t> received_payload;
  bool received = false;

  ASSERT_TRUE(manager.register_horuslink_subscriber(
    topic,
    "std_msgs/msg/String",
    77,
      [&](const std::string & callback_topic, const std::vector<uint8_t> & payload) {
        EXPECT_EQ(callback_topic, topic);
        received_payload = payload;
        received = true;
    },
    rclcpp::QoS(10)));

  for (int attempt = 0; attempt < 40 && !received; ++attempt) {
    rclcpp::spin_some(publisher_node);
    rclcpp::spin_some(subscriber_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
  }

  ASSERT_TRUE(received);
  EXPECT_EQ(received_payload, expected_payload);
  EXPECT_TRUE(manager.get_retained_payloads(topic).empty());
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
  send_horuslink_client_hellos(realtime.get(), bulk.get());
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

TEST_F(BridgeRuntimeTest, HorusLinkTransportPublishesLightTwistFramesToRosTopic)
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
  send_horuslink_client_hellos(realtime.get(), bulk.get());
  expect_bridge_hello(realtime.get());

  const std::string topic = "/horuslink_unity_publish_twist";
  const uint16_t channel_id = 38;
  const horuslink::PublisherRequest request{
    channel_id,
    topic,
    "geometry_msgs/msg/Twist",
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

  auto subscriber_node = std::make_shared<rclcpp::Node>("horuslink_twist_subscriber");
  geometry_msgs::msg::Twist received_twist;
  bool received = false;
  auto subscriber = subscriber_node->create_subscription<geometry_msgs::msg::Twist>(
    topic,
    10,
    [&received_twist, &received](const geometry_msgs::msg::Twist::SharedPtr msg) {
      received_twist = *msg;
      received = true;
    });

  for (int attempt = 0; attempt < 50 && subscriber_node->count_publishers(topic) == 0; ++attempt) {
    rclcpp::spin_some(subscriber_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_GT(subscriber_node->count_publishers(topic), 0u);

  const horuslink::Twist twist{
    horuslink::Vector3{0.0F, -0.5F, 0.75F},
    horuslink::Vector3{0.0F, 0.0F, -1.5F}
  };
  send_horuslink_frame(
    realtime.get(),
    make_horuslink_data_frame(
      channel_id,
      horuslink::encode_twist(twist),
      1,
      static_cast<uint8_t>(horuslink::FrameFlags::RawOpaque | horuslink::FrameFlags::LightCodec)));

  for (int attempt = 0; attempt < 50 && !received; ++attempt) {
    rclcpp::spin_some(subscriber_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  ASSERT_TRUE(received);
  EXPECT_DOUBLE_EQ(received_twist.linear.x, twist.linear.x);
  EXPECT_DOUBLE_EQ(received_twist.linear.y, twist.linear.y);
  EXPECT_DOUBLE_EQ(received_twist.linear.z, twist.linear.z);
  EXPECT_DOUBLE_EQ(received_twist.angular.x, twist.angular.x);
  EXPECT_DOUBLE_EQ(received_twist.angular.y, twist.angular.y);
  EXPECT_DOUBLE_EQ(received_twist.angular.z, twist.angular.z);

  (void)subscriber;
  (void)bulk;
  node.stop();
}

TEST_F(BridgeRuntimeTest, HorusLinkTransportPublishesLightPoseFramesToRosTopic)
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
  send_horuslink_client_hellos(realtime.get(), bulk.get());
  expect_bridge_hello(realtime.get());

  const std::string topic = "/horuslink_unity_publish_pose";
  const uint16_t channel_id = 42;
  const horuslink::PublisherRequest request{
    channel_id,
    topic,
    "geometry_msgs/msg/Pose",
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

  auto subscriber_node = std::make_shared<rclcpp::Node>("horuslink_pose_subscriber");
  geometry_msgs::msg::Pose received_pose;
  bool received = false;
  auto subscriber = subscriber_node->create_subscription<geometry_msgs::msg::Pose>(
    topic,
    10,
    [&received_pose, &received](const geometry_msgs::msg::Pose::SharedPtr msg) {
      received_pose = *msg;
      received = true;
    });

  for (int attempt = 0; attempt < 50 && subscriber_node->count_publishers(topic) == 0; ++attempt) {
    rclcpp::spin_some(subscriber_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_GT(subscriber_node->count_publishers(topic), 0u);

  const horuslink::Pose pose{
    horuslink::Vector3{2.0F, -1.5F, 0.25F},
    horuslink::Quaternion{0.0F, 0.70710677F, 0.0F, 0.70710677F}
  };
  send_horuslink_frame(
    realtime.get(),
    make_horuslink_data_frame(
      channel_id,
      horuslink::encode_pose(pose),
      1,
      static_cast<uint8_t>(horuslink::FrameFlags::RawOpaque | horuslink::FrameFlags::LightCodec)));

  for (int attempt = 0; attempt < 50 && !received; ++attempt) {
    rclcpp::spin_some(subscriber_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  ASSERT_TRUE(received);
  EXPECT_DOUBLE_EQ(received_pose.position.x, pose.position.x);
  EXPECT_DOUBLE_EQ(received_pose.position.y, pose.position.y);
  EXPECT_DOUBLE_EQ(received_pose.position.z, pose.position.z);
  EXPECT_DOUBLE_EQ(received_pose.orientation.x, pose.orientation.x);
  EXPECT_DOUBLE_EQ(received_pose.orientation.y, pose.orientation.y);
  EXPECT_DOUBLE_EQ(received_pose.orientation.z, pose.orientation.z);
  EXPECT_DOUBLE_EQ(received_pose.orientation.w, pose.orientation.w);

  (void)subscriber;
  (void)bulk;
  node.stop();
}

TEST_F(BridgeRuntimeTest, HorusLinkTransportPublishesLightPoseStampedFramesToRosTopic)
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
  send_horuslink_client_hellos(realtime.get(), bulk.get());
  expect_bridge_hello(realtime.get());

  const std::string topic = "/horuslink_unity_publish_pose_stamped";
  const uint16_t channel_id = 39;
  const horuslink::PublisherRequest request{
    channel_id,
    topic,
    "geometry_msgs/msg/PoseStamped",
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

  auto subscriber_node = std::make_shared<rclcpp::Node>("horuslink_pose_stamped_subscriber");
  geometry_msgs::msg::PoseStamped received_pose;
  bool received = false;
  auto subscriber = subscriber_node->create_subscription<geometry_msgs::msg::PoseStamped>(
    topic,
    10,
    [&received_pose, &received](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      received_pose = *msg;
      received = true;
    });

  for (int attempt = 0; attempt < 50 && subscriber_node->count_publishers(topic) == 0; ++attempt) {
    rclcpp::spin_some(subscriber_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_GT(subscriber_node->count_publishers(topic), 0u);

  const horuslink::PoseStamped pose{
    12,
    345,
    "drone/map",
    horuslink::Pose{
      horuslink::Vector3{1.25F, -0.5F, 0.75F},
      horuslink::Quaternion{0.0F, 0.0F, 0.70710677F, 0.70710677F}
    }
  };
  send_horuslink_frame(
    realtime.get(),
    make_horuslink_data_frame(
      channel_id,
      horuslink::encode_pose_stamped(pose),
      1,
      static_cast<uint8_t>(horuslink::FrameFlags::RawOpaque | horuslink::FrameFlags::LightCodec)));

  for (int attempt = 0; attempt < 50 && !received; ++attempt) {
    rclcpp::spin_some(subscriber_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  ASSERT_TRUE(received);
  EXPECT_EQ(received_pose.header.stamp.sec, static_cast<int32_t>(pose.seconds));
  EXPECT_EQ(received_pose.header.stamp.nanosec, pose.nanoseconds);
  EXPECT_EQ(received_pose.header.frame_id, pose.frame_id);
  EXPECT_DOUBLE_EQ(received_pose.pose.position.x, pose.pose.position.x);
  EXPECT_DOUBLE_EQ(received_pose.pose.position.y, pose.pose.position.y);
  EXPECT_DOUBLE_EQ(received_pose.pose.position.z, pose.pose.position.z);
  EXPECT_DOUBLE_EQ(received_pose.pose.orientation.x, pose.pose.orientation.x);
  EXPECT_DOUBLE_EQ(received_pose.pose.orientation.y, pose.pose.orientation.y);
  EXPECT_DOUBLE_EQ(received_pose.pose.orientation.z, pose.pose.orientation.z);
  EXPECT_DOUBLE_EQ(received_pose.pose.orientation.w, pose.pose.orientation.w);

  (void)subscriber;
  (void)bulk;
  node.stop();
}

TEST_F(BridgeRuntimeTest, HorusLinkTransportPublishesLightJoyFramesToRosTopic)
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
  send_horuslink_client_hellos(realtime.get(), bulk.get());
  expect_bridge_hello(realtime.get());

  const std::string topic = "/horuslink_unity_publish_joy";
  const uint16_t channel_id = 40;
  const horuslink::PublisherRequest request{
    channel_id,
    topic,
    "sensor_msgs/msg/Joy",
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

  auto subscriber_node = std::make_shared<rclcpp::Node>("horuslink_joy_subscriber");
  sensor_msgs::msg::Joy received_joy;
  bool received = false;
  auto subscriber = subscriber_node->create_subscription<sensor_msgs::msg::Joy>(
    topic,
    10,
    [&received_joy, &received](const sensor_msgs::msg::Joy::SharedPtr msg) {
      received_joy = *msg;
      received = true;
    });

  for (int attempt = 0; attempt < 50 && subscriber_node->count_publishers(topic) == 0; ++attempt) {
    rclcpp::spin_some(subscriber_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_GT(subscriber_node->count_publishers(topic), 0u);

  const horuslink::Joy joy{
    {1.0F, -0.5F, 0.25F, 0.0F},
    {1, 0, -1, 1}
  };
  send_horuslink_frame(
    realtime.get(),
    make_horuslink_data_frame(
      channel_id,
      horuslink::encode_joy(joy),
      1,
      static_cast<uint8_t>(horuslink::FrameFlags::RawOpaque | horuslink::FrameFlags::LightCodec)));

  for (int attempt = 0; attempt < 50 && !received; ++attempt) {
    rclcpp::spin_some(subscriber_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  ASSERT_TRUE(received);
  ASSERT_EQ(received_joy.axes.size(), joy.axes.size());
  ASSERT_EQ(received_joy.buttons.size(), joy.buttons.size());
  for (size_t i = 0; i < joy.axes.size(); ++i) {
    EXPECT_FLOAT_EQ(received_joy.axes[i], joy.axes[i]);
  }
  for (size_t i = 0; i < joy.buttons.size(); ++i) {
    EXPECT_EQ(received_joy.buttons[i], joy.buttons[i]);
  }

  (void)subscriber;
  (void)bulk;
  node.stop();
}

TEST_F(BridgeRuntimeTest, HorusLinkTransportPublishesLightPathFramesToRosTopic)
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
  send_horuslink_client_hellos(realtime.get(), bulk.get());
  expect_bridge_hello(realtime.get());

  const std::string topic = "/horuslink_unity_publish_path";
  const uint16_t channel_id = 41;
  const horuslink::PublisherRequest request{
    channel_id,
    topic,
    "nav_msgs/msg/Path",
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

  auto subscriber_node = std::make_shared<rclcpp::Node>("horuslink_path_subscriber");
  nav_msgs::msg::Path received_path;
  bool received = false;
  auto subscriber = subscriber_node->create_subscription<nav_msgs::msg::Path>(
    topic,
    10,
    [&received_path, &received](const nav_msgs::msg::Path::SharedPtr msg) {
      received_path = *msg;
      received = true;
    });

  for (int attempt = 0; attempt < 50 && subscriber_node->count_publishers(topic) == 0; ++attempt) {
    rclcpp::spin_some(subscriber_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_GT(subscriber_node->count_publishers(topic), 0u);

  const horuslink::Path path{
    50,
    60,
    "map",
    {
      horuslink::PoseStamped{
        51,
        61,
        "map",
        horuslink::Pose{
          horuslink::Vector3{1.0F, 2.0F, 3.0F},
          horuslink::Quaternion{0.0F, 0.0F, 0.0F, 1.0F}
        }
      },
      horuslink::PoseStamped{
        52,
        62,
        "odom",
        horuslink::Pose{
          horuslink::Vector3{4.0F, 5.0F, 6.0F},
          horuslink::Quaternion{0.0F, 0.0F, 0.70710677F, 0.70710677F}
        }
      }
    }
  };
  send_horuslink_frame(
    realtime.get(),
    make_horuslink_data_frame(
      channel_id,
      horuslink::encode_path(path),
      1,
      static_cast<uint8_t>(horuslink::FrameFlags::RawOpaque | horuslink::FrameFlags::LightCodec)));

  for (int attempt = 0; attempt < 50 && !received; ++attempt) {
    rclcpp::spin_some(subscriber_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  ASSERT_TRUE(received);
  EXPECT_EQ(received_path.header.stamp.sec, static_cast<int32_t>(path.seconds));
  EXPECT_EQ(received_path.header.stamp.nanosec, path.nanoseconds);
  EXPECT_EQ(received_path.header.frame_id, path.frame_id);
  ASSERT_EQ(received_path.poses.size(), path.poses.size());
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const auto & expected = path.poses[i];
    const auto & actual = received_path.poses[i];
    EXPECT_EQ(actual.header.stamp.sec, static_cast<int32_t>(expected.seconds));
    EXPECT_EQ(actual.header.stamp.nanosec, expected.nanoseconds);
    EXPECT_EQ(actual.header.frame_id, expected.frame_id);
    EXPECT_DOUBLE_EQ(actual.pose.position.x, expected.pose.position.x);
    EXPECT_DOUBLE_EQ(actual.pose.position.y, expected.pose.position.y);
    EXPECT_DOUBLE_EQ(actual.pose.position.z, expected.pose.position.z);
    EXPECT_DOUBLE_EQ(actual.pose.orientation.x, expected.pose.orientation.x);
    EXPECT_DOUBLE_EQ(actual.pose.orientation.y, expected.pose.orientation.y);
    EXPECT_DOUBLE_EQ(actual.pose.orientation.z, expected.pose.orientation.z);
    EXPECT_DOUBLE_EQ(actual.pose.orientation.w, expected.pose.orientation.w);
  }

  (void)subscriber;
  (void)bulk;
  node.stop();
}

TEST_F(BridgeRuntimeTest, HorusLinkTransportPublishesLightTfFramesToRosTopic)
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
  send_horuslink_client_hellos(realtime.get(), bulk.get());
  expect_bridge_hello(realtime.get());

  const std::string topic = "/horuslink_unity_publish_tf";
  const uint16_t channel_id = 43;
  const horuslink::PublisherRequest request{
    channel_id,
    topic,
    "tf2_msgs/msg/TFMessage",
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

  auto subscriber_node = std::make_shared<rclcpp::Node>("horuslink_tf_subscriber");
  tf2_msgs::msg::TFMessage received_tf;
  bool received = false;
  auto subscriber = subscriber_node->create_subscription<tf2_msgs::msg::TFMessage>(
    topic,
    10,
    [&received_tf, &received](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
      received_tf = *msg;
      received = true;
    });

  for (int attempt = 0; attempt < 50 && subscriber_node->count_publishers(topic) == 0; ++attempt) {
    rclcpp::spin_some(subscriber_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_GT(subscriber_node->count_publishers(topic), 0u);

  const std::vector<horuslink::TransformStamped> transforms{
    horuslink::TransformStamped{
      1,
      2,
      "map",
      "base_link",
      horuslink::Vector3{1.0F, -2.0F, 0.5F},
      horuslink::Quaternion{0.0F, 0.0F, 0.0F, 1.0F}
    }
  };
  send_horuslink_frame(
    realtime.get(),
    make_horuslink_data_frame(
      channel_id,
      horuslink::encode_tf_message(transforms),
      1,
      static_cast<uint8_t>(horuslink::FrameFlags::RawOpaque | horuslink::FrameFlags::LightCodec)));

  for (int attempt = 0; attempt < 50 && !received; ++attempt) {
    rclcpp::spin_some(subscriber_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  ASSERT_TRUE(received);
  ASSERT_EQ(received_tf.transforms.size(), transforms.size());
  const auto & actual = received_tf.transforms[0];
  const auto & expected = transforms[0];
  EXPECT_EQ(actual.header.stamp.sec, static_cast<int32_t>(expected.seconds));
  EXPECT_EQ(actual.header.stamp.nanosec, expected.nanoseconds);
  EXPECT_EQ(actual.header.frame_id, expected.parent_frame_id);
  EXPECT_EQ(actual.child_frame_id, expected.child_frame_id);
  EXPECT_DOUBLE_EQ(actual.transform.translation.x, expected.translation.x);
  EXPECT_DOUBLE_EQ(actual.transform.translation.y, expected.translation.y);
  EXPECT_DOUBLE_EQ(actual.transform.translation.z, expected.translation.z);
  EXPECT_DOUBLE_EQ(actual.transform.rotation.x, expected.rotation.x);
  EXPECT_DOUBLE_EQ(actual.transform.rotation.y, expected.rotation.y);
  EXPECT_DOUBLE_EQ(actual.transform.rotation.z, expected.rotation.z);
  EXPECT_DOUBLE_EQ(actual.transform.rotation.w, expected.rotation.w);

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
  send_horuslink_client_hellos(realtime.get(), bulk.get());
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
