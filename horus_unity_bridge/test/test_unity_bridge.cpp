// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#include "horus_unity_bridge/connection_manager.hpp"
#include "horus_unity_bridge/message_router.hpp"
#include "horus_unity_bridge/unity_bridge_node.hpp"

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

namespace horus_unity_bridge
{

class MessageRouterTestPeer
{
public:
  static OutboundMessagePolicy classify_subscription_policy(
    const std::string& topic,
    const std::string& message_type)
  {
    return MessageRouter::classify_subscription_policy(topic, message_type);
  }

  static bool has_client_state(const MessageRouter& router, int client_fd)
  {
    std::lock_guard<std::mutex> lock(router.pending_service_state_mutex_);
    return router.pending_service_state_.find(client_fd) != router.pending_service_state_.end();
  }

  static MessageRouter::PendingServiceState client_state(const MessageRouter& router, int client_fd)
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

    if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
      close(fd);
      return 0;
    }

    socklen_t len = sizeof(addr);
    if (getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &len) != 0) {
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
    const auto& queue_view = static_cast<const OutboundMessageQueue&>(replaceable_queue);
    const auto* front = queue_view.front();
    return front == nullptr ? std::string() : front->destination;
  };
  auto front_first_byte = [&replaceable_queue]() {
    const auto& queue_view = static_cast<const OutboundMessageQueue&>(replaceable_queue);
    const auto* front = queue_view.front();
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
      "/points",
      "sensor_msgs/msg/PointCloud2"),
    OutboundMessagePolicy::Replaceable);
  EXPECT_EQ(
    MessageRouterTestPeer::classify_subscription_policy(
      "/horus/registration",
      "std_msgs/String"),
    OutboundMessagePolicy::Strict);
}

TEST_F(BridgeRuntimeTest, PendingServiceStateIsTrackedPerClientAndClearedIndependently)
{
  MessageRouter router;

  auto encode_json = [](const std::string& json_str) {
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
  EXPECT_EQ(node.connection_config().port, port);
  EXPECT_EQ(node.connection_config().max_connections, 7);
  EXPECT_EQ(node.connection_config().socket_buffer_size, 131072u);
  EXPECT_EQ(node.connection_config().message_queue_size, 17u);
  EXPECT_EQ(node.connection_config().connection_timeout_ms, 7000);
  EXPECT_FALSE(node.connection_config().tcp_nodelay);
  EXPECT_EQ(node.worker_threads(), 3);
  EXPECT_FALSE(node.log_protocol_messages());

  const auto& warnings = node.reserved_parameter_warnings();
  EXPECT_NE(std::find(warnings.begin(), warnings.end(), "message_pool_size"), warnings.end());
  EXPECT_NE(std::find(warnings.begin(), warnings.end(), "enable_zero_copy"), warnings.end());
  EXPECT_NE(std::find(warnings.begin(), warnings.end(), "default_publisher_qos.*"), warnings.end());
  EXPECT_NE(std::find(warnings.begin(), warnings.end(), "default_subscriber_qos.*"), warnings.end());

  EXPECT_FALSE(node.has_service_timer());
  ASSERT_TRUE(node.start());
  EXPECT_TRUE(node.has_service_timer());
  node.stop();
}

}  // namespace horus_unity_bridge
