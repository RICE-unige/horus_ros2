// Copyright 2026 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0
#include "horus_unity_bridge/input_timing.hpp"
#include "horus_unity_bridge/horuslink_session.hpp"
#include "horus_unity_bridge/message_router.hpp"
#include <gtest/gtest.h>

namespace horus_unity_bridge
{
TEST(InputTiming, LegacyHelloAndOptionalCapabilityRoundTrip)
{
  horuslink::HelloMessage hello;
  hello.role = horuslink::EndpointRole::Bridge;
  auto legacy = horuslink::encode_hello(hello);
  ASSERT_TRUE(horuslink::decode_hello(legacy.data(), legacy.size()));
  EXPECT_EQ(horuslink::decode_hello(legacy.data(), legacy.size())->features, 0U);
  hello.features = horuslink::kTimedInputFeature;
  auto current = horuslink::encode_hello(hello);
  ASSERT_TRUE(horuslink::decode_hello(current.data(), current.size()));
  EXPECT_EQ(horuslink::decode_hello(current.data(), current.size())->features, horuslink::kTimedInputFeature);
  EXPECT_EQ(current.size(), legacy.size() + 8);
}

TEST(InputTiming, ClockProbeEchoDoesNotChangeLegacyEmptyKeepalive)
{
  horuslink::Session session;
  auto frame = session.make_keepalive_frame();
  EXPECT_TRUE(session.handle_frame(frame).empty());
  frame.payload.resize(20);
  frame.payload[0] = 'H'; frame.payload[1] = 'L'; frame.payload[2] = 'T'; frame.payload[3] = '1';
  horuslink::write_timing_u64(frame.payload.data() + 4, 123);
  const auto result = session.handle_frame(frame);
  ASSERT_EQ(result.size(), 1U);
  EXPECT_EQ(result[0].header.msg_type, horuslink::MessageType::Keepalive);
  EXPECT_EQ(horuslink::read_timing_u64(result[0].payload.data() + 4), 123U);
  EXPECT_GT(horuslink::read_timing_u64(result[0].payload.data() + 12), 0U);
  EXPECT_TRUE(session.handle_frame(result[0]).empty());
}

TEST(InputTiming, BridgeDropsDelayedLocalMotionBeforeRosPublication)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  auto router = std::make_shared<MessageRouter>(rclcpp::NodeOptions(), true);
  horuslink::ChannelDescriptor channel;
  channel.topic = "/timed_control_unit/cmd_vel";
  channel.type_name = "geometry_msgs/msg/Twist";
  channel.delivery = horuslink::Delivery::ReplaceLatest;
  ASSERT_TRUE(router->register_horuslink_publisher(7, channel));
  std::vector<uint8_t> bytes(64, 0);
  horuslink::write_timing_u64(bytes.data(), horuslink::input_clock_ms() - 1);
  EXPECT_FALSE(router->route_horuslink_data_frame(7, channel, bytes, horuslink::kTimedInputFlag | 1));
  horuslink::write_timing_u64(bytes.data(), horuslink::input_clock_ms() + 1000);
  EXPECT_FALSE(router->route_horuslink_data_frame(7, channel, bytes, horuslink::kTimedInputFlag | 1));
  horuslink::write_timing_u64(bytes.data(), horuslink::input_clock_ms() + 200);
  EXPECT_TRUE(router->route_horuslink_data_frame(7, channel, bytes, horuslink::kTimedInputFlag | 1));
  EXPECT_TRUE(router->route_horuslink_data_frame(7, channel, std::vector<uint8_t>(48), 1));
}
}  // namespace horus_unity_bridge
