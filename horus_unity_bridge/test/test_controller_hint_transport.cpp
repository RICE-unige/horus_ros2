// Copyright 2026 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#include "horus_unity_bridge/horuslink_outbound_frame_router.hpp"

#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#include <gtest/gtest.h>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <std_msgs/msg/string.hpp>

namespace horus_unity_bridge::horuslink
{
TEST(ControllerHintTransport, FullSdkRegistrationSurvivesActualRosSerializationAndRouting)
{
  const auto path = std::filesystem::path(HORUS_UNITY_BRIDGE_TEST_SOURCE_DIR) /
    "test/fixtures/controller_hints_registration.json";
  std::ifstream input(path);
  ASSERT_TRUE(input.good());
  std_msgs::msg::String message;
  message.data.assign(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
  ASSERT_NE(message.data.find("controller_hints"), std::string::npos);
  rclcpp::Serialization<std_msgs::msg::String> codec;
  rclcpp::SerializedMessage serialized;
  codec.serialize_message(&message, &serialized);
  const auto & source = serialized.get_rcl_serialized_message();

  ChannelTable channels;
  ASSERT_TRUE(channels.begin_subscribe(7, "/horus/registration", "std_msgs/msg/String",
    Lane::Realtime, Delivery::ReliableFifo));
  ASSERT_TRUE(channels.confirm_subscribe_ack(7));
  OutboundFrameRouter router(4, 4);
  ASSERT_TRUE(router.enqueue_data(channels, "/horus/registration", source.buffer,
    source.buffer_length).accepted);
  const auto frame = router.pop(Lane::Realtime);
  ASSERT_TRUE(frame.has_value());
  ASSERT_EQ(frame->frame.payload.size(), source.buffer_length);
  EXPECT_EQ(frame->frame.header.channel_id, 7);

  rclcpp::SerializedMessage returned(frame->frame.payload.size());
  auto & destination = returned.get_rcl_serialized_message();
  std::memcpy(destination.buffer, frame->frame.payload.data(), frame->frame.payload.size());
  destination.buffer_length = frame->frame.payload.size();
  std_msgs::msg::String decoded;
  codec.deserialize_message(&returned, &decoded);
  EXPECT_EQ(decoded.data, message.data);
  EXPECT_FALSE(router.pop(Lane::Realtime).has_value());
}
}  // namespace horus_unity_bridge::horuslink
