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

// SPDX-FileCopyrightText: 2026 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#include "horus_unity_bridge/horuslink_outbound_frame_router.hpp"

#include <array>

#include <gtest/gtest.h>

namespace horus_unity_bridge::horuslink
{

namespace
{

void activate_channel(
  ChannelTable & table,
  uint16_t channel_id,
  const std::string & topic,
  Lane lane,
  Delivery delivery)
{
  ASSERT_TRUE(table.begin_subscribe(
    channel_id,
    topic,
    "std_msgs/msg/ByteMultiArray",
    lane,
    delivery));
  ASSERT_TRUE(table.confirm_subscribe_ack(channel_id));
}

}  // namespace

TEST(HorusLinkOutboundFrameRouterTest, RoutesRealtimeReliablePayloadsByNegotiatedChannel)
{
  ChannelTable table;
  activate_channel(table, 4, "/tf", Lane::Realtime, Delivery::ReliableFifo);
  OutboundFrameRouter router(4, 4);
  const std::array<uint8_t, 3> payload{0x01, 0x02, 0x03};

  auto result = router.enqueue_data(table, "/tf", payload.data(), payload.size());

  EXPECT_TRUE(result.accepted);
  EXPECT_FALSE(result.overflow);
  EXPECT_EQ(result.lane, Lane::Realtime);
  EXPECT_EQ(result.channel_id, 4);
  EXPECT_EQ(result.seq, 1u);
  EXPECT_EQ(result.realtime_depth, 1u);
  EXPECT_EQ(result.bulk_depth, 0u);

  auto routed = router.pop(Lane::Realtime);
  ASSERT_TRUE(routed.has_value());
  EXPECT_EQ(routed->lane, Lane::Realtime);
  EXPECT_EQ(routed->frame.header.channel_id, 4);
  EXPECT_EQ(routed->frame.header.msg_type, MessageType::Data);
  EXPECT_EQ(routed->frame.header.flags, FrameFlags::RawOpaque);
  EXPECT_EQ(routed->frame.header.seq, 1u);
  EXPECT_EQ(routed->frame.header.length, payload.size());
  EXPECT_EQ(routed->frame.payload, std::vector<uint8_t>(payload.begin(), payload.end()));
  EXPECT_FALSE(router.pop(Lane::Bulk).has_value());
}

TEST(HorusLinkOutboundFrameRouterTest, RoutesBulkReplaceLatestPayloadsToBulkQueue)
{
  ChannelTable table;
  activate_channel(table, 9, "/camera", Lane::Bulk, Delivery::ReplaceLatest);
  OutboundFrameRouter router(4, 4);
  const std::array<uint8_t, 2> first_payload{0x10, 0x11};
  const std::array<uint8_t, 2> second_payload{0x20, 0x21};

  auto first = router.enqueue_data(table, "/camera", first_payload.data(), first_payload.size());
  auto second = router.enqueue_data(table, "/camera", second_payload.data(), second_payload.size());

  EXPECT_TRUE(first.accepted);
  EXPECT_TRUE(second.accepted);
  EXPECT_EQ(first.lane, Lane::Bulk);
  EXPECT_EQ(second.lane, Lane::Bulk);
  EXPECT_EQ(first.seq, 1u);
  EXPECT_EQ(second.seq, 2u);
  EXPECT_EQ(router.bulk_depth(), 2u);

  auto routed = router.pop(Lane::Bulk);
  ASSERT_TRUE(routed.has_value());
  EXPECT_EQ(routed->frame.header.channel_id, 9);
  EXPECT_EQ(routed->frame.header.flags, FrameFlags::RawOpaque | FrameFlags::ReplaceLatest);
  EXPECT_EQ(routed->frame.header.seq, 1u);
  EXPECT_EQ(routed->frame.payload,
      std::vector<uint8_t>(first_payload.begin(), first_payload.end()));
  EXPECT_FALSE(router.pop(Lane::Realtime).has_value());
}

TEST(HorusLinkOutboundFrameRouterTest, ReplaceLatestOverflowKeepsNewestFrame)
{
  ChannelTable table;
  activate_channel(table, 9, "/camera", Lane::Bulk, Delivery::ReplaceLatest);
  OutboundFrameRouter router(4, 1);
  const std::array<uint8_t, 1> first_payload{0x10};
  const std::array<uint8_t, 1> second_payload{0x20};

  ASSERT_TRUE(router.enqueue_data(table, "/camera", first_payload.data(),
      first_payload.size()).accepted);
  auto second = router.enqueue_data(table, "/camera", second_payload.data(), second_payload.size());

  EXPECT_TRUE(second.accepted);
  EXPECT_TRUE(second.overflow);
  EXPECT_TRUE(second.replaced);
  ASSERT_TRUE(second.replaced_frame.has_value());
  EXPECT_EQ(second.replaced_frame->header.seq, 1u);
  auto routed = router.pop(Lane::Bulk);
  ASSERT_TRUE(routed.has_value());
  EXPECT_EQ(routed->frame.header.seq, 2u);
  EXPECT_EQ(routed->frame.payload,
      std::vector<uint8_t>(second_payload.begin(), second_payload.end()));
}

TEST(HorusLinkOutboundFrameRouterTest, ReliableOverflowRejectsAndDoesNotAdvanceSequence)
{
  ChannelTable table;
  activate_channel(table, 4, "/tf", Lane::Realtime, Delivery::ReliableFifo);
  OutboundFrameRouter router(1, 4);
  const std::array<uint8_t, 1> first_payload{0x01};
  const std::array<uint8_t, 1> overflow_payload{0x02};
  const std::array<uint8_t, 1> next_payload{0x03};

  ASSERT_TRUE(router.enqueue_data(table, "/tf", first_payload.data(),
      first_payload.size()).accepted);
  auto overflow = router.enqueue_data(table, "/tf", overflow_payload.data(),
      overflow_payload.size());

  EXPECT_FALSE(overflow.accepted);
  EXPECT_TRUE(overflow.overflow);
  EXPECT_EQ(overflow.seq, 2u);
  EXPECT_FALSE(overflow.error.empty());
  auto first = router.pop(Lane::Realtime);
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(first->frame.header.seq, 1u);

  auto next = router.enqueue_data(table, "/tf", next_payload.data(), next_payload.size());
  EXPECT_TRUE(next.accepted);
  EXPECT_EQ(next.seq, 2u);
  auto routed_next = router.pop(Lane::Realtime);
  ASSERT_TRUE(routed_next.has_value());
  EXPECT_EQ(routed_next->frame.header.seq, 2u);
}

TEST(HorusLinkOutboundFrameRouterTest, RejectsUnknownAndInactiveTopics)
{
  ChannelTable table;
  ASSERT_TRUE(table.begin_subscribe(
    7,
    "/pending",
    "std_msgs/msg/ByteMultiArray",
    Lane::Bulk,
    Delivery::ReliableFifo));
  OutboundFrameRouter router(4, 4);
  const std::array<uint8_t, 1> payload{0x01};

  auto unknown = router.enqueue_data(table, "/missing", payload.data(), payload.size());
  auto inactive = router.enqueue_data(table, "/pending", payload.data(), payload.size());

  EXPECT_FALSE(unknown.accepted);
  EXPECT_FALSE(unknown.error.empty());
  EXPECT_FALSE(inactive.accepted);
  EXPECT_EQ(inactive.channel_id, 7u);
  EXPECT_EQ(inactive.lane, Lane::Bulk);
  EXPECT_FALSE(inactive.error.empty());
  EXPECT_EQ(router.realtime_depth(), 0u);
  EXPECT_EQ(router.bulk_depth(), 0u);
}

TEST(HorusLinkOutboundFrameRouterTest, ServiceFramesRequireCorrelationIds)
{
  ChannelTable table;
  activate_channel(table, 5, "/set_bool", Lane::Realtime, Delivery::ReliableFifo);
  OutboundFrameRouter router(4, 4);
  const std::array<uint8_t, 2> payload{0xAA, 0xBB};

  auto rejected = router.enqueue_payload(
    table,
    "/set_bool",
    MessageType::ServiceResponse,
    0,
    payload.data(),
    payload.size());
  auto accepted = router.enqueue_payload(
    table,
    "/set_bool",
    MessageType::ServiceResponse,
    42,
    payload.data(),
    payload.size());

  EXPECT_FALSE(rejected.accepted);
  EXPECT_FALSE(rejected.error.empty());
  EXPECT_TRUE(accepted.accepted);
  auto routed = router.pop(Lane::Realtime);
  ASSERT_TRUE(routed.has_value());
  EXPECT_EQ(routed->frame.header.msg_type, MessageType::ServiceResponse);
  EXPECT_EQ(routed->frame.header.corr_id, 42u);
}

}  // namespace horus_unity_bridge::horuslink
