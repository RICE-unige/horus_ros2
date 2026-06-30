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

#include "horus_unity_bridge/horuslink_channel_table.hpp"

#include <gtest/gtest.h>

namespace horus_unity_bridge::horuslink
{

TEST(HorusLinkChannelTableTest, SubscribeAckPromotesChannelFromPendingToActive)
{
  ChannelTable table;
  ASSERT_TRUE(table.begin_subscribe(
      7,
      "/tf",
      "tf2_msgs/msg/TFMessage",
      Lane::Realtime,
      Delivery::ReliableFifo));

  auto pending = table.get(7);
  ASSERT_TRUE(pending.has_value());
  EXPECT_EQ(pending->state, ChannelState::PendingSubscribe);
  EXPECT_EQ(pending->topic, "/tf");
  EXPECT_EQ(pending->lane, Lane::Realtime);

  EXPECT_TRUE(table.confirm_subscribe_ack(7));
  auto active = table.get(7);
  ASSERT_TRUE(active.has_value());
  EXPECT_EQ(active->state, ChannelState::Active);
}

TEST(HorusLinkChannelTableTest, DataFramesAreRejectedBeforeSubscribeAck)
{
  ChannelTable table;
  ASSERT_TRUE(table.begin_subscribe(
      11,
      "/camera/image_raw",
      "sensor_msgs/msg/Image",
      Lane::Bulk,
      Delivery::ReplaceLatest));

  FrameHeader header;
  header.channel_id = 11;
  header.msg_type = MessageType::Data;
  header.seq = 1;
  header.length = 128;

  EXPECT_FALSE(table.can_accept_data_frame(header, Lane::Bulk));
  EXPECT_TRUE(table.confirm_subscribe_ack(11));
  EXPECT_TRUE(table.can_accept_data_frame(header, Lane::Bulk));
}

TEST(HorusLinkChannelTableTest, DataFramesAreRejectedOnTheWrongLane)
{
  ChannelTable table;
  ASSERT_TRUE(table.begin_subscribe(
      12,
      "/points",
      "sensor_msgs/msg/PointCloud2",
      Lane::Bulk,
      Delivery::ReplaceLatest));
  ASSERT_TRUE(table.confirm_subscribe_ack(12));

  FrameHeader header;
  header.channel_id = 12;
  header.msg_type = MessageType::Data;

  EXPECT_FALSE(table.can_accept_data_frame(header, Lane::Realtime));
  EXPECT_TRUE(table.can_accept_data_frame(header, Lane::Bulk));
}

TEST(HorusLinkChannelTableTest, TopicLookupAndRemoveKeepBindingsConsistent)
{
  ChannelTable table;
  ASSERT_TRUE(table.begin_subscribe(
      2,
      "/robot/state",
      "std_msgs/msg/String",
      Lane::Realtime,
      Delivery::ReliableFifo));

  auto descriptor = table.get_by_topic("/robot/state");
  ASSERT_TRUE(descriptor.has_value());
  EXPECT_EQ(descriptor->channel_id, 2);

  EXPECT_TRUE(table.remove(2));
  EXPECT_FALSE(table.get(2).has_value());
  EXPECT_FALSE(table.get_by_topic("/robot/state").has_value());
}

TEST(HorusLinkServiceTableTest, RegisterAllocatesNonzeroUniqueCorrelationIds)
{
  ServiceTable table;
  const uint32_t first = table.register_request(2, 100, 500);
  const uint32_t second = table.register_request(2, 101, 500);

  EXPECT_NE(first, 0u);
  EXPECT_NE(second, 0u);
  EXPECT_NE(first, second);
  EXPECT_EQ(table.pending_count(), 2u);
}

TEST(HorusLinkServiceTableTest, CompleteReturnsAndRemovesPendingRequest)
{
  ServiceTable table;
  const uint32_t corr_id = table.register_request(9, 1000, 250);

  auto request = table.complete(corr_id);
  ASSERT_TRUE(request.has_value());
  EXPECT_EQ(request->corr_id, corr_id);
  EXPECT_EQ(request->channel_id, 9);
  EXPECT_EQ(table.pending_count(), 0u);

  EXPECT_FALSE(table.complete(corr_id).has_value());
}

TEST(HorusLinkServiceTableTest, ExpireRemovesOnlyRequestsPastTheirTimeout)
{
  ServiceTable table;
  const uint32_t short_timeout = table.register_request(1, 100, 50);
  const uint32_t long_timeout = table.register_request(1, 100, 500);
  const uint32_t no_timeout = table.register_request(1, 100, -1);

  auto expired = table.expire(160);
  ASSERT_EQ(expired.size(), 1u);
  EXPECT_EQ(expired[0].corr_id, short_timeout);
  EXPECT_EQ(table.pending_count(), 2u);

  EXPECT_FALSE(table.complete(short_timeout).has_value());
  EXPECT_TRUE(table.complete(long_timeout).has_value());
  EXPECT_TRUE(table.complete(no_timeout).has_value());
}

TEST(HorusLinkServiceTableTest, ZeroCorrelationIdNeverCompletesARequest)
{
  ServiceTable table;
  table.register_request(1, 0, 100);

  EXPECT_FALSE(table.complete(0).has_value());
  EXPECT_EQ(table.pending_count(), 1u);
}

}  // namespace horus_unity_bridge::horuslink
