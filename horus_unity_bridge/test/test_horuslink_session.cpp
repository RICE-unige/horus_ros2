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

#include "horus_unity_bridge/horuslink_session.hpp"

#include <gtest/gtest.h>

namespace horus_unity_bridge::horuslink
{

namespace
{

Frame make_control_frame(std::vector<uint8_t> payload, uint32_t seq = 1)
{
  Frame frame;
  frame.header.channel_id = 0;
  frame.header.msg_type = MessageType::Control;
  frame.header.seq = seq;
  frame.header.length = static_cast<uint32_t>(payload.size());
  frame.payload = std::move(payload);
  return frame;
}

Frame make_data_frame(uint16_t channel_id, std::vector<uint8_t> payload, uint32_t seq = 1)
{
  Frame frame;
  frame.header.channel_id = channel_id;
  frame.header.msg_type = MessageType::Data;
  frame.header.flags = FrameFlags::RawOpaque;
  frame.header.seq = seq;
  frame.header.length = static_cast<uint32_t>(payload.size());
  frame.payload = std::move(payload);
  return frame;
}

Frame make_service_frame(
  uint16_t channel_id,
  MessageType message_type,
  uint32_t corr_id,
  std::vector<uint8_t> payload,
  uint32_t seq = 1)
{
  Frame frame;
  frame.header.channel_id = channel_id;
  frame.header.msg_type = message_type;
  frame.header.flags = FrameFlags::RawOpaque;
  frame.header.seq = seq;
  frame.header.corr_id = corr_id;
  frame.header.length = static_cast<uint32_t>(payload.size());
  frame.payload = std::move(payload);
  return frame;
}

Frame make_keepalive_frame(uint32_t seq, uint16_t channel_id = 0, uint32_t corr_id = 0)
{
  Frame frame;
  frame.header.channel_id = channel_id;
  frame.header.msg_type = MessageType::Keepalive;
  frame.header.seq = seq;
  frame.header.corr_id = corr_id;
  frame.header.length = 0;
  return frame;
}

void register_bulk_channel(Session & session, uint16_t channel_id)
{
  const SubscribeRequest request{
    channel_id,
    "/camera",
    "sensor_msgs/msg/Image",
    Lane::Bulk,
    Delivery::ReplaceLatest
  };
  auto responses = session.handle_frame(make_control_frame(encode_subscribe_request(request)));
  ASSERT_EQ(responses.size(), 1u);
}

}  // namespace

TEST(HorusLinkSessionTest, HelloFrameStoresPeerRoleWithoutResponse)
{
  Session session;
  Frame hello = make_control_frame(encode_hello(HelloMessage{EndpointRole::UnityClient, 1024,
        1000}));

  auto responses = session.handle_frame(hello);

  EXPECT_TRUE(responses.empty());
  ASSERT_TRUE(session.peer_hello().has_value());
  EXPECT_EQ(session.peer_hello()->role, EndpointRole::UnityClient);
  EXPECT_EQ(session.peer_hello()->max_payload_bytes, 1024u);
}

TEST(HorusLinkSessionTest, SubscribeRequestActivatesChannelAndReturnsAck)
{
  Session session;
  const SubscribeRequest request{
    12,
    "/camera",
    "sensor_msgs/msg/Image",
    Lane::Bulk,
    Delivery::ReplaceLatest
  };

  auto responses = session.handle_frame(make_control_frame(encode_subscribe_request(request)));

  ASSERT_EQ(responses.size(), 1u);
  EXPECT_EQ(responses[0].header.msg_type, MessageType::Control);
  EXPECT_EQ(responses[0].header.channel_id, 0);
  EXPECT_EQ(responses[0].header.seq, 1u);
  auto ack = decode_subscribe_ack(responses[0].payload.data(), responses[0].payload.size());
  ASSERT_TRUE(ack.has_value());
  EXPECT_EQ(ack->channel_id, 12);
  EXPECT_EQ(ack->status, SubscribeStatus::Accepted);

  auto channel = session.channel_table().get(12);
  ASSERT_TRUE(channel.has_value());
  EXPECT_EQ(channel->state, ChannelState::Active);
  EXPECT_EQ(channel->lane, Lane::Bulk);
}

TEST(HorusLinkSessionTest, DuplicateSubscribeRequestReturnsRejectedAck)
{
  Session session;
  const SubscribeRequest request{
    3,
    "/tf",
    "tf2_msgs/msg/TFMessage",
    Lane::Realtime,
    Delivery::ReliableFifo
  };

  auto first = session.handle_frame(make_control_frame(encode_subscribe_request(request), 1));
  auto second = session.handle_frame(make_control_frame(encode_subscribe_request(request), 2));

  ASSERT_EQ(first.size(), 1u);
  ASSERT_EQ(second.size(), 1u);
  auto first_ack = decode_subscribe_ack(first[0].payload.data(), first[0].payload.size());
  auto second_ack = decode_subscribe_ack(second[0].payload.data(), second[0].payload.size());
  ASSERT_TRUE(first_ack.has_value());
  ASSERT_TRUE(second_ack.has_value());
  EXPECT_EQ(first_ack->status, SubscribeStatus::Accepted);
  EXPECT_EQ(second_ack->status, SubscribeStatus::Rejected);
  EXPECT_FALSE(second_ack->error.empty());
}

TEST(HorusLinkSessionTest, DataFrameIsAcceptedOnlyAfterSubscriptionAckOnNegotiatedLane)
{
  Session session;
  register_bulk_channel(session, 22);
  Frame data = make_data_frame(22, {0x01, 0x02, 0x03});

  auto accepted = session.handle_frame(data, Lane::Bulk);

  ASSERT_EQ(accepted.size(), 1u);
  EXPECT_EQ(accepted[0].header.channel_id, 22);
  EXPECT_EQ(accepted[0].payload, data.payload);
}

TEST(HorusLinkSessionTest, DataFrameOnWrongLaneIsRejected)
{
  Session session;
  register_bulk_channel(session, 22);
  Frame data = make_data_frame(22, {0x01});

  auto rejected = session.handle_frame(data, Lane::Realtime);

  EXPECT_TRUE(rejected.empty());
}

TEST(HorusLinkSessionTest, DataFrameForUnknownChannelIsRejected)
{
  Session session;
  Frame data = make_data_frame(99, {0x01});

  auto rejected = session.handle_frame(data, Lane::Bulk);

  EXPECT_TRUE(rejected.empty());
}

TEST(HorusLinkSessionTest, KeepaliveFrameRecordsLastSequenceWithoutResponse)
{
  Session session;
  Frame keepalive = make_keepalive_frame(55);

  auto responses = session.handle_frame(keepalive, Lane::Realtime);

  EXPECT_TRUE(responses.empty());
  ASSERT_TRUE(session.last_keepalive_sequence().has_value());
  EXPECT_EQ(*session.last_keepalive_sequence(), 55u);
}

TEST(HorusLinkSessionTest, MalformedKeepaliveFrameIsIgnored)
{
  Session session;
  Frame keepalive = make_keepalive_frame(55, 1, 99);

  auto responses = session.handle_frame(keepalive, Lane::Realtime);

  EXPECT_TRUE(responses.empty());
  EXPECT_FALSE(session.last_keepalive_sequence().has_value());
}

TEST(HorusLinkSessionTest, ServiceFramesAreAcceptedOnlyWithCorrelationIdOnNegotiatedLane)
{
  Session session;
  register_bulk_channel(session, 22);
  Frame request = make_service_frame(
    22,
    MessageType::ServiceRequest,
    123,
    {0x01, 0x02},
    9);
  Frame response = make_service_frame(
    22,
    MessageType::ServiceResponse,
    123,
    {0x03},
    10);

  auto accepted_request = session.handle_frame(request, Lane::Bulk);
  auto accepted_response = session.handle_frame(response, Lane::Bulk);

  ASSERT_EQ(accepted_request.size(), 1u);
  ASSERT_EQ(accepted_response.size(), 1u);
  EXPECT_EQ(accepted_request[0].header.corr_id, 123u);
  EXPECT_EQ(accepted_request[0].payload, request.payload);
  EXPECT_EQ(accepted_response[0].header.corr_id, 123u);
  EXPECT_EQ(accepted_response[0].payload, response.payload);
}

TEST(HorusLinkSessionTest, ServiceFrameWithoutCorrelationIdIsRejected)
{
  Session session;
  register_bulk_channel(session, 22);
  Frame request = make_service_frame(22, MessageType::ServiceRequest, 0, {0x01}, 9);

  auto rejected = session.handle_frame(request, Lane::Bulk);

  EXPECT_TRUE(rejected.empty());
}

TEST(HorusLinkSessionTest, ServiceFrameOnWrongLaneIsRejected)
{
  Session session;
  register_bulk_channel(session, 22);
  Frame request = make_service_frame(22, MessageType::ServiceRequest, 123, {0x01}, 9);

  auto rejected = session.handle_frame(request, Lane::Realtime);

  EXPECT_TRUE(rejected.empty());
}

}  // namespace horus_unity_bridge::horuslink
