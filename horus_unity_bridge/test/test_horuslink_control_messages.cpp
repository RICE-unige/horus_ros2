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

#include "horus_unity_bridge/horuslink_control_messages.hpp"
#include "horuslink_golden_vectors.hpp"

#include <gtest/gtest.h>

namespace horus_unity_bridge::horuslink
{

TEST(HorusLinkControlMessagesTest, HelloEncodesStableGoldenVector)
{
  const auto encoded = encode_hello(HelloMessage{EndpointRole::UnityClient, 0x01020304, 1000});
  const auto expected = load_horuslink_golden_vector("hello");
  EXPECT_EQ(encoded, expected);

  auto decoded = decode_hello(encoded.data(), encoded.size());
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(*decoded, (HelloMessage{EndpointRole::UnityClient, 0x01020304, 1000}));
}

TEST(HorusLinkControlMessagesTest, SubscribeRequestEncodesStableGoldenVector)
{
  const SubscribeRequest request{
    7,
    "/tf",
    "tf2_msgs/msg/TFMessage",
    Lane::Realtime,
    Delivery::ReliableFifo
  };
  const auto encoded = encode_subscribe_request(request);
  const auto expected = load_horuslink_golden_vector("subscribe_request");
  EXPECT_EQ(encoded, expected);

  auto decoded = decode_subscribe_request(encoded.data(), encoded.size());
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(*decoded, request);
}

TEST(HorusLinkControlMessagesTest, PublisherRequestEncodesStableGoldenVector)
{
  const PublisherRequest request{
    12,
    "/cmd_vel",
    "geometry_msgs/msg/Twist",
    Lane::Realtime,
    Delivery::ReliableFifo
  };
  const auto encoded = encode_publisher_request(request);
  const auto expected = load_horuslink_golden_vector("publisher_request");
  EXPECT_EQ(encoded, expected);

  auto decoded = decode_publisher_request(encoded.data(), encoded.size());
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(*decoded, request);
}

TEST(HorusLinkControlMessagesTest, SubscribeAckRoundTripsAcceptedAndRejectedMessages)
{
  const SubscribeAck accepted{9, SubscribeStatus::Accepted, ""};
  const auto accepted_bytes = encode_subscribe_ack(accepted);
  EXPECT_EQ(accepted_bytes, load_horuslink_golden_vector("subscribe_ack_accepted"));
  auto accepted_decoded = decode_subscribe_ack(accepted_bytes.data(), accepted_bytes.size());
  ASSERT_TRUE(accepted_decoded.has_value());
  EXPECT_EQ(*accepted_decoded, accepted);

  const SubscribeAck rejected{9, SubscribeStatus::Rejected, "unknown topic"};
  const auto rejected_bytes = encode_subscribe_ack(rejected);
  EXPECT_EQ(rejected_bytes, load_horuslink_golden_vector("subscribe_ack_rejected"));
  auto rejected_decoded = decode_subscribe_ack(rejected_bytes.data(), rejected_bytes.size());
  ASSERT_TRUE(rejected_decoded.has_value());
  EXPECT_EQ(*rejected_decoded, rejected);
}

TEST(HorusLinkControlMessagesTest, TopicTableRoundTripsRepeatedEntries)
{
  const std::vector<TopicEntry> entries{
    TopicEntry{1, "/tf", "tf2_msgs/msg/TFMessage", Lane::Realtime, Delivery::ReliableFifo},
    TopicEntry{2, "/camera", "sensor_msgs/msg/Image", Lane::Bulk, Delivery::ReplaceLatest}
  };

  const auto encoded = encode_topic_table(entries);
  EXPECT_EQ(encoded, load_horuslink_golden_vector("topic_table"));
  auto decoded = decode_topic_table(encoded.data(), encoded.size());
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(*decoded, entries);
}

TEST(HorusLinkControlMessagesTest, WrongKindOrVersionIsRejected)
{
  auto encoded = encode_hello(HelloMessage{EndpointRole::Bridge, 1, 1});
  EXPECT_FALSE(decode_subscribe_request(encoded.data(), encoded.size()).has_value());

  encoded[9] = 0x02;
  EXPECT_FALSE(decode_hello(encoded.data(), encoded.size()).has_value());
}

}  // namespace horus_unity_bridge::horuslink
