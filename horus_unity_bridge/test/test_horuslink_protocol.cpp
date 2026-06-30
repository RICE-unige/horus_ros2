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

#include "horus_unity_bridge/horuslink_protocol.hpp"
#include "horuslink_golden_vectors.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <vector>

namespace horus_unity_bridge::horuslink
{

TEST(HorusLinkProtocolTest, HeaderEncodesLittleEndianFixedSixteenByteFrame)
{
  FrameHeader header;
  header.channel_id = 0x1234;
  header.msg_type = MessageType::Data;
  header.flags = RawOpaque | ReplaceLatest;
  header.seq = 0x11223344;
  header.corr_id = 0x55667788;
  header.length = 0x99AABBCC;

  std::array<uint8_t, FrameHeader::kSize> bytes{};
  ASSERT_TRUE(encode_header(header, bytes));

  const auto expected = load_horuslink_golden_vector("frame_header");
  ASSERT_EQ(expected.size(), bytes.size());
  EXPECT_TRUE(std::equal(bytes.begin(), bytes.end(), expected.begin(), expected.end()));

  FrameHeader decoded;
  ASSERT_TRUE(decode_header(bytes.data(), bytes.size(), decoded));
  EXPECT_EQ(decoded, header);
}

TEST(HorusLinkProtocolTest, HeaderRejectsShortInput)
{
  FrameHeader decoded;
  std::array<uint8_t, FrameHeader::kSize - 1> bytes{};
  EXPECT_FALSE(decode_header(bytes.data(), bytes.size(), decoded));
}

TEST(HorusLinkProtocolTest, TlvCodecRoundTripsRecordsInOrder)
{
  const std::vector<TlvRecord> records{
    TlvRecord{1, std::vector<uint8_t>{0x10, 0x11}},
    TlvRecord{0x1234, std::vector<uint8_t>{}},
    TlvRecord{2, std::vector<uint8_t>{0x20, 0x21, 0x22}}
  };

  const auto encoded = encode_tlvs(records);
  const auto expected = load_horuslink_golden_vector("tlv_records");
  EXPECT_EQ(encoded, expected);

  std::vector<TlvRecord> decoded;
  ASSERT_TRUE(decode_tlvs(encoded.data(), encoded.size(), decoded));
  EXPECT_EQ(decoded, records);
}

TEST(HorusLinkProtocolTest, TlvCodecRejectsTruncatedRecords)
{
  const std::vector<uint8_t> truncated_header{0x01, 0x00, 0x02};
  std::vector<TlvRecord> decoded;
  EXPECT_FALSE(decode_tlvs(truncated_header.data(), truncated_header.size(), decoded));
  EXPECT_TRUE(decoded.empty());

  const std::vector<uint8_t> truncated_value{0x01, 0x00, 0x02, 0x00, 0xFF};
  decoded.clear();
  EXPECT_FALSE(decode_tlvs(truncated_value.data(), truncated_value.size(), decoded));
  EXPECT_TRUE(decoded.empty());
}

TEST(HorusLinkProtocolTest, FrameParserHandlesPartialAndCoalescedReads)
{
  const std::vector<uint8_t> payload_a{0xA0, 0xA1, 0xA2};
  FrameHeader header_a;
  header_a.channel_id = 7;
  header_a.msg_type = MessageType::Data;
  header_a.flags = RawOpaque;
  header_a.seq = 10;
  header_a.corr_id = 0;
  auto frame_a = serialize_frame(header_a, payload_a.data(), payload_a.size());

  const std::vector<uint8_t> payload_b{0xB0};
  FrameHeader header_b;
  header_b.channel_id = 8;
  header_b.msg_type = MessageType::ServiceResponse;
  header_b.flags = 0;
  header_b.seq = 11;
  header_b.corr_id = 99;
  auto frame_b = serialize_frame(header_b, payload_b.data(), payload_b.size());

  std::vector<uint8_t> combined;
  combined.insert(combined.end(), frame_a.begin(), frame_a.end());
  combined.insert(combined.end(), frame_b.begin(), frame_b.end());

  FrameParser parser;
  std::vector<Frame> frames;
  size_t consumed = parser.parse(combined.data(), 5, frames);
  EXPECT_EQ(consumed, 5u);
  EXPECT_TRUE(frames.empty());
  EXPECT_FALSE(parser.has_error());

  consumed = parser.parse(combined.data() + 5, combined.size() - 5, frames);
  EXPECT_EQ(consumed, combined.size() - 5);
  ASSERT_EQ(frames.size(), 2u);
  EXPECT_EQ(frames[0].header.channel_id, 7);
  EXPECT_EQ(frames[0].header.length, payload_a.size());
  EXPECT_EQ(frames[0].payload, payload_a);
  EXPECT_EQ(frames[1].header.channel_id, 8);
  EXPECT_EQ(frames[1].header.corr_id, 99u);
  EXPECT_EQ(frames[1].payload, payload_b);
}

TEST(HorusLinkProtocolTest, FrameParserRejectsOversizedPayload)
{
  FrameHeader header;
  header.length = 16;
  std::array<uint8_t, FrameHeader::kSize> bytes{};
  encode_header(header, bytes);

  FrameParser parser(8);
  std::vector<Frame> frames;
  EXPECT_EQ(parser.parse(bytes.data(), bytes.size(), frames), bytes.size());
  EXPECT_TRUE(parser.has_error());
  EXPECT_TRUE(frames.empty());
}

}  // namespace horus_unity_bridge::horuslink
