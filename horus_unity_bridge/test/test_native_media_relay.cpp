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

#include "horus_unity_bridge/rtp_relay.hpp"
#include "horus_unity_bridge/native_media_relay.hpp"
#include <gtest/gtest.h>
#include <array>

namespace horus_unity_bridge::media
{
TEST(NativeRtpRelay, DetectsOnlyForwardSequenceGapsAcrossWrap)
{
  RtpSequenceGaps gaps;
  EXPECT_EQ(gaps.observe(10, 65534), 0);
  EXPECT_EQ(gaps.observe(10, 65535), 0);
  EXPECT_EQ(gaps.observe(10, 0), 0);
  EXPECT_EQ(gaps.observe(10, 2), 1);
  EXPECT_EQ(gaps.observe(10, 7), 4);
}

TEST(NativeRtpRelay, IgnoresDuplicateAndLateRepairsAndResetsOnNewSsrc)
{
  RtpSequenceGaps gaps;
  EXPECT_EQ(gaps.observe(10, 100), 0);
  EXPECT_EQ(gaps.observe(10, 100), 0);
  EXPECT_EQ(gaps.observe(10, 99), 0);
  EXPECT_EQ(gaps.observe(10, 101), 0);
  EXPECT_EQ(gaps.observe(11, 5000), 0);
  EXPECT_EQ(gaps.observe(11, 5001), 0);
  gaps = {};
  EXPECT_EQ(gaps.observe(11, 12345), 0);
}

#ifdef ENABLE_WEBRTC
TEST(NativeRtpRelay, ValidatesConfigurationBeforeStartingPeers)
{
  nlohmann::json config = {{"streams", {{{"input_topic", "/camera"}, {"output_topic", "/relay"}}}}};
  const auto valid = NativeMediaRelay::validate_config(config);
  EXPECT_EQ(valid["streams"][0]["id"], "camera_0");
  for (auto value : {0, 61, -1}) {
    config["streams"][0]["framerate"] = value;
    EXPECT_THROW(NativeMediaRelay::validate_config(config), std::invalid_argument);
  }
  config["streams"][0].erase("framerate");
  config["streams"].push_back(config["streams"][0]);
  EXPECT_THROW(NativeMediaRelay::validate_config(config), std::invalid_argument);
  config["streams"] = nlohmann::json::array();
  EXPECT_THROW(NativeMediaRelay::validate_config(config), std::invalid_argument);
}
TEST(NativeRtpRelay, InheritsOptionalIceSettingsWithoutChangingLegacyDefaults)
{
  nlohmann::json config = {{"streams", {{{"input_topic", "/camera"}, {"output_topic", "/relay"}}}}};
  const auto legacy = NativeMediaRelay::validate_config(config);
  EXPECT_FALSE(legacy["streams"][0].contains("ice_servers"));
  config["ice_servers"] = {"stun:127.0.0.1:3478", "turn://user:password@127.0.0.1:3478"};
  config["ice_transport_policy"] = "relay";
  const auto inherited = NativeMediaRelay::validate_config(config);
  EXPECT_EQ(inherited["streams"][0]["ice_servers"], config["ice_servers"]);
  EXPECT_EQ(inherited["streams"][0]["ice_transport_policy"], "relay");
  config["streams"][0]["ice_transport_policy"] = "all";
  EXPECT_EQ(NativeMediaRelay::validate_config(config)["streams"][0]["ice_transport_policy"], "all");
}

TEST(NativeRtpRelay, RejectsUnusableIcePolicyAndServersBeforeOpeningNetwork)
{
  const nlohmann::json base = {{"streams", {{{"input_topic", "/camera"}, {"output_topic", "/relay"}}}}};
  for (const auto & servers : {nlohmann::json("bad"), nlohmann::json({123}),
      nlohmann::json({"https://invalid"}), nlohmann::json({"turn://bad\nvalue"})}) {
    auto config = base;
    config["ice_servers"] = servers;
    EXPECT_THROW(NativeMediaRelay::validate_config(config), std::invalid_argument);
  }
  auto config = base;
  config["ice_transport_policy"] = "relay";
  EXPECT_THROW(NativeMediaRelay::validate_config(config), std::invalid_argument);
  config["ice_transport_policy"] = "guess";
  EXPECT_THROW(NativeMediaRelay::validate_config(config), std::invalid_argument);
}

TEST(NativeRtpRelay, ValidatesAndInheritsOptionalPacketGapRecoverySwitch)
{
  nlohmann::json config = {{"streams", {{{"input_topic", "/camera"}, {"output_topic", "/relay"}}}}};
  config["recover_on_packet_gap"] = false;
  EXPECT_EQ(NativeMediaRelay::validate_config(config)["streams"][0]["recover_on_packet_gap"], false);
  config["streams"][0]["recover_on_packet_gap"] = true;
  EXPECT_EQ(NativeMediaRelay::validate_config(config)["streams"][0]["recover_on_packet_gap"], true);
  config["streams"][0]["recover_on_packet_gap"] = "yes";
  EXPECT_THROW(NativeMediaRelay::validate_config(config), std::invalid_argument);
}
#endif
TEST(NativeRtpRelay, RetainsPayloadTimingAndSequenceGapsAcrossWrap)
{
  std::array<uint8_t, 16> source{0x80, 0xe0, 0xff, 0xfe, 0xff, 0xff, 0xff, 0xf0,
    0, 0, 0, 2, 0x7c, 0x85, 0xaa, 0xbb};
  auto first = parse_rtp(source.data(), source.size());
  ASSERT_TRUE(first);
  auto output = translate_rtp(*first, 111, 0x12345678, 5, 64);
  auto translated = parse_rtp(output.data(), output.size());
  ASSERT_TRUE(translated);
  EXPECT_EQ(translated->sequence, 3);
  EXPECT_EQ(translated->timestamp, 48U);
  EXPECT_EQ(read_u32(output.data() + 8), 0x12345678U);
  EXPECT_EQ(output[1], uint8_t(128 | 111));
  EXPECT_EQ(std::vector<uint8_t>(output.begin() + 12, output.end()),
    std::vector<uint8_t>(source.begin() + 12, source.end()));
  source[2] = 0; source[3] = 1;  // Two missing source packets must not disappear.
  write_u32(source.data() + 4, 9000);
  auto next = parse_rtp(source.data(), source.size());
  ASSERT_TRUE(next);
  auto next_output = translate_rtp(*next, 111, 0x12345678, 5, 64);
  auto next_packet = parse_rtp(next_output.data(), next_output.size());
  EXPECT_EQ(uint16_t(next_packet->sequence - translated->sequence), 3);
  EXPECT_EQ(uint32_t(next_packet->timestamp - translated->timestamp), 9016U);
}

TEST(NativeRtpRelay, RemovesHopLocalExtensionsAndPaddingNotPictureData)
{
  const std::array<uint8_t, 26> source{0xb1, 96, 0, 1, 0, 0, 0, 2,
    0, 0, 0, 3, 0, 0, 0, 7, 0xbe, 0xde, 0, 1, 0x11, 0xff, 0, 0, 0x65, 1};
  auto packet = parse_rtp(source.data(), source.size());
  ASSERT_TRUE(packet);
  ASSERT_EQ(packet->size, 1U);
  auto output = translate_rtp(*packet, 96, 2, 0, 0);
  ASSERT_EQ(output.size(), 13U);
  EXPECT_EQ(output[0], 128);
  EXPECT_EQ(output[12], 0x65);
}

TEST(NativeRtpRelay, RejectsTruncatedExtensionsRtcpAndInvalidPadding)
{
  std::array<uint8_t, 16> packet{0x90, 96, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0xbe, 0xde, 0xff, 0xff};
  EXPECT_FALSE(parse_rtp(packet.data(), packet.size()));
  packet[0] = 0xa0; packet[15] = 0;
  EXPECT_FALSE(parse_rtp(packet.data(), packet.size()));
  packet[15] = 5;
  EXPECT_FALSE(parse_rtp(packet.data(), packet.size()));
  packet[0] = 0x80; packet[1] = 200;
  EXPECT_FALSE(parse_rtp(packet.data(), packet.size()));
  EXPECT_FALSE(parse_rtp(nullptr, 100));
}
}  // namespace horus_unity_bridge::media
