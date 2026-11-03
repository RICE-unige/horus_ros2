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

#ifdef ENABLE_WEBRTC

#include <array>
#include <cstddef>
#include <cstdint>
#include <gtest/gtest.h>
#include <thread>

#include "horus_unity_bridge/plaintext_rtcp_nack_responder.hpp"
#include "horus_unity_bridge/webrtc_packet_budget.hpp"
#include <rtc/h264rtppacketizer.hpp>

namespace
{

rtc::message_ptr make_rtp(uint16_t sequence)
{
  auto packet = rtc::make_message(16);
  const std::array<uint8_t, 16> bytes = {
    0x80, 124, static_cast<uint8_t>(sequence >> 8), static_cast<uint8_t>(sequence),
    0, 0, 0, 1, 0, 0, 0, 2, 0x67, 0x42, 0xe0, 0x33};
  for (std::size_t i = 0; i < bytes.size(); ++i) {
    (*packet)[i] = static_cast<std::byte>(bytes[i]);
  }
  return packet;
}

rtc::message_vector make_nack(uint16_t sequence)
{
  auto packet = rtc::make_message(16, rtc::Message::Control);
  const std::array<uint8_t, 16> bytes = {
    0x81, 205, 0, 3, 0, 0, 0, 1, 0, 0, 0, 2,
    static_cast<uint8_t>(sequence >> 8), static_cast<uint8_t>(sequence), 0, 0};
  for (std::size_t i = 0; i < bytes.size(); ++i) {
    (*packet)[i] = static_cast<std::byte>(bytes[i]);
  }
  return {packet};
}

TEST(PlaintextRtcpNackResponder, RetainedPayloadSurvivesTransportMutation)
{
  horus_unity_bridge::PlaintextRtcpNackResponder responder;
  auto original = make_rtp(0x1234);
  const rtc::binary plaintext(*original);
  rtc::message_vector outbound{original};
  responder.outgoing(outbound, [](rtc::message_ptr) {});
  ASSERT_EQ(outbound.front(), original);

  // DtlsSrtpTransport encrypts and extends this same outgoing buffer in place.
  (*original)[12] = std::byte{0xaa};
  original->resize(original->size() + 10);
  auto feedback = make_nack(0x1234);
  int retransmits = 0;
  const auto capture = [&](rtc::message_ptr packet) {
      EXPECT_EQ(static_cast<const rtc::binary &>(*packet), plaintext);
      ++retransmits;
      // A retransmission is also encrypted in place; the cache must survive it.
      (*packet)[12] = std::byte{0xbb};
    };
  responder.incoming(feedback, capture);
  responder.incoming(feedback, capture);
  EXPECT_EQ(retransmits, 2);
}

TEST(PlaintextRtcpNackResponder, PreservedBoundedCacheAndSequenceWrap)
{
  horus_unity_bridge::PlaintextRtcpNackResponder responder(2);
  for (uint16_t sequence : {uint16_t{65535}, uint16_t{0}, uint16_t{1}}) {
    rtc::message_vector outbound{make_rtp(sequence)};
    responder.outgoing(outbound, [](rtc::message_ptr) {});
  }
  int retransmits = 0;
  for (uint16_t sequence : {uint16_t{65535}, uint16_t{0}, uint16_t{1}}) {
    auto feedback = make_nack(sequence);
    responder.incoming(feedback, [&](rtc::message_ptr) {++retransmits;});
  }
  EXPECT_EQ(retransmits, 2);
}

TEST(PlaintextRtcpNackResponder, IgnoresControlMessages)
{
  horus_unity_bridge::PlaintextRtcpNackResponder responder;
  auto control = make_rtp(0x1234);
  control->type = rtc::Message::Control;
  rtc::message_vector outbound{control};
  responder.outgoing(outbound, [](rtc::message_ptr) {});
  auto feedback = make_nack(0x1234);
  responder.incoming(feedback, [](rtc::message_ptr) {FAIL() << "Cached RTCP as RTP";});
}

TEST(PlaintextRtcpNackResponder, DoesNotRetransmitExpiredPictures)
{
  horus_unity_bridge::PlaintextRtcpNackResponder responder(4, std::chrono::milliseconds(10));
  rtc::message_vector outbound{make_rtp(123)};
  responder.outgoing(outbound, [](rtc::message_ptr) {});
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  auto feedback = make_nack(123);
  responder.incoming(feedback, [](rtc::message_ptr) {FAIL() << "Replayed expired video packet";});
}

TEST(PlaintextRtcpNackResponder, RepairAllowanceCannotBeBypassedByRepeatedFeedback)
{
  horus_unity_bridge::PlaintextRtcpNackResponder responder(4, std::chrono::milliseconds(250), {},
    []() {return std::size_t(16);});
  rtc::message_vector outbound{make_rtp(123)};
  responder.outgoing(outbound, [](rtc::message_ptr) {});
  int sent = 0;
  auto feedback = make_nack(123);
  responder.incoming(feedback, [&](rtc::message_ptr) {++sent;});
  responder.incoming(feedback, [&](rtc::message_ptr) {++sent;});
  EXPECT_EQ(sent, 1);
}

TEST(PlaintextRtcpNackResponder, VideoFragmentsLeaveVpnAuthenticationHeadroom)
{
  auto config = std::make_shared<rtc::RtpPacketizationConfig>(1, "budget", 99, 90000);
  rtc::H264RtpPacketizer packetizer(rtc::NalUnit::Separator::StartSequence, config,
    horus_unity_bridge::kVideoRtpMaxFragmentBytes);
  auto frame = rtc::make_message(32000);
  std::fill(frame->begin(), frame->end(), std::byte{0x55});
  (*frame)[0] = (*frame)[1] = (*frame)[2] = std::byte{0};
  (*frame)[3] = std::byte{1};
  (*frame)[4] = std::byte{0x65};
  rtc::message_vector packets{frame};
  packetizer.outgoing(packets, [](rtc::message_ptr) {});
  ASSERT_GT(packets.size(), 20u);
  for (const auto & packet : packets) {
    EXPECT_LE(packet->size(), 1112u);
    // IPv6 + UDP + a conservative SRTP authentication/extension allowance.
    EXPECT_LE(packet->size() + 40u + 8u + 64u, 1280u);
  }
}

}  // namespace

#endif  // ENABLE_WEBRTC
