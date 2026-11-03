// Copyright 2026 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0
#ifdef ENABLE_WEBRTC
#include "horus_unity_bridge/gst_bandwidth_pacer.hpp"
#include "horus_unity_bridge/rtp_relay.hpp"
#include <gtest/gtest.h>
#include <atomic>
#include <thread>
#include <mutex>

namespace horus_unity_bridge
{
#ifdef ENABLE_WEBRTC
TEST(GstBandwidthPacer, PacesOriginalPayloadAndAddsOnlyNegotiatedHopExtension)
{
  if (!GstBandwidthPacer::available()) {GTEST_SKIP() << "Optional pinned GCC plugin not on GST_PLUGIN_PATH";}
  std::mutex mutex;
  std::vector<std::vector<uint8_t>> output;
  GstBandwidthPacer pacer(1234, 96, 3, 1500, [](int) {}, []() {});
  std::vector<uint8_t> bytes(100, 0x65);
  bytes[0] = 0x80; bytes[1] = 96;
  bytes[2] = 0; bytes[3] = 1;
  media::write_u32(bytes.data() + 4, 9000);
  media::write_u32(bytes.data() + 8, 1234);
  auto message = rtc::make_message(reinterpret_cast<const std::byte *>(bytes.data()),
    reinterpret_cast<const std::byte *>(bytes.data() + bytes.size()), rtc::Message::Binary, 1234);
  rtc::message_vector messages{message};
  pacer.outgoing(messages, [&](auto message) {
      std::lock_guard<std::mutex> lock(mutex);
      const auto * p = reinterpret_cast<const uint8_t *>(message->data());
      output.emplace_back(p, p + message->size());
    });
  EXPECT_TRUE(messages.empty());
  for (int i = 0; i < 100; ++i) {
    {std::lock_guard<std::mutex> lock(mutex); if (!output.empty()) {break;}}
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  pacer.shutdown();
  std::lock_guard<std::mutex> lock(mutex);
  ASSERT_EQ(output.size(), 1U) << pacer.statistics().dump();
  const auto packet = media::parse_rtp(output[0].data(), output[0].size());
  ASSERT_TRUE(packet);
  EXPECT_EQ(packet->timestamp, 9000U);
  EXPECT_EQ(packet->sequence, 1);
  EXPECT_EQ(output[0][16] >> 4, 3);
  EXPECT_EQ(std::vector<uint8_t>(packet->payload, packet->payload + packet->size),
    std::vector<uint8_t>(bytes.begin() + 12, bytes.end()));
  EXPECT_EQ(pacer.statistics()["paced_packets"], 1);
}

TEST(GstBandwidthPacer, OriginalAndRepairsHaveIndependentPendingIdentities)
{
  if (!GstBandwidthPacer::available()) {GTEST_SKIP() << "Optional pinned GCC plugin not on GST_PLUGIN_PATH";}
  std::atomic<unsigned> sent{0}, keyframes{0};
  GstBandwidthPacer pacer(1234, 96, 3, 100, [](int) {}, [&]() {++keyframes;});
  std::vector<uint8_t> bytes(100, 0x65);
  bytes[0] = 0x80; bytes[1] = 0x80 | 96;
  bytes[2] = 0; bytes[3] = 77;
  media::write_u32(bytes.data()+4, 9000);
  media::write_u32(bytes.data()+8, 1234);
  rtc::message_vector messages;
  for (unsigned i=0; i<10; ++i) {
    messages.push_back(rtc::make_message(reinterpret_cast<const std::byte *>(bytes.data()),
      reinterpret_cast<const std::byte *>(bytes.data()+bytes.size()), rtc::Message::Binary, 1234));
  }
  pacer.outgoing(messages, [&](auto) {++sent;});
  for (unsigned i=0; i<200 && sent.load()<10; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  pacer.shutdown();
  EXPECT_EQ(sent.load(), 10U) << pacer.statistics().dump();
  EXPECT_EQ(keyframes.load(), 0U);
  EXPECT_FALSE(pacer.statistics()["waiting_idr"].get<bool>());
}
#endif
}  // namespace horus_unity_bridge
#endif
