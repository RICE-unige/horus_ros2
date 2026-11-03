// Copyright 2026 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <rtc/mediahandler.hpp>
#include <nlohmann/json.hpp>
#include <functional>
#include <memory>

namespace horus_unity_bridge
{
class GstBandwidthPacer : public rtc::MediaHandler
{
public:
  GstBandwidthPacer(uint32_t ssrc, uint8_t payload_type, int extension_id, int bitrate_kbps,
    std::function<void(int)> estimate, std::function<void()> keyframe);
  ~GstBandwidthPacer() override;
  static bool available();
  void incoming(rtc::message_vector & messages, const rtc::message_callback & send) override;
  void outgoing(rtc::message_vector & messages, const rtc::message_callback & send) override;
  void shutdown();
  nlohmann::json statistics() const;
  uint64_t frames_sent() const;
  uint64_t bytes_sent() const;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace horus_unity_bridge
