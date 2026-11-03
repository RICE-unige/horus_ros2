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

#pragma once

#include <memory>
#include <chrono>
#include <map>
#include <mutex>
#include <deque>
#include <rtc/rtcpnackresponder.hpp>

namespace horus_unity_bridge
{

// The pinned libdatachannel caches shared message buffers, while its SRTP
// transport encrypts those buffers in place. Retain independent plaintext so
// retransmission encrypts the original RTP payload, not ciphertext a second time.
class PlaintextRtcpNackResponder : public rtc::MediaHandler
{
public:
  explicit PlaintextRtcpNackResponder(
    std::size_t capacity = rtc::RtcpNackResponder::DefaultMaxSize,
    std::chrono::milliseconds maximum_age = std::chrono::milliseconds(250),
    std::function<void(rtc::message_ptr, const rtc::message_callback &)> repair_output = {},
    std::function<std::size_t()> repair_allowance = {})
  : responder_(capacity), capacity_(capacity), maximum_age_(maximum_age),
    repair_output_(std::move(repair_output)), repair_allowance_(std::move(repair_allowance)) {}

  void incoming(
    rtc::message_vector & messages,
    const rtc::message_callback & send) override
  {
    responder_.incoming(messages, [&](rtc::message_ptr packet) {
      if (!packet || packet->size() < 12) {return;}
      const auto * bytes = reinterpret_cast<const uint8_t *>(packet->data());
      const uint16_t sequence = (uint16_t(bytes[2]) << 8) | bytes[3];
      {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto now = std::chrono::steady_clock::now();
        const auto found = times_.find(sequence);
        if (found == times_.end() || now - found->second > maximum_age_) {return;}
        if (now - repair_window_ > std::chrono::milliseconds(100)) {
          repair_window_ = now; repair_bytes_ = 0;
        }
        const auto allowance = repair_allowance_ ? std::min<std::size_t>(32 * 1024, repair_allowance_()) : 32 * 1024;
        if (repair_bytes_ + packet->size() > allowance) {return;}
        repair_bytes_ += packet->size();
      }
      if (repair_output_) {repair_output_(std::move(packet), send);} else {send(std::move(packet));}
    });
  }

  void outgoing(
    rtc::message_vector & messages,
    const rtc::message_callback & send) override
  {
    rtc::message_vector retained;
    retained.reserve(messages.size());
    for (const auto & message : messages) {
      if (message && message->type != rtc::Message::Control) {
        retained.push_back(std::make_shared<rtc::Message>(*message));
        if (message->size() >= 12) {
          const auto * bytes = reinterpret_cast<const uint8_t *>(message->data());
          const uint16_t sequence = (uint16_t(bytes[2]) << 8) | bytes[3];
          const auto now = std::chrono::steady_clock::now();
          std::lock_guard<std::mutex> lock(mutex_);
          times_[sequence] = now;
          order_.emplace_back(sequence, now);
          while (order_.size() > capacity_) {
            const auto first = order_.front(); order_.pop_front();
            const auto found = times_.find(first.first);
            if (found != times_.end() && found->second == first.second) {times_.erase(found);}
          }
        }
      }
    }
    responder_.outgoing(retained, send);
  }

private:
  rtc::RtcpNackResponder responder_;
  std::size_t capacity_, repair_bytes_ = 0;
  std::chrono::milliseconds maximum_age_;
  std::function<void(rtc::message_ptr, const rtc::message_callback &)> repair_output_;
  std::function<std::size_t()> repair_allowance_;
  std::mutex mutex_;
  std::chrono::steady_clock::time_point repair_window_;
  std::map<uint16_t, std::chrono::steady_clock::time_point> times_;
  std::deque<std::pair<uint16_t, std::chrono::steady_clock::time_point>> order_;
};

}  // namespace horus_unity_bridge
