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

#include "horus_unity_bridge/horuslink_channel_table.hpp"
#include "horus_unity_bridge/horuslink_outbound_lane_queue.hpp"
#include "horus_unity_bridge/horuslink_protocol.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>

namespace horus_unity_bridge::horuslink
{

struct RoutedOutboundFrame
{
  Lane lane = Lane::Realtime;
  Frame frame;
};

struct OutboundFrameRouteResult
{
  bool accepted = false;
  bool replaced = false;
  bool overflow = false;
  Lane lane = Lane::Realtime;
  uint16_t channel_id = 0;
  uint32_t seq = 0;
  size_t realtime_depth = 0;
  size_t bulk_depth = 0;
  std::optional<Frame> replaced_frame;
  std::string error;
};

class OutboundFrameRouter
{
public:
  OutboundFrameRouter(size_t realtime_depth, size_t bulk_depth);

  OutboundFrameRouteResult enqueue_data(
    const ChannelTable & channel_table,
    const std::string & topic,
    const uint8_t * payload,
    size_t size);

  OutboundFrameRouteResult enqueue_payload(
    const ChannelTable & channel_table,
    const std::string & topic,
    MessageType message_type,
    uint32_t corr_id,
    const uint8_t * payload,
    size_t size);

  std::optional<RoutedOutboundFrame> pop(Lane lane);
  void clear();

  size_t realtime_depth() const {return realtime_queue_.size();}
  size_t bulk_depth() const {return bulk_queue_.size();}

private:
  OutboundLaneQueue & queue_for(Lane lane);
  const OutboundLaneQueue & queue_for(Lane lane) const;
  uint32_t next_sequence_for(uint16_t channel_id) const;
  void advance_sequence(uint16_t channel_id);
  OutboundFrameRouteResult make_error_result(
    const ChannelDescriptor * channel,
    const std::string & error) const;

  static bool requires_correlation(MessageType message_type);
  static uint8_t flags_for_delivery(Delivery delivery);

  OutboundLaneQueue realtime_queue_;
  OutboundLaneQueue bulk_queue_;
  std::unordered_map<uint16_t, uint32_t> next_sequence_by_channel_;
};

}  // namespace horus_unity_bridge::horuslink
