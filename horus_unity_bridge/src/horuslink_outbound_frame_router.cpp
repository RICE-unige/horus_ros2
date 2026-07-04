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

#include "horus_unity_bridge/horuslink_outbound_frame_router.hpp"

#include <limits>
#include <utility>

namespace horus_unity_bridge::horuslink
{

OutboundFrameRouter::OutboundFrameRouter(size_t realtime_depth, size_t bulk_depth)
: realtime_queue_(realtime_depth),
  bulk_queue_(bulk_depth)
{
}

OutboundFrameRouteResult OutboundFrameRouter::enqueue_data(
  const ChannelTable & channel_table,
  const std::string & topic,
  const uint8_t * payload,
  size_t size)
{
  return enqueue_payload(channel_table, topic, MessageType::Data, 0, payload, size);
}

OutboundFrameRouteResult OutboundFrameRouter::enqueue_payload(
  const ChannelTable & channel_table,
  const std::string & topic,
  MessageType message_type,
  uint32_t corr_id,
  const uint8_t * payload,
  size_t size)
{
  const auto channel = channel_table.get_by_topic(topic);
  if (!channel.has_value()) {
    return make_error_result(nullptr, "topic is not subscribed");
  }
  if (channel->state != ChannelState::Active) {
    return make_error_result(&*channel, "channel is not active");
  }
  if (requires_correlation(message_type) && corr_id == 0) {
    return make_error_result(&*channel, "service frames require a non-zero correlation id");
  }
  if (size > std::numeric_limits<uint32_t>::max()) {
    return make_error_result(&*channel, "payload exceeds HorusLink uint32 length");
  }
  if (payload == nullptr && size > 0) {
    return make_error_result(&*channel, "payload pointer is null");
  }

  Frame frame;
  frame.header.channel_id = channel->channel_id;
  frame.header.msg_type = message_type;
  frame.header.flags = flags_for_delivery(channel->delivery);
  frame.header.seq = next_sequence_for(channel->channel_id);
  frame.header.corr_id = corr_id;
  frame.header.length = static_cast<uint32_t>(size);
  if (size > 0) {
    frame.payload.assign(payload, payload + size);
  }

  auto & queue = queue_for(channel->lane);
  auto enqueue_result = queue.enqueue(std::move(frame));

  OutboundFrameRouteResult result;
  result.accepted = enqueue_result.accepted;
  result.replaced = enqueue_result.replaced;
  result.overflow = enqueue_result.overflow;
  result.lane = channel->lane;
  result.delivery = channel->delivery;
  result.channel_id = channel->channel_id;
  result.seq = next_sequence_for(channel->channel_id);
  result.realtime_depth = realtime_queue_.size();
  result.bulk_depth = bulk_queue_.size();
  result.replaced_frame = std::move(enqueue_result.replaced_frame);
  if (result.accepted) {
    advance_sequence(channel->channel_id);
  } else if (result.overflow) {
    result.error = "lane queue overflow";
  }

  return result;
}

std::optional<RoutedOutboundFrame> OutboundFrameRouter::pop(Lane lane)
{
  auto frame = queue_for(lane).pop();
  if (!frame.has_value()) {
    return std::nullopt;
  }

  return RoutedOutboundFrame{lane, std::move(*frame)};
}

void OutboundFrameRouter::clear()
{
  realtime_queue_.clear();
  bulk_queue_.clear();
}

OutboundLaneQueue & OutboundFrameRouter::queue_for(Lane lane)
{
  return lane == Lane::Bulk ? bulk_queue_ : realtime_queue_;
}

const OutboundLaneQueue & OutboundFrameRouter::queue_for(Lane lane) const
{
  return lane == Lane::Bulk ? bulk_queue_ : realtime_queue_;
}

uint32_t OutboundFrameRouter::next_sequence_for(uint16_t channel_id) const
{
  const auto sequence = next_sequence_by_channel_.find(channel_id);
  if (sequence == next_sequence_by_channel_.end()) {
    return 1;
  }

  return sequence->second;
}

void OutboundFrameRouter::advance_sequence(uint16_t channel_id)
{
  uint32_t next = next_sequence_for(channel_id) + 1;
  if (next == 0) {
    next = 1;
  }

  next_sequence_by_channel_[channel_id] = next;
}

OutboundFrameRouteResult OutboundFrameRouter::make_error_result(
  const ChannelDescriptor * channel,
  const std::string & error) const
{
  OutboundFrameRouteResult result;
  if (channel != nullptr) {
    result.lane = channel->lane;
    result.delivery = channel->delivery;
    result.channel_id = channel->channel_id;
    result.seq = next_sequence_for(channel->channel_id);
  }
  result.realtime_depth = realtime_queue_.size();
  result.bulk_depth = bulk_queue_.size();
  result.error = error;
  return result;
}

bool OutboundFrameRouter::requires_correlation(MessageType message_type)
{
  return message_type == MessageType::ServiceRequest ||
         message_type == MessageType::ServiceResponse;
}

uint8_t OutboundFrameRouter::flags_for_delivery(Delivery delivery)
{
  uint8_t flags = FrameFlags::RawOpaque;
  if (delivery == Delivery::ReplaceLatest) {
    flags |= FrameFlags::ReplaceLatest;
  }

  return flags;
}

}  // namespace horus_unity_bridge::horuslink
