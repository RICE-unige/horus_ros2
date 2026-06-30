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

#include "horus_unity_bridge/horuslink_channel_table.hpp"

#include <limits>
#include <stdexcept>

namespace horus_unity_bridge::horuslink
{

bool ChannelDescriptor::operator==(const ChannelDescriptor & other) const
{
  return channel_id == other.channel_id &&
         topic == other.topic &&
         type_name == other.type_name &&
         lane == other.lane &&
         delivery == other.delivery &&
         state == other.state;
}

bool ChannelTable::begin_subscribe(
  uint16_t channel_id,
  std::string topic,
  std::string type_name,
  Lane lane,
  Delivery delivery)
{
  if (topic.empty() || type_name.empty()) {
    return false;
  }

  const auto existing_channel = channels_.find(channel_id);
  if (existing_channel != channels_.end() &&
    (existing_channel->second.state == ChannelState::PendingSubscribe ||
    existing_channel->second.state == ChannelState::Active))
  {
    return false;
  }

  const auto existing_topic = topic_to_channel_.find(topic);
  if (existing_topic != topic_to_channel_.end() && existing_topic->second != channel_id) {
    return false;
  }

  channels_[channel_id] = ChannelDescriptor{
    channel_id,
    topic,
    type_name,
    lane,
    delivery,
    ChannelState::PendingSubscribe
  };
  topic_to_channel_[channels_[channel_id].topic] = channel_id;
  return true;
}

bool ChannelTable::confirm_subscribe_ack(uint16_t channel_id)
{
  auto channel = channels_.find(channel_id);
  if (channel == channels_.end()) {
    return false;
  }
  if (channel->second.state == ChannelState::Active) {
    return true;
  }
  if (channel->second.state != ChannelState::PendingSubscribe) {
    return false;
  }

  channel->second.state = ChannelState::Active;
  return true;
}

bool ChannelTable::can_accept_data_frame(const FrameHeader & header, Lane lane) const
{
  const auto channel = channels_.find(header.channel_id);
  return channel != channels_.end() &&
         channel->second.state == ChannelState::Active &&
         channel->second.lane == lane;
}

std::optional<ChannelDescriptor> ChannelTable::get(uint16_t channel_id) const
{
  const auto channel = channels_.find(channel_id);
  if (channel == channels_.end()) {
    return std::nullopt;
  }

  return channel->second;
}

std::optional<ChannelDescriptor> ChannelTable::get_by_topic(const std::string & topic) const
{
  const auto topic_entry = topic_to_channel_.find(topic);
  if (topic_entry == topic_to_channel_.end()) {
    return std::nullopt;
  }

  return get(topic_entry->second);
}

bool ChannelTable::remove(uint16_t channel_id)
{
  auto channel = channels_.find(channel_id);
  if (channel == channels_.end()) {
    return false;
  }

  topic_to_channel_.erase(channel->second.topic);
  channels_.erase(channel);
  return true;
}

bool PendingServiceRequest::is_expired(int64_t now_ms) const
{
  return timeout_ms >= 0 && now_ms - created_at_ms >= timeout_ms;
}

bool PendingServiceRequest::operator==(const PendingServiceRequest & other) const
{
  return corr_id == other.corr_id &&
         channel_id == other.channel_id &&
         created_at_ms == other.created_at_ms &&
         timeout_ms == other.timeout_ms;
}

uint32_t ServiceTable::register_request(
  uint16_t channel_id,
  int64_t now_ms,
  int32_t timeout_ms)
{
  if (timeout_ms < -1) {
    throw std::out_of_range("HorusLink service timeout must be non-negative, or -1.");
  }

  const uint32_t corr_id = allocate_corr_id();
  pending_.emplace(corr_id, PendingServiceRequest{corr_id, channel_id, now_ms, timeout_ms});
  return corr_id;
}

std::optional<PendingServiceRequest> ServiceTable::complete(uint32_t corr_id)
{
  if (corr_id == 0) {
    return std::nullopt;
  }

  auto request = pending_.find(corr_id);
  if (request == pending_.end()) {
    return std::nullopt;
  }

  PendingServiceRequest completed = request->second;
  pending_.erase(request);
  return completed;
}

std::vector<PendingServiceRequest> ServiceTable::expire(int64_t now_ms)
{
  std::vector<PendingServiceRequest> expired;
  for (auto it = pending_.begin(); it != pending_.end(); ) {
    if (!it->second.is_expired(now_ms)) {
      ++it;
      continue;
    }

    expired.push_back(it->second);
    it = pending_.erase(it);
  }

  return expired;
}

void ServiceTable::clear()
{
  pending_.clear();
}

uint32_t ServiceTable::allocate_corr_id()
{
  for (uint32_t attempt = 0; attempt < std::numeric_limits<uint32_t>::max(); ++attempt) {
    uint32_t candidate = next_corr_id_++;
    if (candidate == 0) {
      candidate = next_corr_id_++;
    }

    if (pending_.find(candidate) == pending_.end()) {
      return candidate;
    }
  }

  throw std::runtime_error("No HorusLink service correlation IDs are available.");
}

}  // namespace horus_unity_bridge::horuslink
