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

#include "horus_unity_bridge/horuslink_protocol.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace horus_unity_bridge::horuslink
{

enum class Lane : uint8_t
{
  Realtime = 1,
  Bulk = 2
};

enum class Delivery : uint8_t
{
  ReliableFifo = 1,
  ReplaceLatest = 2
};

enum class ChannelState : uint8_t
{
  Empty = 0,
  PendingSubscribe = 1,
  Active = 2,
  Closed = 3
};

struct ChannelDescriptor
{
  uint16_t channel_id = 0;
  std::string topic;
  std::string type_name;
  Lane lane = Lane::Realtime;
  Delivery delivery = Delivery::ReliableFifo;
  ChannelState state = ChannelState::Empty;

  bool operator==(const ChannelDescriptor & other) const;
};

class ChannelTable
{
public:
  bool begin_subscribe(
    uint16_t channel_id,
    std::string topic,
    std::string type_name,
    Lane lane,
    Delivery delivery);

  bool confirm_subscribe_ack(uint16_t channel_id);
  bool can_accept_data_frame(const FrameHeader & header, Lane lane) const;
  std::optional<ChannelDescriptor> get(uint16_t channel_id) const;
  std::optional<ChannelDescriptor> get_by_topic(const std::string & topic) const;
  bool remove(uint16_t channel_id);

private:
  std::unordered_map<uint16_t, ChannelDescriptor> channels_;
  std::unordered_map<std::string, uint16_t> topic_to_channel_;
};

struct PendingServiceRequest
{
  uint32_t corr_id = 0;
  uint16_t channel_id = 0;
  int64_t created_at_ms = 0;
  int32_t timeout_ms = 0;

  bool is_expired(int64_t now_ms) const;
  bool operator==(const PendingServiceRequest & other) const;
};

class ServiceTable
{
public:
  uint32_t register_request(uint16_t channel_id, int64_t now_ms, int32_t timeout_ms);
  std::optional<PendingServiceRequest> complete(uint32_t corr_id);
  std::vector<PendingServiceRequest> expire(int64_t now_ms);
  void clear();
  size_t pending_count() const {return pending_.size();}

private:
  uint32_t allocate_corr_id();

  uint32_t next_corr_id_ = 1;
  std::unordered_map<uint32_t, PendingServiceRequest> pending_;
};

}  // namespace horus_unity_bridge::horuslink
