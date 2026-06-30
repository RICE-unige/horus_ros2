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
#include "horus_unity_bridge/horuslink_control_messages.hpp"
#include "horus_unity_bridge/horuslink_protocol.hpp"

#include <cstdint>
#include <optional>
#include <vector>

namespace horus_unity_bridge::horuslink
{

class Session
{
public:
  std::vector<Frame> handle_frame(const Frame & frame);
  std::vector<Frame> handle_frame(const Frame & frame, Lane lane);
  Frame make_topic_table_frame(const std::vector<TopicEntry> & entries);

  const ChannelTable & channel_table() const {return channel_table_;}
  ChannelTable & channel_table() {return channel_table_;}

  const std::optional<HelloMessage> & peer_hello() const {return peer_hello_;}
  const std::optional<uint32_t> & last_keepalive_sequence() const
  {
    return last_keepalive_sequence_;
  }

private:
  std::vector<Frame> handle_control_frame(const Frame & frame);
  Frame make_control_response(std::vector<uint8_t> payload);
  uint32_t next_sequence();

  ChannelTable channel_table_;
  ServiceTable service_table_;
  std::optional<HelloMessage> peer_hello_;
  std::optional<uint32_t> last_keepalive_sequence_;
  uint32_t control_seq_ = 0;
};

}  // namespace horus_unity_bridge::horuslink
