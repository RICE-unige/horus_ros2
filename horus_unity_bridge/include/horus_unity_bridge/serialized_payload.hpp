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

#include <rclcpp/serialized_message.hpp>

#include <cstring>
#include <vector>

namespace horus_unity_bridge
{
namespace detail
{

inline bool has_cdr_header(const std::vector<uint8_t> & data)
{
  if (data.size() < 4) {
    return false;
  }
  if (data[0] != 0x00 || data[2] != 0x00 || data[3] != 0x00) {
    return false;
  }
  return data[1] == 0x00 || data[1] == 0x01;
}

inline std::vector<uint8_t> strip_cdr_header(const std::vector<uint8_t> & data)
{
  if (!has_cdr_header(data)) {
    return data;
  }
  return std::vector<uint8_t>(data.begin() + 4, data.end());
}

inline std::vector<uint8_t> add_cdr_header(const std::vector<uint8_t> & data)
{
  std::vector<uint8_t> result;
  result.reserve(data.size() + 4);
  result.push_back(0x00);
  result.push_back(0x01);  // little-endian CDR encapsulation
  result.push_back(0x00);
  result.push_back(0x00);
  result.insert(result.end(), data.begin(), data.end());
  return result;
}

inline void fill_serialized_message(
  const std::vector<uint8_t> & data,
  rclcpp::SerializedMessage & serialized_msg)
{
  serialized_msg.reserve(data.size());
  auto & rcl_msg = serialized_msg.get_rcl_serialized_message();
  if (rcl_msg.buffer_capacity < data.size()) {
    rcl_msg.allocator.deallocate(rcl_msg.buffer, rcl_msg.allocator.state);
    rcl_msg.buffer = static_cast<uint8_t *>(
      rcl_msg.allocator.allocate(data.size(), rcl_msg.allocator.state));
    rcl_msg.buffer_capacity = data.size();
  }
  std::memcpy(rcl_msg.buffer, data.data(), data.size());
  rcl_msg.buffer_length = data.size();
}

}  // namespace detail
}  // namespace horus_unity_bridge
