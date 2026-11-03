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

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cctype>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace horus_unity_bridge::remote_render
{

constexpr std::size_t kMaximumMediaMetadataBytes = 64U * 1024U;
constexpr uint8_t kUserDataUnregisteredPayloadType = 5U;

inline bool contains_h264_idr(const std::vector<uint8_t> & data)
{
  for (std::size_t i = 0; i + 3U < data.size(); ++i) {
    if (data[i] == 0U && data[i + 1U] == 0U && data[i + 2U] == 1U &&
      (data[i + 3U] & 0x1fU) == 5U)
    {
      return true;
    }
  }
  return false;
}

// A missing access unit invalidates the prediction chain until an IDR.
class H264Continuity
{
public:
  void reset() {waiting_for_idr_ = true;}
  bool accept(const std::vector<uint8_t> & data, int64_t index = -1)
  {
    if (index >= 0 && last_index_ >= 0 && index != last_index_ + 1) {
      waiting_for_idr_ = true;
    }
    last_index_ = index;
    if (contains_h264_idr(data)) {
      waiting_for_idr_ = false;
    }
    return !waiting_for_idr_;
  }

private:
  bool waiting_for_idr_ = true;
  int64_t last_index_ = -1;
};
constexpr std::array<uint8_t, 16U> kMediaMetadataUuid = {
  0x48, 0x4f, 0x52, 0x55, 0x53, 0x2d, 0x52, 0x52,
  0x2d, 0x56, 0x33, 0x2d, 0x4d, 0x45, 0x54, 0x41};

inline bool is_h264_compressed_format(const std::string_view format)
{
  std::string normalized;
  normalized.reserve(format.size());
  for (const char value : format) {
    if (value == ';') {
      break;
    }
    if (!std::isspace(static_cast<unsigned char>(value))) {
      normalized.push_back(static_cast<char>(
          std::tolower(static_cast<unsigned char>(value))));
    }
  }
  return normalized == "h264" || normalized == "video/h264";
}

inline void append_ff_encoded_value(std::vector<uint8_t> & output, std::size_t value)
{
  while (value >= 255U) {
    output.push_back(0xffU);
    value -= 255U;
  }
  output.push_back(static_cast<uint8_t>(value));
}

inline std::vector<uint8_t> escape_rbsp(const std::vector<uint8_t> & rbsp)
{
  std::vector<uint8_t> escaped;
  escaped.reserve(rbsp.size() + rbsp.size() / 32U + 4U);
  unsigned int consecutive_zeroes = 0U;
  for (const uint8_t value : rbsp) {
    if (consecutive_zeroes >= 2U && value <= 0x03U) {
      escaped.push_back(0x03U);
      consecutive_zeroes = 0U;
    }
    escaped.push_back(value);
    consecutive_zeroes = value == 0U ? consecutive_zeroes + 1U : 0U;
  }
  return escaped;
}

inline std::vector<uint8_t> unescape_rbsp(
  const uint8_t * data, const std::size_t size)
{
  std::vector<uint8_t> rbsp;
  rbsp.reserve(size);
  unsigned int consecutive_zeroes = 0U;
  for (std::size_t index = 0U; index < size; ++index) {
    const uint8_t value = data[index];
    if (consecutive_zeroes >= 2U && value == 0x03U &&
      index + 1U < size && data[index + 1U] <= 0x03U)
    {
      consecutive_zeroes = 0U;
      continue;
    }
    rbsp.push_back(value);
    consecutive_zeroes = value == 0U ? consecutive_zeroes + 1U : 0U;
  }
  return rbsp;
}

inline std::size_t start_code_length_at(
  const std::vector<uint8_t> & data, const std::size_t offset)
{
  if (offset + 3U <= data.size() && data[offset] == 0U && data[offset + 1U] == 0U) {
    if (data[offset + 2U] == 1U) {
      return 3U;
    }
    if (offset + 4U <= data.size() && data[offset + 2U] == 0U &&
      data[offset + 3U] == 1U)
    {
      return 4U;
    }
  }
  return 0U;
}

inline void append_media_metadata(
  std::vector<uint8_t> & encoded_access_unit,
  const std::string_view metadata)
{
  if (metadata.empty()) {
    return;
  }
  if (metadata.size() > kMaximumMediaMetadataBytes) {
    throw std::length_error("remote-render media metadata exceeds 64 KiB");
  }

  std::vector<uint8_t> rbsp;
  rbsp.reserve(kMediaMetadataUuid.size() + metadata.size() + 8U);
  append_ff_encoded_value(rbsp, kUserDataUnregisteredPayloadType);
  append_ff_encoded_value(rbsp, kMediaMetadataUuid.size() + metadata.size());
  rbsp.insert(rbsp.end(), kMediaMetadataUuid.begin(), kMediaMetadataUuid.end());
  rbsp.insert(rbsp.end(), metadata.begin(), metadata.end());
  rbsp.push_back(0x80U);

  const std::vector<uint8_t> escaped = escape_rbsp(rbsp);
  std::vector<uint8_t> sei;
  sei.reserve(5U + escaped.size());
  sei.insert(sei.end(), {0U, 0U, 0U, 1U});
  sei.push_back(0x06U);
  sei.insert(sei.end(), escaped.begin(), escaped.end());

  // Preserve the encoder's AUD/SPS/PPS before our SEI. Android WebRTC derives
  // frame dimensions from those headers; metadata must not displace them.
  std::size_t insertion = 0U;
  for (std::size_t offset = 0U; offset < encoded_access_unit.size(); ++offset) {
    const std::size_t prefix = start_code_length_at(encoded_access_unit, offset);
    if (prefix == 0U || offset + prefix >= encoded_access_unit.size()) {
      continue;
    }
    const uint8_t type = encoded_access_unit[offset + prefix] & 0x1fU;
    if (type >= 1U && type <= 5U) {
      insertion = offset;
      break;
    }
    offset += prefix;
  }
  encoded_access_unit.insert(
    encoded_access_unit.begin() + static_cast<std::ptrdiff_t>(insertion),
    sei.begin(), sei.end());
}

inline bool split_media_metadata(
  const std::vector<uint8_t> & encoded_access_unit,
  std::size_t & encoded_size,
  std::string & metadata)
{
  encoded_size = encoded_access_unit.size();
  metadata.clear();

  std::size_t offset = 0U;
  while (offset < encoded_access_unit.size()) {
    const std::size_t start_length = start_code_length_at(encoded_access_unit, offset);
    if (start_length == 0U) {
      ++offset;
      continue;
    }
    const std::size_t nal_start = offset + start_length;
    if (nal_start >= encoded_access_unit.size()) {
      break;
    }
    std::size_t nal_end = nal_start + 1U;
    while (nal_end < encoded_access_unit.size() &&
      start_code_length_at(encoded_access_unit, nal_end) == 0U)
    {
      ++nal_end;
    }
    if ((encoded_access_unit[nal_start] & 0x1fU) != 6U) {
      offset = nal_end;
      continue;
    }

    const std::vector<uint8_t> rbsp = unescape_rbsp(
      encoded_access_unit.data() + nal_start + 1U,
      nal_end - nal_start - 1U);
    std::size_t cursor = 0U;
    while (cursor < rbsp.size() && rbsp[cursor] != 0x80U) {
      std::size_t payload_type = 0U;
      while (cursor < rbsp.size() && rbsp[cursor] == 0xffU) {
        payload_type += 255U;
        ++cursor;
      }
      if (cursor >= rbsp.size()) {
        break;
      }
      payload_type += rbsp[cursor++];

      std::size_t payload_size = 0U;
      while (cursor < rbsp.size() && rbsp[cursor] == 0xffU) {
        payload_size += 255U;
        ++cursor;
      }
      if (cursor >= rbsp.size()) {
        break;
      }
      payload_size += rbsp[cursor++];
      if (payload_size > rbsp.size() - cursor) {
        break;
      }

      if (payload_type == kUserDataUnregisteredPayloadType &&
        payload_size >= kMediaMetadataUuid.size() &&
        std::equal(
          kMediaMetadataUuid.begin(), kMediaMetadataUuid.end(),
          rbsp.begin() + static_cast<std::ptrdiff_t>(cursor)))
      {
        const std::size_t metadata_size = payload_size - kMediaMetadataUuid.size();
        if (metadata_size == 0U || metadata_size > kMaximumMediaMetadataBytes) {
          return false;
        }
        const std::size_t metadata_start = cursor + kMediaMetadataUuid.size();
        metadata.assign(
          reinterpret_cast<const char *>(rbsp.data() + metadata_start),
          metadata_size);
        return true;
      }
      cursor += payload_size;
    }
    offset = nal_end;
  }
  return false;
}

}  // namespace horus_unity_bridge::remote_render
