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

#pragma once

#include <cstdint>
#include <cstddef>
#include <optional>
#include <vector>
#include <algorithm>

namespace horus_unity_bridge::media
{
class RtpSequenceGaps
{
public:
  uint16_t observe(uint32_t ssrc, uint16_t sequence)
  {
    if (!initialized_ || ssrc != ssrc_) {
      initialized_ = true; ssrc_ = ssrc; last_ = sequence;
      return 0;
    }
    const auto advance = static_cast<uint16_t>(sequence - last_);
    if (advance == 0 || advance >= 0x8000) {return 0;}
    last_ = sequence;
    return advance - 1;
  }

private:
  bool initialized_ = false;
  uint32_t ssrc_ = 0;
  uint16_t last_ = 0;
};

struct RtpPacket
{
  uint16_t sequence;
  uint32_t timestamp;
  bool marker;
  const uint8_t * payload;
  size_t size;
};

inline uint32_t read_u32(const uint8_t * p)
{
  return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) |
    (uint32_t(p[2]) << 8) | uint32_t(p[3]);
}

inline std::optional<RtpPacket> parse_rtp(const uint8_t * data, size_t size)
{
  if (!data || size < 12 || data[0] >> 6 != 2 ||
    (data[1] >= 192 && data[1] <= 223)) {return std::nullopt;}
  size_t header = 12 + (data[0] & 15) * 4;
  if (header > size) {return std::nullopt;}
  if (data[0] & 16) {
    if (size - header < 4) {return std::nullopt;}
    const size_t words = (size_t(data[header + 2]) << 8) | data[header + 3];
    if (words > (size - header - 4) / 4) {return std::nullopt;}
    header += 4 + words * 4;
  }
  size_t end = size;
  if (data[0] & 32) {
    const size_t padding = data[size - 1];
    if (padding == 0 || padding > size - header) {return std::nullopt;}
    end -= padding;
  }
  if (end <= header) {return std::nullopt;}
  return RtpPacket{uint16_t((uint16_t(data[2]) << 8) | data[3]),
    read_u32(data + 4), (data[1] & 128) != 0, data + header, end - header};
}

inline void write_u32(uint8_t * p, uint32_t value)
{
  p[0] = value >> 24; p[1] = value >> 16; p[2] = value >> 8; p[3] = value;
}

// Translate only hop-local identifiers. The source timestamp intervals and
// sequence gaps survive: hiding a missing FU-A packet would corrupt decoding.
inline std::vector<uint8_t> translate_rtp(
  const RtpPacket & input, uint8_t payload_type, uint32_t ssrc,
  uint16_t sequence_offset, uint32_t timestamp_offset)
{
  std::vector<uint8_t> output(12 + input.size);
  output[0] = 128;
  output[1] = (input.marker ? 128 : 0) | (payload_type & 127);
  const uint16_t sequence = input.sequence + sequence_offset;
  output[2] = sequence >> 8; output[3] = sequence;
  write_u32(output.data() + 4, input.timestamp + timestamp_offset);
  write_u32(output.data() + 8, ssrc);
  std::copy(input.payload, input.payload + input.size, output.begin() + 12);
  return output;
}
}  // namespace horus_unity_bridge::media
