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
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace horus_unity_bridge::remote_render
{

constexpr std::size_t kAuxiliaryChunkHeaderSize = 20U;
constexpr std::size_t kAuxiliaryChunkPayloadBytes = 48U * 1024U;
constexpr uint8_t kAuxiliaryChunkMagic[4] = {'H', 'R', 'A', '1'};

inline uint32_t read_u32_le(const uint8_t * data)
{
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8U) |
         (static_cast<uint32_t>(data[2]) << 16U) |
         (static_cast<uint32_t>(data[3]) << 24U);
}

inline void append_u16_le(std::vector<uint8_t> & output, uint16_t value)
{
  output.push_back(static_cast<uint8_t>(value & 0xffU));
  output.push_back(static_cast<uint8_t>((value >> 8U) & 0xffU));
}

inline void append_u32_le(std::vector<uint8_t> & output, uint32_t value)
{
  output.push_back(static_cast<uint8_t>(value & 0xffU));
  output.push_back(static_cast<uint8_t>((value >> 8U) & 0xffU));
  output.push_back(static_cast<uint8_t>((value >> 16U) & 0xffU));
  output.push_back(static_cast<uint8_t>((value >> 24U) & 0xffU));
}

inline uint32_t remote_frame_sequence(const std::vector<uint8_t> & data)
{
  if (data.size() < 12U ||
    data[0] != 'H' || data[1] != 'R' || data[2] != 'R' || data[3] != 'F')
  {
    throw std::invalid_argument("auxiliary payload is not a HORUS remote frame");
  }
  return read_u32_le(data.data() + 8U);
}

inline std::vector<std::vector<uint8_t>> chunk_auxiliary_payload(
  const std::vector<uint8_t> & data,
  std::size_t maximum_message_size)
{
  if (data.empty()) {
    return {};
  }

  const std::size_t negotiated_payload_limit =
    maximum_message_size > kAuxiliaryChunkHeaderSize ?
    maximum_message_size - kAuxiliaryChunkHeaderSize :
    0U;
  const std::size_t chunk_payload_size = maximum_message_size == 0U ?
    kAuxiliaryChunkPayloadBytes :
    std::min(kAuxiliaryChunkPayloadBytes, negotiated_payload_limit);
  if (chunk_payload_size == 0U) {
    throw std::length_error("negotiated data-channel message size is too small");
  }
  if (data.size() <= chunk_payload_size) {
    return {data};
  }
  if (data.size() > std::numeric_limits<uint32_t>::max()) {
    throw std::length_error("auxiliary payload exceeds the chunk protocol limit");
  }

  const std::size_t chunk_count =
    (data.size() + chunk_payload_size - 1U) / chunk_payload_size;
  if (chunk_count > std::numeric_limits<uint16_t>::max()) {
    throw std::length_error("auxiliary payload requires too many chunks");
  }

  const uint32_t sequence = remote_frame_sequence(data);
  std::vector<std::vector<uint8_t>> chunks;
  chunks.reserve(chunk_count);
  for (std::size_t index = 0; index < chunk_count; ++index) {
    const std::size_t offset = index * chunk_payload_size;
    const std::size_t size = std::min(chunk_payload_size, data.size() - offset);
    std::vector<uint8_t> chunk;
    chunk.reserve(kAuxiliaryChunkHeaderSize + size);
    chunk.insert(
      chunk.end(),
      std::begin(kAuxiliaryChunkMagic),
      std::end(kAuxiliaryChunkMagic));
    append_u32_le(chunk, sequence);
    append_u32_le(chunk, static_cast<uint32_t>(data.size()));
    append_u16_le(chunk, static_cast<uint16_t>(index));
    append_u16_le(chunk, static_cast<uint16_t>(chunk_count));
    append_u32_le(chunk, static_cast<uint32_t>(offset));
    chunk.insert(
      chunk.end(),
      data.begin() + static_cast<std::ptrdiff_t>(offset),
      data.begin() + static_cast<std::ptrdiff_t>(offset + size));
    chunks.push_back(std::move(chunk));
  }
  return chunks;
}

}  // namespace horus_unity_bridge::remote_render
