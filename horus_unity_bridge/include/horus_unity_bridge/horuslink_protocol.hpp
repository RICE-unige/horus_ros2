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

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace horus_unity_bridge::horuslink
{

enum class MessageType : uint8_t
{
  Data = 1,
  Control = 2,
  ServiceRequest = 3,
  ServiceResponse = 4,
  Keepalive = 5,
  Flow = 6
};

enum FrameFlags : uint8_t
{
  None = 0,
  RawOpaque = 1u << 0,
  Begin = 1u << 1,
  End = 1u << 2,
  ReplaceLatest = 1u << 3,
  Compressed = 1u << 4,
  LightCodec = 1u << 5
};

struct FrameHeader
{
  static constexpr size_t kSize = 16;

  uint16_t channel_id = 0;
  MessageType msg_type = MessageType::Data;
  uint8_t flags = 0;
  uint32_t seq = 0;
  uint32_t corr_id = 0;
  uint32_t length = 0;

  bool operator==(const FrameHeader & other) const;
};

struct Frame
{
  FrameHeader header;
  std::vector<uint8_t> payload;
};

struct TlvRecord
{
  uint16_t type = 0;
  std::vector<uint8_t> value;

  bool operator==(const TlvRecord & other) const;
};

bool encode_header(const FrameHeader & header, std::array<uint8_t, FrameHeader::kSize> & out);
bool decode_header(const uint8_t * data, size_t size, FrameHeader & out);
std::vector<uint8_t> serialize_frame(
  const FrameHeader & header, const uint8_t * payload,
  size_t size);

std::vector<uint8_t> encode_tlvs(const std::vector<TlvRecord> & records);
bool decode_tlvs(const uint8_t * data, size_t size, std::vector<TlvRecord> & out);

class FrameParser
{
public:
  explicit FrameParser(size_t max_payload_size = 512u * 1024u * 1024u);

  size_t parse(const uint8_t * data, size_t size, std::vector<Frame> & out);
  void reset();
  bool has_error() const {return parse_error_;}

private:
  enum class State
  {
    ReadingHeader,
    ReadingPayload
  };

  State state_ = State::ReadingHeader;
  size_t max_payload_size_ = 0;
  std::array<uint8_t, FrameHeader::kSize> header_buffer_{};
  size_t header_offset_ = 0;
  FrameHeader current_header_{};
  std::vector<uint8_t> current_payload_;
  size_t payload_offset_ = 0;
  bool parse_error_ = false;
};

}  // namespace horus_unity_bridge::horuslink
