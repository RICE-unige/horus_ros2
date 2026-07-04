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

#include "horus_unity_bridge/horuslink_protocol.hpp"

#include <algorithm>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace horus_unity_bridge::horuslink
{

namespace
{

uint16_t read_u16(const uint8_t * data)
{
  return static_cast<uint16_t>(data[0]) |
         static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8);
}

uint32_t read_u32(const uint8_t * data)
{
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) |
         (static_cast<uint32_t>(data[3]) << 24);
}

void write_u16(uint8_t * data, uint16_t value)
{
  data[0] = static_cast<uint8_t>(value & 0xFFu);
  data[1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
}

void write_u32(uint8_t * data, uint32_t value)
{
  data[0] = static_cast<uint8_t>(value & 0xFFu);
  data[1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
  data[2] = static_cast<uint8_t>((value >> 16) & 0xFFu);
  data[3] = static_cast<uint8_t>((value >> 24) & 0xFFu);
}

}  // namespace

bool FrameHeader::operator==(const FrameHeader & other) const
{
  return channel_id == other.channel_id &&
         msg_type == other.msg_type &&
         flags == other.flags &&
         seq == other.seq &&
         corr_id == other.corr_id &&
         length == other.length;
}

bool TlvRecord::operator==(const TlvRecord & other) const
{
  return type == other.type && value == other.value;
}

bool encode_header(const FrameHeader & header, std::array<uint8_t, FrameHeader::kSize> & out)
{
  write_u16(out.data(), header.channel_id);
  out[2] = static_cast<uint8_t>(header.msg_type);
  out[3] = header.flags;
  write_u32(out.data() + 4, header.seq);
  write_u32(out.data() + 8, header.corr_id);
  write_u32(out.data() + 12, header.length);
  return true;
}

bool decode_header(const uint8_t * data, size_t size, FrameHeader & out)
{
  if (data == nullptr || size < FrameHeader::kSize) {
    return false;
  }

  out.channel_id = read_u16(data);
  out.msg_type = static_cast<MessageType>(data[2]);
  out.flags = data[3];
  out.seq = read_u32(data + 4);
  out.corr_id = read_u32(data + 8);
  out.length = read_u32(data + 12);
  return true;
}

std::vector<uint8_t> serialize_frame(
  const FrameHeader & header,
  const uint8_t * payload,
  size_t size)
{
  if (size > std::numeric_limits<uint32_t>::max()) {
    throw std::out_of_range("HorusLink frame payload exceeds uint32 length.");
  }
  if (size > 0 && payload == nullptr) {
    throw std::invalid_argument("HorusLink frame payload pointer is null.");
  }

  FrameHeader encoded_header = header;
  encoded_header.length = static_cast<uint32_t>(size);
  std::array<uint8_t, FrameHeader::kSize> header_bytes{};
  encode_header(encoded_header, header_bytes);

  std::vector<uint8_t> output;
  output.reserve(FrameHeader::kSize + size);
  output.insert(output.end(), header_bytes.begin(), header_bytes.end());
  output.insert(output.end(), payload, payload + size);
  return output;
}

std::vector<uint8_t> encode_tlvs(const std::vector<TlvRecord> & records)
{
  size_t total_size = 0;
  for (const auto & record : records) {
    if (record.value.size() > std::numeric_limits<uint16_t>::max()) {
      throw std::out_of_range("HorusLink TLV value exceeds uint16 length.");
    }
    total_size += 4 + record.value.size();
  }

  std::vector<uint8_t> output(total_size);
  size_t offset = 0;
  for (const auto & record : records) {
    write_u16(output.data() + offset, record.type);
    write_u16(output.data() + offset + 2, static_cast<uint16_t>(record.value.size()));
    offset += 4;
    if (!record.value.empty()) {
      std::memcpy(output.data() + offset, record.value.data(), record.value.size());
      offset += record.value.size();
    }
  }

  return output;
}

bool decode_tlvs(const uint8_t * data, size_t size, std::vector<TlvRecord> & out)
{
  if (size > 0 && data == nullptr) {
    return false;
  }

  size_t offset = 0;
  while (offset < size) {
    if (size - offset < 4) {
      return false;
    }

    TlvRecord record;
    record.type = read_u16(data + offset);
    const uint16_t length = read_u16(data + offset + 2);
    offset += 4;
    if (size - offset < length) {
      return false;
    }

    record.value.assign(data + offset, data + offset + length);
    out.push_back(std::move(record));
    offset += length;
  }

  return true;
}

FrameParser::FrameParser(size_t max_payload_size)
: max_payload_size_(max_payload_size)
{
}

size_t FrameParser::parse(const uint8_t * data, size_t size, std::vector<Frame> & out)
{
  if (size > 0 && data == nullptr) {
    parse_error_ = true;
    return 0;
  }

  size_t consumed = 0;
  while (consumed < size && !parse_error_) {
    if (state_ == State::ReadingHeader) {
      const size_t to_copy = std::min(FrameHeader::kSize - header_offset_, size - consumed);
      std::memcpy(header_buffer_.data() + header_offset_, data + consumed, to_copy);
      header_offset_ += to_copy;
      consumed += to_copy;

      if (header_offset_ < FrameHeader::kSize) {
        break;
      }

      if (!decode_header(header_buffer_.data(), header_buffer_.size(), current_header_)) {
        parse_error_ = true;
        break;
      }
      if (current_header_.length > max_payload_size_) {
        parse_error_ = true;
        break;
      }

      current_payload_.assign(current_header_.length, uint8_t{0});
      payload_offset_ = 0;
      header_offset_ = 0;
      state_ = State::ReadingPayload;
      if (current_header_.length == 0) {
        out.push_back(Frame{current_header_, {}});
        state_ = State::ReadingHeader;
      }
    } else {
      const size_t remaining_payload = current_payload_.size() - payload_offset_;
      const size_t to_copy = std::min(remaining_payload, size - consumed);
      std::memcpy(current_payload_.data() + payload_offset_, data + consumed, to_copy);
      payload_offset_ += to_copy;
      consumed += to_copy;

      if (payload_offset_ == current_payload_.size()) {
        out.push_back(Frame{current_header_, std::move(current_payload_)});
        current_payload_.clear();
        payload_offset_ = 0;
        state_ = State::ReadingHeader;
      }
    }
  }

  return consumed;
}

void FrameParser::reset()
{
  state_ = State::ReadingHeader;
  header_buffer_.fill(0);
  header_offset_ = 0;
  current_header_ = FrameHeader{};
  current_payload_.clear();
  payload_offset_ = 0;
  parse_error_ = false;
}

}  // namespace horus_unity_bridge::horuslink
