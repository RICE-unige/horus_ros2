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

#include "horus_unity_bridge/horuslink_control_messages.hpp"

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

void write_u16(std::vector<uint8_t> & data, uint16_t value)
{
  data.push_back(static_cast<uint8_t>(value & 0xFFu));
  data.push_back(static_cast<uint8_t>((value >> 8) & 0xFFu));
}

void write_u32(std::vector<uint8_t> & data, uint32_t value)
{
  data.push_back(static_cast<uint8_t>(value & 0xFFu));
  data.push_back(static_cast<uint8_t>((value >> 8) & 0xFFu));
  data.push_back(static_cast<uint8_t>((value >> 16) & 0xFFu));
  data.push_back(static_cast<uint8_t>((value >> 24) & 0xFFu));
}

TlvRecord byte_record(uint16_t type, uint8_t value)
{
  return TlvRecord{type, std::vector<uint8_t>{value}};
}

TlvRecord u16_record(uint16_t type, uint16_t value)
{
  std::vector<uint8_t> bytes;
  write_u16(bytes, value);
  return TlvRecord{type, bytes};
}

TlvRecord u32_record(uint16_t type, uint32_t value)
{
  std::vector<uint8_t> bytes;
  write_u32(bytes, value);
  return TlvRecord{type, bytes};
}

TlvRecord string_record(uint16_t type, const std::string & value)
{
  return TlvRecord{type, std::vector<uint8_t>(value.begin(), value.end())};
}

const TlvRecord * find_record(const std::vector<TlvRecord> & records, uint16_t type)
{
  for (const auto & record : records) {
    if (record.type == type) {
      return &record;
    }
  }

  return nullptr;
}

bool get_byte(const std::vector<TlvRecord> & records, uint16_t type, uint8_t & out)
{
  const TlvRecord * record = find_record(records, type);
  if (record == nullptr || record->value.size() != 1) {
    return false;
  }

  out = record->value[0];
  return true;
}

bool get_u16(const std::vector<TlvRecord> & records, uint16_t type, uint16_t & out)
{
  const TlvRecord * record = find_record(records, type);
  if (record == nullptr || record->value.size() != 2) {
    return false;
  }

  out = read_u16(record->value.data());
  return true;
}

bool get_u32(const std::vector<TlvRecord> & records, uint16_t type, uint32_t & out)
{
  const TlvRecord * record = find_record(records, type);
  if (record == nullptr || record->value.size() != 4) {
    return false;
  }

  out = read_u32(record->value.data());
  return true;
}

bool get_string(const std::vector<TlvRecord> & records, uint16_t type, std::string & out)
{
  const TlvRecord * record = find_record(records, type);
  if (record == nullptr) {
    return false;
  }

  out.assign(record->value.begin(), record->value.end());
  return true;
}

std::optional<std::vector<TlvRecord>> decode_records(
  const uint8_t * data,
  size_t size,
  ControlMessageKind expected_kind)
{
  std::vector<TlvRecord> records;
  if (!decode_tlvs(data, size, records)) {
    return std::nullopt;
  }

  uint8_t kind = 0;
  uint16_t version = 0;
  if (!get_byte(records, control_tlv::kKind, kind) ||
    kind != static_cast<uint8_t>(expected_kind) ||
    !get_u16(records, control_tlv::kProtocolVersion, version) ||
    version != control_tlv::kProtocolVersionValue)
  {
    return std::nullopt;
  }

  return records;
}

void validate_topic_fields(const std::string & topic, const std::string & type_name)
{
  if (topic.empty() || type_name.empty()) {
    throw std::invalid_argument("HorusLink topic and type fields cannot be empty.");
  }
}

std::vector<uint8_t> encode_topic_entry_payload(const TopicEntry & entry)
{
  validate_topic_fields(entry.topic, entry.type_name);
  if (entry.topic.size() > std::numeric_limits<uint16_t>::max() ||
    entry.type_name.size() > std::numeric_limits<uint16_t>::max())
  {
    throw std::out_of_range("HorusLink topic table string exceeds uint16 length.");
  }

  std::vector<uint8_t> payload;
  payload.reserve(8 + entry.topic.size() + entry.type_name.size());
  write_u16(payload, entry.channel_id);
  payload.push_back(static_cast<uint8_t>(entry.lane));
  payload.push_back(static_cast<uint8_t>(entry.delivery));
  write_u16(payload, static_cast<uint16_t>(entry.topic.size()));
  payload.insert(payload.end(), entry.topic.begin(), entry.topic.end());
  write_u16(payload, static_cast<uint16_t>(entry.type_name.size()));
  payload.insert(payload.end(), entry.type_name.begin(), entry.type_name.end());
  return payload;
}

std::optional<TopicEntry> decode_topic_entry_payload(const std::vector<uint8_t> & payload)
{
  if (payload.size() < 8) {
    return std::nullopt;
  }

  size_t offset = 0;
  TopicEntry entry;
  entry.channel_id = read_u16(payload.data());
  offset += 2;
  entry.lane = static_cast<Lane>(payload[offset++]);
  entry.delivery = static_cast<Delivery>(payload[offset++]);
  const uint16_t topic_length = read_u16(payload.data() + offset);
  offset += 2;
  if (offset + topic_length + 2 > payload.size()) {
    return std::nullopt;
  }

  entry.topic.assign(payload.begin() + offset, payload.begin() + offset + topic_length);
  offset += topic_length;
  const uint16_t type_name_length = read_u16(payload.data() + offset);
  offset += 2;
  if (offset + type_name_length != payload.size()) {
    return std::nullopt;
  }

  entry.type_name.assign(payload.begin() + offset, payload.end());
  return entry;
}

}  // namespace

bool HelloMessage::operator==(const HelloMessage & other) const
{
  return role == other.role &&
         max_payload_bytes == other.max_payload_bytes &&
         keepalive_ms == other.keepalive_ms;
}

bool SubscribeRequest::operator==(const SubscribeRequest & other) const
{
  return channel_id == other.channel_id &&
         topic == other.topic &&
         type_name == other.type_name &&
         lane == other.lane &&
         delivery == other.delivery;
}

bool PublisherRequest::operator==(const PublisherRequest & other) const
{
  return channel_id == other.channel_id &&
         topic == other.topic &&
         type_name == other.type_name &&
         lane == other.lane &&
         delivery == other.delivery;
}

bool SubscribeAck::operator==(const SubscribeAck & other) const
{
  return channel_id == other.channel_id &&
         status == other.status &&
         error == other.error;
}

bool TopicEntry::operator==(const TopicEntry & other) const
{
  return channel_id == other.channel_id &&
         topic == other.topic &&
         type_name == other.type_name &&
         lane == other.lane &&
         delivery == other.delivery;
}

std::vector<uint8_t> encode_hello(const HelloMessage & message)
{
  return encode_tlvs({
      byte_record(control_tlv::kKind, static_cast<uint8_t>(ControlMessageKind::Hello)),
      u16_record(control_tlv::kProtocolVersion, control_tlv::kProtocolVersionValue),
      byte_record(control_tlv::kRole, static_cast<uint8_t>(message.role)),
      u32_record(control_tlv::kMaxPayload, message.max_payload_bytes),
      u32_record(control_tlv::kKeepalive, message.keepalive_ms)
    });
}

std::optional<HelloMessage> decode_hello(const uint8_t * data, size_t size)
{
  auto records = decode_records(data, size, ControlMessageKind::Hello);
  if (!records.has_value()) {
    return std::nullopt;
  }

  uint8_t role = 0;
  uint32_t max_payload_bytes = 0;
  uint32_t keepalive_ms = 0;
  if (!get_byte(*records, control_tlv::kRole, role) ||
    !get_u32(*records, control_tlv::kMaxPayload, max_payload_bytes) ||
    !get_u32(*records, control_tlv::kKeepalive, keepalive_ms))
  {
    return std::nullopt;
  }

  return HelloMessage{static_cast<EndpointRole>(role), max_payload_bytes, keepalive_ms};
}

std::vector<uint8_t> encode_subscribe_request(const SubscribeRequest & request)
{
  validate_topic_fields(request.topic, request.type_name);
  return encode_tlvs({
      byte_record(control_tlv::kKind, static_cast<uint8_t>(ControlMessageKind::SubscribeRequest)),
      u16_record(control_tlv::kProtocolVersion, control_tlv::kProtocolVersionValue),
      u16_record(control_tlv::kChannelId, request.channel_id),
      string_record(control_tlv::kTopic, request.topic),
      string_record(control_tlv::kTypeName, request.type_name),
      byte_record(control_tlv::kLane, static_cast<uint8_t>(request.lane)),
      byte_record(control_tlv::kDelivery, static_cast<uint8_t>(request.delivery))
    });
}

std::optional<SubscribeRequest> decode_subscribe_request(const uint8_t * data, size_t size)
{
  auto records = decode_records(data, size, ControlMessageKind::SubscribeRequest);
  if (!records.has_value()) {
    return std::nullopt;
  }

  SubscribeRequest request;
  uint8_t lane = 0;
  uint8_t delivery = 0;
  if (!get_u16(*records, control_tlv::kChannelId, request.channel_id) ||
    !get_string(*records, control_tlv::kTopic, request.topic) ||
    !get_string(*records, control_tlv::kTypeName, request.type_name) ||
    !get_byte(*records, control_tlv::kLane, lane) ||
    !get_byte(*records, control_tlv::kDelivery, delivery))
  {
    return std::nullopt;
  }

  request.lane = static_cast<Lane>(lane);
  request.delivery = static_cast<Delivery>(delivery);
  return request;
}

std::vector<uint8_t> encode_publisher_request(const PublisherRequest & request)
{
  validate_topic_fields(request.topic, request.type_name);
  return encode_tlvs({
      byte_record(control_tlv::kKind, static_cast<uint8_t>(ControlMessageKind::PublisherRequest)),
      u16_record(control_tlv::kProtocolVersion, control_tlv::kProtocolVersionValue),
      u16_record(control_tlv::kChannelId, request.channel_id),
      string_record(control_tlv::kTopic, request.topic),
      string_record(control_tlv::kTypeName, request.type_name),
      byte_record(control_tlv::kLane, static_cast<uint8_t>(request.lane)),
      byte_record(control_tlv::kDelivery, static_cast<uint8_t>(request.delivery))
    });
}

std::optional<PublisherRequest> decode_publisher_request(const uint8_t * data, size_t size)
{
  auto records = decode_records(data, size, ControlMessageKind::PublisherRequest);
  if (!records.has_value()) {
    return std::nullopt;
  }

  PublisherRequest request;
  uint8_t lane = 0;
  uint8_t delivery = 0;
  if (!get_u16(*records, control_tlv::kChannelId, request.channel_id) ||
    !get_string(*records, control_tlv::kTopic, request.topic) ||
    !get_string(*records, control_tlv::kTypeName, request.type_name) ||
    !get_byte(*records, control_tlv::kLane, lane) ||
    !get_byte(*records, control_tlv::kDelivery, delivery))
  {
    return std::nullopt;
  }

  request.lane = static_cast<Lane>(lane);
  request.delivery = static_cast<Delivery>(delivery);
  return request;
}

std::vector<uint8_t> encode_subscribe_ack(const SubscribeAck & ack)
{
  return encode_tlvs({
      byte_record(control_tlv::kKind, static_cast<uint8_t>(ControlMessageKind::SubscribeAck)),
      u16_record(control_tlv::kProtocolVersion, control_tlv::kProtocolVersionValue),
      u16_record(control_tlv::kChannelId, ack.channel_id),
      byte_record(control_tlv::kSubscribeStatus, static_cast<uint8_t>(ack.status)),
      string_record(control_tlv::kError, ack.error)
    });
}

std::optional<SubscribeAck> decode_subscribe_ack(const uint8_t * data, size_t size)
{
  auto records = decode_records(data, size, ControlMessageKind::SubscribeAck);
  if (!records.has_value()) {
    return std::nullopt;
  }

  SubscribeAck ack;
  uint8_t status = 0;
  if (!get_u16(*records, control_tlv::kChannelId, ack.channel_id) ||
    !get_byte(*records, control_tlv::kSubscribeStatus, status) ||
    !get_string(*records, control_tlv::kError, ack.error))
  {
    return std::nullopt;
  }

  ack.status = static_cast<SubscribeStatus>(status);
  return ack;
}

std::vector<uint8_t> encode_topic_table(const std::vector<TopicEntry> & entries)
{
  std::vector<TlvRecord> records{
    byte_record(control_tlv::kKind, static_cast<uint8_t>(ControlMessageKind::TopicTable)),
    u16_record(control_tlv::kProtocolVersion, control_tlv::kProtocolVersionValue)
  };
  records.reserve(entries.size() + records.size());
  for (const auto & entry : entries) {
    records.push_back(TlvRecord{control_tlv::kTopicEntry, encode_topic_entry_payload(entry)});
  }

  return encode_tlvs(records);
}

std::optional<std::vector<TopicEntry>> decode_topic_table(const uint8_t * data, size_t size)
{
  auto records = decode_records(data, size, ControlMessageKind::TopicTable);
  if (!records.has_value()) {
    return std::nullopt;
  }

  std::vector<TopicEntry> entries;
  for (const auto & record : *records) {
    if (record.type != control_tlv::kTopicEntry) {
      continue;
    }

    auto entry = decode_topic_entry_payload(record.value);
    if (!entry.has_value()) {
      return std::nullopt;
    }

    entries.push_back(*entry);
  }

  return entries;
}

}  // namespace horus_unity_bridge::horuslink
