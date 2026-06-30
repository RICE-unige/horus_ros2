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
#include "horus_unity_bridge/horuslink_protocol.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace horus_unity_bridge::horuslink
{

enum class ControlMessageKind : uint8_t
{
  Hello = 1,
  SubscribeRequest = 2,
  SubscribeAck = 3,
  TopicTable = 4,
  PublisherRequest = 5,
  ServiceClientRequest = 6,
  ChannelCloseRequest = 7,
  TopicTableRequest = 8
};

enum class EndpointRole : uint8_t
{
  UnityClient = 1,
  Bridge = 2
};

enum class SubscribeStatus : uint8_t
{
  Accepted = 1,
  Rejected = 2
};

struct HelloMessage
{
  EndpointRole role = EndpointRole::UnityClient;
  uint32_t max_payload_bytes = 0;
  uint32_t keepalive_ms = 0;
  Lane lane = Lane::Realtime;
  uint64_t session_id = 0;

  bool operator==(const HelloMessage & other) const;
};

struct SubscribeRequest
{
  uint16_t channel_id = 0;
  std::string topic;
  std::string type_name;
  Lane lane = Lane::Realtime;
  Delivery delivery = Delivery::ReliableFifo;

  bool operator==(const SubscribeRequest & other) const;
};

struct PublisherRequest
{
  uint16_t channel_id = 0;
  std::string topic;
  std::string type_name;
  Lane lane = Lane::Realtime;
  Delivery delivery = Delivery::ReliableFifo;

  bool operator==(const PublisherRequest & other) const;
};

struct ServiceClientRequest
{
  uint16_t channel_id = 0;
  std::string service_name;
  std::string service_type;
  Lane lane = Lane::Realtime;
  Delivery delivery = Delivery::ReliableFifo;

  bool operator==(const ServiceClientRequest & other) const;
};

struct ChannelCloseRequest
{
  uint16_t channel_id = 0;

  bool operator==(const ChannelCloseRequest & other) const;
};

struct SubscribeAck
{
  uint16_t channel_id = 0;
  SubscribeStatus status = SubscribeStatus::Accepted;
  std::string error;

  bool operator==(const SubscribeAck & other) const;
};

struct TopicEntry
{
  uint16_t channel_id = 0;
  std::string topic;
  std::string type_name;
  Lane lane = Lane::Realtime;
  Delivery delivery = Delivery::ReliableFifo;

  bool operator==(const TopicEntry & other) const;
};

namespace control_tlv
{
constexpr uint16_t kKind = 1;
constexpr uint16_t kProtocolVersion = 2;
constexpr uint16_t kRole = 3;
constexpr uint16_t kChannelId = 4;
constexpr uint16_t kTopic = 5;
constexpr uint16_t kTypeName = 6;
constexpr uint16_t kLane = 7;
constexpr uint16_t kDelivery = 8;
constexpr uint16_t kMaxPayload = 9;
constexpr uint16_t kKeepalive = 10;
constexpr uint16_t kSubscribeStatus = 11;
constexpr uint16_t kError = 12;
constexpr uint16_t kSessionId = 13;
constexpr uint16_t kTopicEntry = 20;
constexpr uint16_t kProtocolVersionValue = 1;
}  // namespace control_tlv

std::vector<uint8_t> encode_hello(const HelloMessage & message);
std::optional<HelloMessage> decode_hello(const uint8_t * data, size_t size);

std::vector<uint8_t> encode_subscribe_request(const SubscribeRequest & request);
std::optional<SubscribeRequest> decode_subscribe_request(const uint8_t * data, size_t size);

std::vector<uint8_t> encode_publisher_request(const PublisherRequest & request);
std::optional<PublisherRequest> decode_publisher_request(const uint8_t * data, size_t size);

std::vector<uint8_t> encode_service_client_request(const ServiceClientRequest & request);
std::optional<ServiceClientRequest> decode_service_client_request(
  const uint8_t * data,
  size_t size);

std::vector<uint8_t> encode_channel_close_request(const ChannelCloseRequest & request);
std::optional<ChannelCloseRequest> decode_channel_close_request(const uint8_t * data, size_t size);

std::vector<uint8_t> encode_topic_table_request();
bool decode_topic_table_request(const uint8_t * data, size_t size);

std::vector<uint8_t> encode_subscribe_ack(const SubscribeAck & ack);
std::optional<SubscribeAck> decode_subscribe_ack(const uint8_t * data, size_t size);

std::vector<uint8_t> encode_topic_table(const std::vector<TopicEntry> & entries);
std::optional<std::vector<TopicEntry>> decode_topic_table(const uint8_t * data, size_t size);

}  // namespace horus_unity_bridge::horuslink
