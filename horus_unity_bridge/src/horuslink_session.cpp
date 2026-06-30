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

#include "horus_unity_bridge/horuslink_session.hpp"

namespace horus_unity_bridge::horuslink
{

std::vector<Frame> Session::handle_frame(const Frame & frame)
{
  return handle_frame(frame, Lane::Realtime);
}

std::vector<Frame> Session::handle_frame(const Frame & frame, Lane lane)
{
  if (frame.header.msg_type == MessageType::Control) {
    return handle_control_frame(frame);
  }

  if (frame.header.msg_type == MessageType::Keepalive &&
    frame.header.channel_id == 0 &&
    frame.header.corr_id == 0 &&
    frame.payload.empty())
  {
    last_keepalive_sequence_ = frame.header.seq;
    return {};
  }

  if (frame.header.msg_type == MessageType::Data &&
    channel_table_.can_accept_data_frame(frame.header, lane))
  {
    return {frame};
  }

  if ((frame.header.msg_type == MessageType::ServiceRequest ||
    frame.header.msg_type == MessageType::ServiceResponse) &&
    frame.header.corr_id != 0 &&
    channel_table_.can_accept_data_frame(frame.header, lane))
  {
    return {frame};
  }

  return {};
}

Frame Session::make_topic_table_frame(const std::vector<TopicEntry> & entries)
{
  return make_control_response(encode_topic_table(entries));
}

Frame Session::make_hello_frame(const HelloMessage & message)
{
  return make_control_response(encode_hello(message));
}

Frame Session::make_subscribe_ack_frame(const SubscribeAck & ack)
{
  return make_control_response(encode_subscribe_ack(ack));
}

std::vector<Frame> Session::handle_control_frame(const Frame & frame)
{
  if (auto hello = decode_hello(frame.payload.data(), frame.payload.size())) {
    peer_hello_ = *hello;
    return {};
  }

  auto subscribe_request = decode_subscribe_request(frame.payload.data(), frame.payload.size());
  auto publisher_request = decode_publisher_request(frame.payload.data(), frame.payload.size());
  auto service_client_request = decode_service_client_request(
    frame.payload.data(),
    frame.payload.size());
  if (!subscribe_request.has_value() &&
    !publisher_request.has_value() &&
    !service_client_request.has_value())
  {
    return {};
  }

  uint16_t channel_id = 0;
  std::string topic;
  std::string type_name;
  Lane lane = Lane::Realtime;
  Delivery delivery = Delivery::ReliableFifo;
  if (subscribe_request.has_value()) {
    channel_id = subscribe_request->channel_id;
    topic = subscribe_request->topic;
    type_name = subscribe_request->type_name;
    lane = subscribe_request->lane;
    delivery = subscribe_request->delivery;
  } else if (publisher_request.has_value()) {
    channel_id = publisher_request->channel_id;
    topic = publisher_request->topic;
    type_name = publisher_request->type_name;
    lane = publisher_request->lane;
    delivery = publisher_request->delivery;
  } else {
    channel_id = service_client_request->channel_id;
    topic = service_client_request->service_name;
    type_name = service_client_request->service_type;
    lane = service_client_request->lane;
    delivery = service_client_request->delivery;
  }

  const bool accepted = channel_table_.begin_subscribe(
    channel_id,
    topic,
    type_name,
    lane,
    delivery);
  if (accepted) {
    channel_table_.confirm_subscribe_ack(channel_id);
  }

  const SubscribeAck ack{
    channel_id,
    accepted ? SubscribeStatus::Accepted : SubscribeStatus::Rejected,
    accepted ? "" : "channel or topic already registered"
  };
  return {make_subscribe_ack_frame(ack)};
}

Frame Session::make_control_response(std::vector<uint8_t> payload)
{
  Frame frame;
  frame.header.channel_id = 0;
  frame.header.msg_type = MessageType::Control;
  frame.header.flags = 0;
  frame.header.seq = next_sequence();
  frame.header.corr_id = 0;
  frame.header.length = static_cast<uint32_t>(payload.size());
  frame.payload = std::move(payload);
  return frame;
}

uint32_t Session::next_sequence()
{
  ++control_seq_;
  if (control_seq_ == 0) {
    ++control_seq_;
  }

  return control_seq_;
}

}  // namespace horus_unity_bridge::horuslink
