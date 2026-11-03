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

#include "horus_unity_bridge/horuslink_connection_manager.hpp"
#include "horus_unity_bridge/horuslink_control_messages.hpp"

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <dirent.h>

#include <atomic>
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

namespace horus_unity_bridge::horuslink
{

namespace
{

class SocketHandle
{
public:
  explicit SocketHandle(int fd = -1)
  : fd_(fd)
  {
  }

  ~SocketHandle()
  {
    reset();
  }

  SocketHandle(const SocketHandle &) = delete;
  SocketHandle & operator=(const SocketHandle &) = delete;

  SocketHandle(SocketHandle && other) noexcept
  : fd_(other.fd_)
  {
    other.fd_ = -1;
  }

  SocketHandle & operator=(SocketHandle && other) noexcept
  {
    if (this != &other) {
      reset();
      fd_ = other.fd_;
      other.fd_ = -1;
    }
    return *this;
  }

  int get() const {return fd_;}

  void reset()
  {
    if (fd_ >= 0) {
      close(fd_);
      fd_ = -1;
    }
  }

private:
  int fd_ = -1;
};

bool wait_until(const std::function<bool()> & condition, int timeout_ms = 2000)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (condition()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  return condition();
}

SocketHandle connect_to(uint16_t port)
{
  SocketHandle socket_handle(socket(AF_INET, SOCK_STREAM, 0));
  EXPECT_GE(socket_handle.get(), 0);

  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(port);
  inet_pton(AF_INET, "127.0.0.1", &address.sin_addr);
  EXPECT_EQ(
    connect(socket_handle.get(), reinterpret_cast<sockaddr *>(&address), sizeof(address)),
    0);
  return socket_handle;
}

bool recv_exact(int fd, uint8_t * data, size_t size)
{
  size_t offset = 0;
  while (offset < size) {
    pollfd poll_fd{};
    poll_fd.fd = fd;
    poll_fd.events = POLLIN;
    if (poll(&poll_fd, 1, 2000) <= 0) {
      return false;
    }
    const ssize_t received = recv(fd, data + offset, size - offset, 0);
    if (received <= 0) {
      return false;
    }
    offset += static_cast<size_t>(received);
  }

  return true;
}

bool wait_for_peer_close(int fd, int timeout_ms = 2000)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    pollfd poll_fd{};
    poll_fd.fd = fd;
    poll_fd.events = POLLIN | POLLHUP | POLLERR;
    const int ready = poll(&poll_fd, 1, 50);
    if (ready < 0 && errno == EINTR) {
      continue;
    }
    if (ready <= 0) {
      continue;
    }

    uint8_t byte = 0;
    const ssize_t received = recv(fd, &byte, sizeof(byte), MSG_PEEK);
    return received == 0 || (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK);
  }

  return false;
}

Frame recv_frame(int fd)
{
  std::array<uint8_t, FrameHeader::kSize> header_bytes{};
  EXPECT_TRUE(recv_exact(fd, header_bytes.data(), header_bytes.size()));
  FrameHeader header;
  EXPECT_TRUE(decode_header(header_bytes.data(), header_bytes.size(), header));
  std::vector<uint8_t> payload(header.length);
  if (header.length > 0) {
    EXPECT_TRUE(recv_exact(fd, payload.data(), payload.size()));
  }

  return Frame{header, std::move(payload)};
}

void expect_bridge_hello(int fd, const HorusLinkConnectionManager::Config & config)
{
  const Frame hello_frame = recv_frame(fd);
  ASSERT_EQ(hello_frame.header.channel_id, 0u);
  ASSERT_EQ(hello_frame.header.msg_type, MessageType::Control);
  ASSERT_EQ(hello_frame.header.seq, 1u);
  ASSERT_EQ(hello_frame.header.corr_id, 0u);
  auto hello = decode_hello(hello_frame.payload.data(), hello_frame.payload.size());
  ASSERT_TRUE(hello.has_value());
  EXPECT_EQ(hello->role, EndpointRole::Bridge);
  EXPECT_EQ(hello->max_payload_bytes, static_cast<uint32_t>(config.max_payload_size));
  EXPECT_EQ(hello->keepalive_ms, config.keepalive_ms);
}

void send_frame(int fd, Frame frame)
{
  const auto bytes = serialize_frame(frame.header, frame.payload.data(), frame.payload.size());
  size_t offset = 0;
  while (offset < bytes.size()) {
    const ssize_t sent = send(fd, bytes.data() + offset, bytes.size() - offset, MSG_NOSIGNAL);
    ASSERT_GT(sent, 0);
    offset += static_cast<size_t>(sent);
  }
}

Frame make_control_frame(std::vector<uint8_t> payload, uint32_t seq = 1)
{
  Frame frame;
  frame.header.channel_id = 0;
  frame.header.msg_type = MessageType::Control;
  frame.header.seq = seq;
  frame.header.length = static_cast<uint32_t>(payload.size());
  frame.payload = std::move(payload);
  return frame;
}

void send_client_hellos(int realtime_fd, int bulk_fd, uint64_t session_id = 0x1020304050607080ull)
{
  send_frame(
    realtime_fd,
    make_control_frame(
      encode_hello(HelloMessage{
        EndpointRole::UnityClient,
        512u * 1024u * 1024u,
        1000,
        Lane::Realtime,
        session_id}),
      1));
  send_frame(
    bulk_fd,
    make_control_frame(
      encode_hello(HelloMessage{
        EndpointRole::UnityClient,
        512u * 1024u * 1024u,
        1000,
        Lane::Bulk,
        session_id}),
      1));
}

Frame make_data_frame(uint16_t channel_id, std::vector<uint8_t> payload, uint32_t seq = 1)
{
  Frame frame;
  frame.header.channel_id = channel_id;
  frame.header.msg_type = MessageType::Data;
  frame.header.flags = FrameFlags::RawOpaque | FrameFlags::ReplaceLatest;
  frame.header.seq = seq;
  frame.header.length = static_cast<uint32_t>(payload.size());
  frame.payload = std::move(payload);
  return frame;
}

HorusLinkConnectionManager::Config make_config()
{
  HorusLinkConnectionManager::Config config;
  config.bind_address = "127.0.0.1";
  config.realtime_port = 0;
  config.bulk_port = 0;
  config.realtime_queue_depth = 4;
  config.bulk_queue_depth = 4;
  return config;
}

}  // namespace

TEST(HorusLinkConnectionManagerTest, AcceptsDualLaneSessionAndAcksSubscribe)
{
  auto config = make_config();
  HorusLinkConnectionManager manager(config);
  std::atomic<int> connected_id{0};
  manager.set_connection_callback(
    [&connected_id](int connection_id, const std::string &) {
      connected_id = connection_id;
    });
  ASSERT_TRUE(manager.start());

  auto realtime = connect_to(manager.realtime_port());
  auto bulk = connect_to(manager.bulk_port());
  send_client_hellos(realtime.get(), bulk.get());
  ASSERT_TRUE(wait_until([&connected_id]() {return connected_id.load() != 0;}));
  EXPECT_EQ(manager.connection_count(), 1u);
  expect_bridge_hello(realtime.get(), config);

  const SubscribeRequest request{
    7,
    "/camera",
    "sensor_msgs/msg/Image",
    Lane::Bulk,
    Delivery::ReplaceLatest
  };
  send_frame(realtime.get(), make_control_frame(encode_subscribe_request(request)));

  Frame ack_frame = recv_frame(realtime.get());
  ASSERT_EQ(ack_frame.header.msg_type, MessageType::Control);
  auto ack = decode_subscribe_ack(ack_frame.payload.data(), ack_frame.payload.size());
  ASSERT_TRUE(ack.has_value());
  EXPECT_EQ(ack->channel_id, 7u);
  EXPECT_EQ(ack->status, SubscribeStatus::Accepted);
}

TEST(HorusLinkConnectionManagerTest, StopClosesUnpairedPendingLaneSockets)
{
  auto config = make_config();
  HorusLinkConnectionManager manager(config);
  ASSERT_TRUE(manager.start());

  auto realtime = connect_to(manager.realtime_port());
  send_frame(
    realtime.get(),
    make_control_frame(
      encode_hello(HelloMessage{
      EndpointRole::UnityClient,
      512u * 1024u * 1024u,
      1000,
      Lane::Realtime,
      0x8877665544332211ull}),
      1));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  manager.stop();

  EXPECT_TRUE(wait_for_peer_close(realtime.get()));
}

TEST(HorusLinkConnectionManagerTest, RepliesToTopicTableRequestOnRealtimeLane)
{
  auto config = make_config();
  HorusLinkConnectionManager manager(config);
  std::atomic<int> connected_id{0};
  std::atomic<int> request_connection_id{0};
  const std::vector<TopicEntry> entries{
    TopicEntry{0, "/tf", "tf2_msgs/msg/TFMessage", Lane::Realtime, Delivery::ReliableFifo},
    TopicEntry{0, "/camera", "sensor_msgs/msg/Image", Lane::Bulk, Delivery::ReplaceLatest}
  };

  manager.set_connection_callback(
    [&connected_id](int connection_id, const std::string &) {
      connected_id = connection_id;
    });
  manager.set_topic_table_callback(
    [&request_connection_id, &entries](int connection_id) {
      request_connection_id = connection_id;
      return entries;
    });
  ASSERT_TRUE(manager.start());

  auto realtime = connect_to(manager.realtime_port());
  auto bulk = connect_to(manager.bulk_port());
  send_client_hellos(realtime.get(), bulk.get());
  ASSERT_TRUE(wait_until([&connected_id]() {return connected_id.load() != 0;}));
  expect_bridge_hello(realtime.get(), config);

  send_frame(realtime.get(), make_control_frame(encode_topic_table_request(), 2));

  Frame topic_table_frame = recv_frame(realtime.get());
  ASSERT_EQ(topic_table_frame.header.channel_id, 0u);
  ASSERT_EQ(topic_table_frame.header.msg_type, MessageType::Control);
  auto decoded = decode_topic_table(
    topic_table_frame.payload.data(),
    topic_table_frame.payload.size());
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(*decoded, entries);
  EXPECT_EQ(request_connection_id.load(), connected_id.load());
}

TEST(HorusLinkConnectionManagerTest, ChannelCloseDispatchesKindAndRemovesChannel)
{
  auto config = make_config();
  HorusLinkConnectionManager manager(config);
  std::atomic<int> connected_id{0};
  std::mutex event_mutex;
  std::vector<std::pair<ChannelRegistrationKind, std::string>> close_events;
  std::vector<Frame> delivered_frames;

  manager.set_connection_callback(
    [&connected_id](int connection_id, const std::string &) {
      connected_id = connection_id;
    });
  manager.set_subscribe_callback(
    [](int, const ChannelDescriptor &, std::string &) {
      return true;
    });
  manager.set_publisher_callback(
    [](int, const ChannelDescriptor &, std::string &) {
      return true;
    });
  manager.set_service_client_callback(
    [](int, const ChannelDescriptor &, std::string &) {
      return true;
    });
  manager.set_channel_close_callback(
    [&event_mutex, &close_events](
      int,
      ChannelRegistrationKind registration_kind,
      const ChannelDescriptor & channel) {
      std::lock_guard<std::mutex> lock(event_mutex);
      close_events.emplace_back(registration_kind, channel.topic);
    });
  manager.set_frame_callback(
    [&event_mutex, &delivered_frames](
      int,
      Lane,
      const ChannelDescriptor &,
      const Frame & frame) {
      std::lock_guard<std::mutex> lock(event_mutex);
      delivered_frames.push_back(frame);
    });

  ASSERT_TRUE(manager.start());

  auto realtime = connect_to(manager.realtime_port());
  auto bulk = connect_to(manager.bulk_port());
  send_client_hellos(realtime.get(), bulk.get());
  ASSERT_TRUE(wait_until([&connected_id]() {return connected_id.load() != 0;}));
  expect_bridge_hello(realtime.get(), config);

  const SubscribeRequest subscriber{
    7,
    "/camera",
    "sensor_msgs/msg/Image",
    Lane::Bulk,
    Delivery::ReplaceLatest
  };
  send_frame(realtime.get(), make_control_frame(encode_subscribe_request(subscriber), 1));
  recv_frame(realtime.get());
  send_frame(realtime.get(), make_control_frame(encode_channel_close_request({7}), 2));

  const PublisherRequest publisher{
    8,
    "/cmd_vel",
    "geometry_msgs/msg/Twist",
    Lane::Realtime,
    Delivery::ReliableFifo
  };
  send_frame(realtime.get(), make_control_frame(encode_publisher_request(publisher), 3));
  recv_frame(realtime.get());
  send_frame(realtime.get(), make_control_frame(encode_channel_close_request({8}), 4));

  const ServiceClientRequest service{
    9,
    "/trigger",
    "std_srvs/srv/Trigger",
    Lane::Realtime,
    Delivery::ReliableFifo
  };
  send_frame(realtime.get(), make_control_frame(encode_service_client_request(service), 5));
  recv_frame(realtime.get());
  send_frame(realtime.get(), make_control_frame(encode_channel_close_request({9}), 6));

  ASSERT_TRUE(wait_until([&event_mutex, &close_events]() {
      std::lock_guard<std::mutex> lock(event_mutex);
      return close_events.size() == 3;
    }));

  {
    std::lock_guard<std::mutex> lock(event_mutex);
    ASSERT_EQ(close_events.size(), 3u);
    EXPECT_EQ(close_events[0].first, ChannelRegistrationKind::Subscriber);
    EXPECT_EQ(close_events[0].second, "/camera");
    EXPECT_EQ(close_events[1].first, ChannelRegistrationKind::Publisher);
    EXPECT_EQ(close_events[1].second, "/cmd_vel");
    EXPECT_EQ(close_events[2].first, ChannelRegistrationKind::ServiceClient);
    EXPECT_EQ(close_events[2].second, "/trigger");
  }

  send_frame(bulk.get(), make_data_frame(7, {0x10, 0x20}, 7));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  {
    std::lock_guard<std::mutex> lock(event_mutex);
    EXPECT_TRUE(delivered_frames.empty());
  }
}

TEST(HorusLinkConnectionManagerTest, SendsRealtimeKeepaliveAfterHandshake)
{
  auto config = make_config();
  config.keepalive_ms = 20;
  HorusLinkConnectionManager manager(config);
  std::atomic<int> connected_id{0};
  manager.set_connection_callback(
    [&connected_id](int connection_id, const std::string &) {
      connected_id = connection_id;
    });
  ASSERT_TRUE(manager.start());

  auto realtime = connect_to(manager.realtime_port());
  auto bulk = connect_to(manager.bulk_port());
  send_client_hellos(realtime.get(), bulk.get());
  ASSERT_TRUE(wait_until([&connected_id]() {return connected_id.load() != 0;}));
  expect_bridge_hello(realtime.get(), config);

  Frame keepalive = recv_frame(realtime.get());
  EXPECT_EQ(keepalive.header.channel_id, 0u);
  EXPECT_EQ(keepalive.header.msg_type, MessageType::Keepalive);
  EXPECT_EQ(keepalive.header.flags, 0u);
  EXPECT_EQ(keepalive.header.seq, 2u);
  EXPECT_EQ(keepalive.header.corr_id, 0u);
  EXPECT_EQ(keepalive.header.length, 0u);
  EXPECT_TRUE(keepalive.payload.empty());
}

TEST(HorusLinkConnectionManagerTest, LiveMapPolicyReachesBridgeCallback)
{
  auto config = make_config();
  HorusLinkConnectionManager manager(config);
  std::atomic<int> connected_id{0};
  std::atomic<bool> live_map{false};
  manager.set_connection_callback([&](int id, const std::string &) {connected_id = id;});
  manager.set_subscribe_callback([&](int, const ChannelDescriptor & channel, std::string &) {
    live_map = channel.live_map;
    return true;
  });
  ASSERT_TRUE(manager.start());
  auto realtime = connect_to(manager.realtime_port());
  auto bulk = connect_to(manager.bulk_port());
  send_client_hellos(realtime.get(), bulk.get());
  ASSERT_TRUE(wait_until([&]() {return connected_id.load() != 0;}));
  expect_bridge_hello(realtime.get(), config);
  SubscribeRequest request{7, "/vendor/map/indexed", "std_msgs/msg/UInt8MultiArray",
    Lane::Bulk, Delivery::ReliableFifo, true};
  send_frame(realtime.get(), make_control_frame(encode_subscribe_request(request), 1));
  recv_frame(realtime.get());
  EXPECT_TRUE(live_map.load());
}

TEST(HorusLinkConnectionManagerTest, DeliversInboundBulkDataAfterSubscribeAck)
{
  auto config = make_config();
  HorusLinkConnectionManager manager(config);
  std::atomic<int> connected_id{0};
  std::mutex frame_mutex;
  std::vector<Frame> observed_frames;
  std::vector<ChannelDescriptor> observed_channels;
  std::vector<Lane> observed_lanes;
  manager.set_connection_callback(
    [&connected_id](int connection_id, const std::string &) {
      connected_id = connection_id;
    });
  manager.set_frame_callback(
    [&frame_mutex, &observed_frames, &observed_channels, &observed_lanes](
      int, Lane lane, const ChannelDescriptor & channel, const Frame & frame) {
      std::lock_guard<std::mutex> lock(frame_mutex);
      observed_lanes.push_back(lane);
      observed_channels.push_back(channel);
      observed_frames.push_back(frame);
    });
  ASSERT_TRUE(manager.start());

  auto realtime = connect_to(manager.realtime_port());
  auto bulk = connect_to(manager.bulk_port());
  send_client_hellos(realtime.get(), bulk.get());
  ASSERT_TRUE(wait_until([&connected_id]() {return connected_id.load() != 0;}));
  expect_bridge_hello(realtime.get(), config);

  const SubscribeRequest request{
    7,
    "/camera",
    "sensor_msgs/msg/Image",
    Lane::Bulk,
    Delivery::ReplaceLatest
  };
  send_frame(realtime.get(), make_control_frame(encode_subscribe_request(request)));
  recv_frame(realtime.get());
  send_frame(bulk.get(), make_data_frame(7, {0x10, 0x20, 0x30}, 2));

  ASSERT_TRUE(wait_until([&frame_mutex, &observed_frames]() {
      std::lock_guard<std::mutex> lock(frame_mutex);
      return !observed_frames.empty();
    }));
  std::lock_guard<std::mutex> lock(frame_mutex);
  ASSERT_EQ(observed_frames.size(), 1u);
  EXPECT_EQ(observed_lanes[0], Lane::Bulk);
  EXPECT_EQ(observed_channels[0].topic, "/camera");
  EXPECT_EQ(observed_channels[0].type_name, "sensor_msgs/msg/Image");
  EXPECT_EQ(observed_channels[0].delivery, Delivery::ReplaceLatest);
  EXPECT_EQ(observed_frames[0].header.channel_id, 7u);
  EXPECT_EQ(observed_frames[0].payload, std::vector<uint8_t>({0x10, 0x20, 0x30}));
}

TEST(HorusLinkConnectionManagerTest, RejectsSubscribeWhenBridgeCallbackFails)
{
  auto config = make_config();
  HorusLinkConnectionManager manager(config);
  std::atomic<int> connected_id{0};
  manager.set_connection_callback(
    [&connected_id](int connection_id, const std::string &) {
      connected_id = connection_id;
    });
  manager.set_subscribe_callback(
    [](int, const ChannelDescriptor &, std::string & error) {
      error = "no ROS subscriber";
      return false;
    });
  ASSERT_TRUE(manager.start());

  auto realtime = connect_to(manager.realtime_port());
  auto bulk = connect_to(manager.bulk_port());
  send_client_hellos(realtime.get(), bulk.get());
  ASSERT_TRUE(wait_until([&connected_id]() {return connected_id.load() != 0;}));
  expect_bridge_hello(realtime.get(), config);

  const SubscribeRequest request{
    11,
    "/missing",
    "std_msgs/msg/String",
    Lane::Realtime,
    Delivery::ReliableFifo
  };
  send_frame(realtime.get(), make_control_frame(encode_subscribe_request(request)));

  Frame ack_frame = recv_frame(realtime.get());
  ASSERT_EQ(ack_frame.header.msg_type, MessageType::Control);
  auto ack = decode_subscribe_ack(ack_frame.payload.data(), ack_frame.payload.size());
  ASSERT_TRUE(ack.has_value());
  EXPECT_EQ(ack->channel_id, 11u);
  EXPECT_EQ(ack->status, SubscribeStatus::Rejected);
  EXPECT_EQ(ack->error, "no ROS subscriber");
}

TEST(HorusLinkConnectionManagerTest, SendsOutboundPayloadOnNegotiatedBulkLane)
{
  auto config = make_config();
  HorusLinkConnectionManager manager(config);
  std::atomic<int> connected_id{0};
  manager.set_connection_callback(
    [&connected_id](int connection_id, const std::string &) {
      connected_id = connection_id;
    });
  ASSERT_TRUE(manager.start());

  auto realtime = connect_to(manager.realtime_port());
  auto bulk = connect_to(manager.bulk_port());
  send_client_hellos(realtime.get(), bulk.get());
  ASSERT_TRUE(wait_until([&connected_id]() {return connected_id.load() != 0;}));
  expect_bridge_hello(realtime.get(), config);

  const SubscribeRequest request{
    9,
    "/mesh",
    "horus_msgs/msg/IndexedMesh",
    Lane::Bulk,
    Delivery::ReplaceLatest
  };
  send_frame(realtime.get(), make_control_frame(encode_subscribe_request(request)));
  recv_frame(realtime.get());

  const std::vector<uint8_t> payload{0x44, 0x55, 0x66};
  ASSERT_TRUE(manager.send_to_topic(connected_id.load(), "/mesh", payload));

  Frame outbound = recv_frame(bulk.get());
  EXPECT_EQ(outbound.header.channel_id, 9u);
  EXPECT_EQ(outbound.header.msg_type, MessageType::Data);
  EXPECT_EQ(outbound.header.flags, FrameFlags::RawOpaque | FrameFlags::ReplaceLatest);
  EXPECT_EQ(outbound.header.seq, 1u);
  EXPECT_EQ(outbound.payload, payload);
}

TEST(HorusLinkConnectionManagerTest, RealtimeSendIsNotBlockedBySaturatedBulkLane)
{
  auto config = make_config();
  config.keepalive_ms = 0;
  config.socket_buffer_size = 4096;
  HorusLinkConnectionManager manager(config);
  std::atomic<int> connected_id{0};
  manager.set_connection_callback(
    [&connected_id](int connection_id, const std::string &) {
      connected_id = connection_id;
    });
  ASSERT_TRUE(manager.start());

  auto realtime = connect_to(manager.realtime_port());
  auto bulk = connect_to(manager.bulk_port());
  const int client_receive_buffer = 4096;
  ASSERT_EQ(
    setsockopt(
      bulk.get(),
      SOL_SOCKET,
      SO_RCVBUF,
      &client_receive_buffer,
      sizeof(client_receive_buffer)),
    0);
  send_client_hellos(realtime.get(), bulk.get());
  ASSERT_TRUE(wait_until([&connected_id]() {return connected_id.load() != 0;}));
  expect_bridge_hello(realtime.get(), config);

  const SubscribeRequest realtime_request{
    3,
    "/tf",
    "tf2_msgs/msg/TFMessage",
    Lane::Realtime,
    Delivery::ReliableFifo
  };
  send_frame(realtime.get(), make_control_frame(encode_subscribe_request(realtime_request), 1));
  recv_frame(realtime.get());

  const SubscribeRequest bulk_request{
    9,
    "/mesh",
    "horus_msgs/msg/IndexedMesh",
    Lane::Bulk,
    Delivery::ReliableFifo
  };
  send_frame(realtime.get(), make_control_frame(encode_subscribe_request(bulk_request), 2));
  recv_frame(realtime.get());

  const std::vector<uint8_t> bulk_payload(64u * 1024u * 1024u, 0x7E);
  std::atomic<bool> bulk_send_started{false};
  std::atomic<bool> bulk_send_returned{false};
  std::thread bulk_send_thread([&]() {
      bulk_send_started = true;
      manager.send_to_topic(connected_id.load(), "/mesh", bulk_payload);
      bulk_send_returned = true;
    });
  ASSERT_TRUE(wait_until([&bulk_send_started]() {return bulk_send_started.load();}));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_TRUE(bulk_send_returned.load())
    << "a ROS callback must only enqueue, even when the bulk peer stops reading";

  const std::vector<uint8_t> realtime_payload{0x01, 0x02, 0x03};
  std::atomic<bool> realtime_send_returned{false};
  std::atomic<bool> realtime_send_succeeded{false};
  std::thread realtime_send_thread([&]() {
      realtime_send_succeeded = manager.send_to_topic(
        connected_id.load(),
        "/tf",
        realtime_payload);
      realtime_send_returned = true;
    });

  const Frame realtime_frame = recv_frame(realtime.get());
  EXPECT_TRUE(wait_until([&realtime_send_returned]() {return realtime_send_returned.load();}));
  EXPECT_TRUE(realtime_send_succeeded.load());
  EXPECT_EQ(realtime_frame.header.channel_id, 3u);
  EXPECT_EQ(realtime_frame.header.msg_type, MessageType::Data);
  EXPECT_EQ(realtime_frame.header.flags, FrameFlags::RawOpaque);
  EXPECT_EQ(realtime_frame.payload, realtime_payload);

  manager.stop();
  bulk.reset();
  realtime.reset();
  if (realtime_send_thread.joinable()) {
    realtime_send_thread.join();
  }
  if (bulk_send_thread.joinable()) {
    bulk_send_thread.join();
  }
}

TEST(HorusLinkConnectionManagerTest, SlowBulkPeerDoesNotOccupyRosCallbackWorker)
{
  auto config = make_config();
  config.keepalive_ms = 0;
  config.socket_buffer_size = 4096;
  HorusLinkConnectionManager manager(config);
  std::atomic<int> id{0};
  manager.set_connection_callback([&](int value, const std::string &) {id = value;});
  ASSERT_TRUE(manager.start());
  auto realtime = connect_to(manager.realtime_port());
  auto bulk = connect_to(manager.bulk_port());
  int small_buffer = 4096;
  ASSERT_EQ(setsockopt(bulk.get(), SOL_SOCKET, SO_RCVBUF, &small_buffer, sizeof(small_buffer)), 0);
  send_client_hellos(realtime.get(), bulk.get());
  ASSERT_TRUE(wait_until([&]() {return id.load() != 0;}));
  expect_bridge_hello(realtime.get(), config);
  send_frame(realtime.get(), make_control_frame(encode_subscribe_request(
    SubscribeRequest{3, "/tf", "tf2_msgs/msg/TFMessage", Lane::Realtime,
        Delivery::ReplaceLatest})));
  recv_frame(realtime.get());
  send_frame(realtime.get(), make_control_frame(encode_subscribe_request(
    SubscribeRequest{9, "/mesh", "visualization_msgs/msg/Marker", Lane::Bulk,
        Delivery::ReliableFifo})));
  recv_frame(realtime.get());

  // One executor worker must be able to enqueue a map and immediately service TF.
  // The peer deliberately never reads bulk data, so copying cannot mask a socket wait.
  const std::vector<uint8_t> payload(2u * 1024u * 1024u, 0x7E);
  const auto start = std::chrono::steady_clock::now();
  const bool accepted = manager.send_to_topic(id.load(), "/mesh", payload);
  const auto elapsed = std::chrono::steady_clock::now() - start;
  EXPECT_TRUE(accepted);
  EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(), 100);
  if (accepted && manager.send_to_topic(id.load(), "/tf", std::vector<uint8_t>{42})) {
    const Frame frame = recv_frame(realtime.get());
    EXPECT_EQ(frame.header.channel_id, 3u);
    EXPECT_EQ(frame.payload, std::vector<uint8_t>{42});
  }
  manager.stop();
}

namespace
{
struct TestPeer
{
  SocketHandle realtime;
  SocketHandle bulk;

  TestPeer(
    HorusLinkConnectionManager & manager, const HorusLinkConnectionManager::Config & config,
    uint64_t session, bool slow = false)
  : realtime(connect_to(manager.realtime_port())), bulk(connect_to(manager.bulk_port()))
  {
    if (slow) {
      int size = 4096;
      EXPECT_EQ(setsockopt(bulk.get(), SOL_SOCKET, SO_RCVBUF, &size, sizeof(size)), 0);
    }
    send_client_hellos(realtime.get(), bulk.get(), session);
    expect_bridge_hello(realtime.get(), config);
  }

  void subscribe(uint16_t channel, const char * topic, Lane lane, Delivery delivery)
  {
    send_frame(realtime.get(), make_control_frame(encode_subscribe_request(
      SubscribeRequest{channel, topic, "std_msgs/msg/String", lane, delivery})));
    const auto frame = recv_frame(realtime.get());
    const auto ack = decode_subscribe_ack(frame.payload.data(), frame.payload.size());
    ASSERT_TRUE(ack.has_value());
    ASSERT_EQ(ack->status, SubscribeStatus::Accepted);
  }
};
}  // namespace

TEST(HorusLinkConnectionManagerTest, ConcurrentReliableProducersRemainInSequenceOnWire)
{
  auto config = make_config();
  config.keepalive_ms = 0;
  config.bulk_queue_depth = 1024;
  HorusLinkConnectionManager manager(config);
  ASSERT_TRUE(manager.start());
  TestPeer peer(manager, config, 100);
  peer.subscribe(9, "/mesh", Lane::Bulk, Delivery::ReliableFifo);
  constexpr int workers = 4;
  constexpr int per_worker = 100;
  std::vector<std::thread> producers;
  std::atomic<int> accepted{0};
  for (int w = 0; w < workers; ++w) {
    producers.emplace_back([&, w]() {
        for (int i = 0; i < per_worker; ++i) {
          accepted += manager.send_to_topic(1, "/mesh",
            std::vector<uint8_t>(1024, static_cast<uint8_t>(w)));
        }
    });
  }
  for (auto & producer : producers) {
    producer.join();
                                                     }
  ASSERT_EQ(accepted.load(), workers * per_worker);
  for (uint32_t sequence = 1; sequence <= workers * per_worker; ++sequence) {
    const Frame frame = recv_frame(peer.bulk.get());
    EXPECT_EQ(frame.header.seq, sequence);
    EXPECT_EQ(frame.payload.size(), 1024u);
  }
  EXPECT_TRUE(wait_until([&]() {return manager.statistics().at(0).bulk.frames_sent == 400;}));
  manager.stop();
}

TEST(HorusLinkConnectionManagerTest, SlowClientCannotBlockHealthyClientsOrRealtimeCallbacks)
{
  auto config = make_config();
  config.keepalive_ms = 0;
  config.socket_buffer_size = 4096;
  config.socket_send_timeout_ms = 2000;
  HorusLinkConnectionManager manager(config);
  ASSERT_TRUE(manager.start());
  TestPeer slow(manager, config, 100, true);
  TestPeer healthy(manager, config, 101);
  slow.subscribe(9, "/mesh", Lane::Bulk, Delivery::ReliableFifo);
  slow.subscribe(3, "/tf", Lane::Realtime, Delivery::ReliableFifo);
  healthy.subscribe(9, "/mesh", Lane::Bulk, Delivery::ReliableFifo);
  ASSERT_TRUE(manager.send_to_topic(1, "/mesh", std::vector<uint8_t>(2 * 1024 * 1024, 1)));
  std::vector<double> latency_ms;
  for (int i = 0; i < 50; ++i) {
    const auto start = std::chrono::steady_clock::now();
    ASSERT_TRUE(manager.send_to_topic(1, "/tf", std::vector<uint8_t>{static_cast<uint8_t>(i)}));
    EXPECT_EQ(recv_frame(slow.realtime.get()).payload.front(), i);
    latency_ms.push_back(std::chrono::duration<double,
      std::milli>(std::chrono::steady_clock::now() - start).count());
  }
  ASSERT_TRUE(manager.send_to_topic(2, "/mesh", std::vector<uint8_t>{42}));
  EXPECT_EQ(recv_frame(healthy.bulk.get()).payload, std::vector<uint8_t>{42});
  std::sort(latency_ms.begin(), latency_ms.end());
  RecordProperty("realtime_loopback_p95_ms", latency_ms[47]);
  RecordProperty("realtime_loopback_max_ms", latency_ms.back());
  EXPECT_LT(latency_ms[47], 100.0);
  manager.stop();
}

TEST(HorusLinkConnectionManagerTest,
    ReliableOverflowDisconnectsOnlySlowSessionWithoutSilentChunkLoss)
{
  auto config = make_config();
  config.keepalive_ms = 0;
  config.socket_buffer_size = 4096;
  config.socket_send_timeout_ms = 2000;
  config.bulk_queue_bytes = 1024;
  HorusLinkConnectionManager manager(config);
  ASSERT_TRUE(manager.start());
  TestPeer slow(manager, config, 100, true);
  TestPeer healthy(manager, config, 101);
  slow.subscribe(9, "/mesh", Lane::Bulk, Delivery::ReliableFifo);
  healthy.subscribe(3, "/tf", Lane::Realtime, Delivery::ReliableFifo);
  ASSERT_TRUE(manager.send_to_topic(1, "/mesh", std::vector<uint8_t>(2 * 1024 * 1024, 1)));
  ASSERT_TRUE(wait_until([&]() {
      const auto stats = manager.statistics();
      for (const auto & entry : stats) {
        if (entry.connection_id == 1) {return entry.bulk.write_in_progress;}
      }
      return false;
  }));
  ASSERT_TRUE(manager.send_to_topic(1, "/mesh", std::vector<uint8_t>(1000, 2)));
  EXPECT_FALSE(manager.send_to_topic(1, "/mesh", std::vector<uint8_t>(1000, 3)));
  EXPECT_TRUE(wait_for_peer_close(slow.realtime.get()));
  ASSERT_TRUE(manager.send_to_topic(2, "/tf", std::vector<uint8_t>{42}));
  EXPECT_EQ(recv_frame(healthy.realtime.get()).payload, std::vector<uint8_t>{42});
  manager.stop();
}

TEST(HorusLinkConnectionManagerTest, StopInterruptsIncompleteHelloPromptly)
{
  auto config = make_config();
  HorusLinkConnectionManager manager(config);
  ASSERT_TRUE(manager.start());
  auto socket = connect_to(manager.realtime_port());
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  const auto begin = std::chrono::steady_clock::now();
  manager.stop();
  const double elapsed_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - begin).count();
  EXPECT_LT(elapsed_ms, 500.0);
}

TEST(HorusLinkConnectionManagerTest, DisconnectedSessionsAreReapedWithoutAnotherConnection)
{
  auto descriptor_count = []() {
      size_t count = 0;
      auto * dir = opendir("/proc/self/fd");
      if (!dir) {return size_t{0};}
      while (readdir(dir)) {++count;}
      closedir(dir);
      return count;
    };
  auto config = make_config();
  config.keepalive_ms = 0;
  HorusLinkConnectionManager manager(config);
  ASSERT_TRUE(manager.start());
  const auto baseline = descriptor_count();
  ASSERT_GT(baseline, 0u);
  {
    TestPeer peer(manager, config, 105);
    EXPECT_GE(descriptor_count(), baseline + 4);
  }
  EXPECT_TRUE(wait_until([&]() {return descriptor_count() <= baseline;}));
  manager.stop();
}

}  // namespace horus_unity_bridge::horuslink
