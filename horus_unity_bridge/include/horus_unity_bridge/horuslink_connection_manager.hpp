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
#include "horus_unity_bridge/horuslink_outbound_frame_router.hpp"
#include "horus_unity_bridge/horuslink_protocol.hpp"
#include "horus_unity_bridge/horuslink_session.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace horus_unity_bridge::horuslink
{

class HorusLinkConnectionManager
{
public:
  struct Config
  {
    std::string bind_address = "0.0.0.0";
    uint16_t realtime_port = 10000;
    uint16_t bulk_port = 10001;
    int max_connections = 10;
    size_t socket_buffer_size = 256u * 1024u;
    size_t max_payload_size = 512u * 1024u * 1024u;
    size_t realtime_queue_depth = 1024;
    size_t bulk_queue_depth = 1024;
    uint32_t keepalive_ms = 1000;
    bool tcp_nodelay = true;
  };

  using FrameCallback = std::function<void(
        int connection_id,
        Lane lane,
        const ChannelDescriptor & channel,
        const Frame & frame)>;
  using SubscribeCallback = std::function<bool(
        int connection_id,
        const ChannelDescriptor & channel,
        std::string & error)>;
  using PublisherCallback = std::function<bool(
        int connection_id,
        const ChannelDescriptor & channel,
        std::string & error)>;
  using ServiceClientCallback = std::function<bool(
        int connection_id,
        const ChannelDescriptor & channel,
        std::string & error)>;
  using ConnectionCallback = std::function<void(int connection_id, const std::string & ip)>;

  explicit HorusLinkConnectionManager(Config config);
  ~HorusLinkConnectionManager();

  HorusLinkConnectionManager(const HorusLinkConnectionManager &) = delete;
  HorusLinkConnectionManager & operator=(const HorusLinkConnectionManager &) = delete;

  bool start();
  void stop();

  bool send_to_topic(
    int connection_id,
    const std::string & topic,
    const uint8_t * payload,
    size_t size);

  bool send_to_topic(
    int connection_id,
    const std::string & topic,
    const std::vector<uint8_t> & payload);

  bool send_service_response(
    int connection_id,
    const std::string & service_name,
    uint32_t corr_id,
    const std::vector<uint8_t> & payload);

  void broadcast_to_topic(const std::string & topic, const std::vector<uint8_t> & payload);

  void set_frame_callback(FrameCallback callback)
  {
    frame_callback_ = std::move(callback);
  }
  void set_subscribe_callback(SubscribeCallback callback)
  {
    subscribe_callback_ = std::move(callback);
  }
  void set_publisher_callback(PublisherCallback callback)
  {
    publisher_callback_ = std::move(callback);
  }
  void set_service_client_callback(ServiceClientCallback callback)
  {
    service_client_callback_ = std::move(callback);
  }
  void set_connection_callback(ConnectionCallback callback)
  {
    connection_callback_ = std::move(callback);
  }
  void set_disconnection_callback(ConnectionCallback callback)
  {
    disconnection_callback_ = std::move(callback);
  }

  size_t connection_count() const;
  uint16_t realtime_port() const;
  uint16_t bulk_port() const;

private:
  struct AcceptedSocket
  {
    int fd = -1;
    std::string ip;
  };

  struct Connection
  {
    Connection(
      int connection_id,
      int realtime_socket,
      int bulk_socket,
      std::string remote_ip,
      size_t max_payload_size,
      size_t realtime_queue_depth,
      size_t bulk_queue_depth);
    ~Connection();

    int id = 0;
    int realtime_fd = -1;
    int bulk_fd = -1;
    std::string ip;
    FrameParser realtime_parser;
    FrameParser bulk_parser;
    Session session;
    OutboundFrameRouter outbound;
    std::atomic<bool> connected{true};
    std::mutex mutex;
    std::thread realtime_thread;
    std::thread bulk_thread;
  };

  bool setup_server_socket(uint16_t requested_port, int & out_fd, uint16_t & out_port);
  void accept_loop();
  void accept_ready_sockets(int listen_fd, std::vector<AcceptedSocket> & out_sockets);
  void pair_pending_sockets();
  void start_connection(AcceptedSocket realtime_socket, AcceptedSocket bulk_socket);
  void read_loop(std::shared_ptr<Connection> connection, Lane lane);
  void handle_received_frame(
    const std::shared_ptr<Connection> & connection,
    Lane lane,
    const Frame & frame);
  bool handle_subscribe_request(
    const std::shared_ptr<Connection> & connection,
    const SubscribeRequest & request);
  bool handle_publisher_request(
    const std::shared_ptr<Connection> & connection,
    const PublisherRequest & request);
  bool handle_service_client_request(
    const std::shared_ptr<Connection> & connection,
    const ServiceClientRequest & request);
  bool send_frame(const std::shared_ptr<Connection> & connection, Lane lane, const Frame & frame);
  bool drain_lane(const std::shared_ptr<Connection> & connection, Lane lane);
  void disconnect_connection(int connection_id);
  void prune_disconnected_connections();
  std::shared_ptr<Connection> find_connection(int connection_id) const;
  void configure_socket(int socket_fd) const;

  static void close_fd(int & fd);
  static bool set_nonblocking(int fd);
  static bool send_all(int fd, const uint8_t * data, size_t size);

  Config config_;
  int realtime_server_fd_ = -1;
  int bulk_server_fd_ = -1;
  uint16_t bound_realtime_port_ = 0;
  uint16_t bound_bulk_port_ = 0;
  std::atomic<bool> running_{false};
  std::thread accept_thread_;
  mutable std::mutex mutex_;
  std::vector<AcceptedSocket> pending_realtime_;
  std::vector<AcceptedSocket> pending_bulk_;
  std::unordered_map<int, std::shared_ptr<Connection>> connections_;
  int next_connection_id_ = 1;
  FrameCallback frame_callback_;
  SubscribeCallback subscribe_callback_;
  PublisherCallback publisher_callback_;
  ServiceClientCallback service_client_callback_;
  ConnectionCallback connection_callback_;
  ConnectionCallback disconnection_callback_;
};

}  // namespace horus_unity_bridge::horuslink
