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

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <iostream>
#include <thread>
#include <utility>

namespace horus_unity_bridge::horuslink
{

HorusLinkConnectionManager::Connection::Connection(
  int connection_id,
  int realtime_socket,
  int bulk_socket,
  std::string remote_ip,
  size_t max_payload_size,
  size_t realtime_queue_depth,
  size_t bulk_queue_depth)
: id(connection_id),
  realtime_fd(realtime_socket),
  bulk_fd(bulk_socket),
  ip(std::move(remote_ip)),
  realtime_parser(max_payload_size),
  bulk_parser(max_payload_size),
  outbound(realtime_queue_depth, bulk_queue_depth)
{
}

HorusLinkConnectionManager::Connection::~Connection()
{
  connected = false;
  close_fd(realtime_fd);
  close_fd(bulk_fd);

  const auto current_thread = std::this_thread::get_id();
  if (realtime_thread.joinable()) {
    if (realtime_thread.get_id() == current_thread) {
      realtime_thread.detach();
    } else {
      realtime_thread.join();
    }
  }
  if (bulk_thread.joinable()) {
    if (bulk_thread.get_id() == current_thread) {
      bulk_thread.detach();
    } else {
      bulk_thread.join();
    }
  }
}

HorusLinkConnectionManager::HorusLinkConnectionManager(Config config)
: config_(std::move(config))
{
}

HorusLinkConnectionManager::~HorusLinkConnectionManager()
{
  stop();
}

bool HorusLinkConnectionManager::start()
{
  if (running_.load()) {
    return true;
  }

  if (!setup_server_socket(config_.realtime_port, realtime_server_fd_, bound_realtime_port_)) {
    return false;
  }
  if (!setup_server_socket(config_.bulk_port, bulk_server_fd_, bound_bulk_port_)) {
    close_fd(realtime_server_fd_);
    return false;
  }

  running_ = true;
  accept_thread_ = std::thread(&HorusLinkConnectionManager::accept_loop, this);
  return true;
}

void HorusLinkConnectionManager::stop()
{
  if (!running_.exchange(false)) {
    return;
  }

  close_fd(realtime_server_fd_);
  close_fd(bulk_server_fd_);

  if (accept_thread_.joinable()) {
    accept_thread_.join();
  }

  std::vector<std::shared_ptr<Connection>> connections;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto & entry : connections_) {
      entry.second->connected = false;
      close_fd(entry.second->realtime_fd);
      close_fd(entry.second->bulk_fd);
      connections.push_back(entry.second);
    }
    connections_.clear();
    pending_realtime_.clear();
    pending_bulk_.clear();
  }

  for (auto & connection : connections) {
    if (connection->realtime_thread.joinable()) {
      connection->realtime_thread.join();
    }
    if (connection->bulk_thread.joinable()) {
      connection->bulk_thread.join();
    }
  }
}

bool HorusLinkConnectionManager::send_to_topic(
  int connection_id,
  const std::string & topic,
  const uint8_t * payload,
  size_t size)
{
  auto connection = find_connection(connection_id);
  if (!connection || !connection->connected.load()) {
    return false;
  }

  Lane lane = Lane::Realtime;
  {
    std::lock_guard<std::mutex> lock(connection->mutex);
    auto result = connection->outbound.enqueue_data(
      connection->session.channel_table(),
      topic,
      payload,
      size);
    if (!result.accepted) {
      return false;
    }
    lane = result.lane;
  }

  return drain_lane(connection, lane);
}

bool HorusLinkConnectionManager::send_to_topic(
  int connection_id,
  const std::string & topic,
  const std::vector<uint8_t> & payload)
{
  return send_to_topic(connection_id, topic, payload.data(), payload.size());
}

void HorusLinkConnectionManager::broadcast_to_topic(
  const std::string & topic,
  const std::vector<uint8_t> & payload)
{
  std::vector<int> connection_ids;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    connection_ids.reserve(connections_.size());
    for (const auto & entry : connections_) {
      connection_ids.push_back(entry.first);
    }
  }

  for (const int connection_id : connection_ids) {
    send_to_topic(connection_id, topic, payload);
  }
}

size_t HorusLinkConnectionManager::connection_count() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return connections_.size();
}

uint16_t HorusLinkConnectionManager::realtime_port() const
{
  return bound_realtime_port_;
}

uint16_t HorusLinkConnectionManager::bulk_port() const
{
  return bound_bulk_port_;
}

bool HorusLinkConnectionManager::setup_server_socket(
  uint16_t requested_port,
  int & out_fd,
  uint16_t & out_port)
{
  int fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    std::cerr << "Failed to create HorusLink listen socket: " << std::strerror(errno)
              << std::endl;
    return false;
  }

  int reuse = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(requested_port);
  if (config_.bind_address == "0.0.0.0") {
    address.sin_addr.s_addr = INADDR_ANY;
  } else if (inet_pton(AF_INET, config_.bind_address.c_str(), &address.sin_addr) <= 0) {
    std::cerr << "Invalid HorusLink bind address: " << config_.bind_address << std::endl;
    close_fd(fd);
    return false;
  }

  if (bind(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0) {
    std::cerr << "Failed to bind HorusLink listen socket: " << std::strerror(errno) << std::endl;
    close_fd(fd);
    return false;
  }
  if (listen(fd, config_.max_connections) != 0) {
    std::cerr << "Failed to listen on HorusLink socket: " << std::strerror(errno) << std::endl;
    close_fd(fd);
    return false;
  }
  if (!set_nonblocking(fd)) {
    close_fd(fd);
    return false;
  }

  sockaddr_in bound_address{};
  socklen_t length = sizeof(bound_address);
  if (getsockname(fd, reinterpret_cast<sockaddr *>(&bound_address), &length) != 0) {
    close_fd(fd);
    return false;
  }

  out_fd = fd;
  out_port = ntohs(bound_address.sin_port);
  return true;
}

void HorusLinkConnectionManager::accept_loop()
{
  while (running_.load()) {
    pollfd fds[2]{};
    fds[0].fd = realtime_server_fd_;
    fds[0].events = POLLIN;
    fds[1].fd = bulk_server_fd_;
    fds[1].events = POLLIN;

    const int ready = poll(fds, 2, 100);
    if (ready < 0) {
      if (errno == EINTR) {
        continue;
      }
      if (running_.load()) {
        std::cerr << "HorusLink accept poll failed: " << std::strerror(errno) << std::endl;
      }
      break;
    }
    if (ready == 0) {
      continue;
    }

    if ((fds[0].revents & POLLIN) != 0) {
      accept_ready_sockets(realtime_server_fd_, pending_realtime_);
    }
    if ((fds[1].revents & POLLIN) != 0) {
      accept_ready_sockets(bulk_server_fd_, pending_bulk_);
    }
    pair_pending_sockets();
  }
}

void HorusLinkConnectionManager::accept_ready_sockets(
  int listen_fd,
  std::vector<AcceptedSocket> & out_sockets)
{
  while (running_.load()) {
    sockaddr_in client_address{};
    socklen_t address_length = sizeof(client_address);
    int fd = accept(listen_fd, reinterpret_cast<sockaddr *>(&client_address), &address_length);
    if (fd < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK && running_.load()) {
        std::cerr << "HorusLink accept failed: " << std::strerror(errno) << std::endl;
      }
      return;
    }

    configure_socket(fd);
    char ip[INET_ADDRSTRLEN]{};
    inet_ntop(AF_INET, &client_address.sin_addr, ip, sizeof(ip));
    out_sockets.push_back(AcceptedSocket{fd, ip});
  }
}

void HorusLinkConnectionManager::pair_pending_sockets()
{
  while (running_.load()) {
    AcceptedSocket realtime_socket;
    AcceptedSocket bulk_socket;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (pending_realtime_.empty() || pending_bulk_.empty()) {
        return;
      }
      realtime_socket = pending_realtime_.front();
      bulk_socket = pending_bulk_.front();
      pending_realtime_.erase(pending_realtime_.begin());
      pending_bulk_.erase(pending_bulk_.begin());
    }

    start_connection(realtime_socket, bulk_socket);
  }
}

void HorusLinkConnectionManager::start_connection(
  AcceptedSocket realtime_socket,
  AcceptedSocket bulk_socket)
{
  int connection_id = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    connection_id = next_connection_id_++;
  }

  auto connection = std::make_shared<Connection>(
    connection_id,
    realtime_socket.fd,
    bulk_socket.fd,
    realtime_socket.ip.empty() ? bulk_socket.ip : realtime_socket.ip,
    config_.max_payload_size,
    config_.realtime_queue_depth,
    config_.bulk_queue_depth);

  realtime_socket.fd = -1;
  bulk_socket.fd = -1;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    connections_[connection_id] = connection;
  }

  connection->realtime_thread = std::thread(
    &HorusLinkConnectionManager::read_loop,
    this,
    connection,
    Lane::Realtime);
  connection->bulk_thread = std::thread(
    &HorusLinkConnectionManager::read_loop,
    this,
    connection,
    Lane::Bulk);

  if (connection_callback_) {
    connection_callback_(connection_id, connection->ip);
  }
}

void HorusLinkConnectionManager::read_loop(std::shared_ptr<Connection> connection, Lane lane)
{
  const int fd = lane == Lane::Bulk ? connection->bulk_fd : connection->realtime_fd;
  auto & parser = lane == Lane::Bulk ? connection->bulk_parser : connection->realtime_parser;
  std::vector<uint8_t> buffer(config_.socket_buffer_size);

  while (running_.load() && connection->connected.load()) {
    const ssize_t read_count = recv(fd, buffer.data(), buffer.size(), 0);
    if (read_count < 0) {
      if (errno == EINTR) {
        continue;
      }
      break;
    }
    if (read_count == 0) {
      break;
    }

    std::vector<Frame> frames;
    parser.parse(buffer.data(), static_cast<size_t>(read_count), frames);
    if (parser.has_error()) {
      break;
    }

    for (const auto & frame : frames) {
      handle_received_frame(connection, lane, frame);
    }
  }

  disconnect_connection(connection->id);
}

void HorusLinkConnectionManager::handle_received_frame(
  const std::shared_ptr<Connection> & connection,
  Lane lane,
  const Frame & frame)
{
  if (frame.header.msg_type == MessageType::Control) {
    auto subscribe_request = decode_subscribe_request(frame.payload.data(), frame.payload.size());
    if (subscribe_request.has_value()) {
      handle_subscribe_request(connection, *subscribe_request);
      return;
    }
  }

  std::vector<std::pair<ChannelDescriptor, Frame>> accepted;
  std::vector<Frame> responses;
  {
    std::lock_guard<std::mutex> lock(connection->mutex);
    responses = connection->session.handle_frame(frame, lane);
    for (const auto & response : responses) {
      if (response.header.msg_type == MessageType::Data ||
        response.header.msg_type == MessageType::ServiceRequest ||
        response.header.msg_type == MessageType::ServiceResponse)
      {
        auto channel = connection->session.channel_table().get(response.header.channel_id);
        if (channel.has_value()) {
          accepted.emplace_back(*channel, response);
        }
      }
    }
  }

  for (const auto & response : responses) {
    if (response.header.msg_type == MessageType::Control) {
      send_frame(connection, Lane::Realtime, response);
    }
  }

  if (frame_callback_) {
    for (const auto & accepted_frame : accepted) {
      frame_callback_(connection->id, lane, accepted_frame.first, accepted_frame.second);
    }
  }
}

bool HorusLinkConnectionManager::handle_subscribe_request(
  const std::shared_ptr<Connection> & connection,
  const SubscribeRequest & request)
{
  SubscribeAck ack;
  ack.channel_id = request.channel_id;
  ack.status = SubscribeStatus::Rejected;
  ack.error = "channel or topic already registered";

  std::optional<ChannelDescriptor> channel;
  {
    std::lock_guard<std::mutex> lock(connection->mutex);
    const bool reserved = connection->session.channel_table().begin_subscribe(
      request.channel_id,
      request.topic,
      request.type_name,
      request.lane,
      request.delivery);
    if (reserved) {
      channel = connection->session.channel_table().get(request.channel_id);
    }
  }

  if (channel.has_value()) {
    std::string callback_error;
    bool accepted = true;
    if (subscribe_callback_) {
      accepted = subscribe_callback_(connection->id, *channel, callback_error);
    }

    if (accepted) {
      std::lock_guard<std::mutex> lock(connection->mutex);
      connection->session.channel_table().confirm_subscribe_ack(request.channel_id);
      ack.status = SubscribeStatus::Accepted;
      ack.error.clear();
    } else {
      std::lock_guard<std::mutex> lock(connection->mutex);
      connection->session.channel_table().remove(request.channel_id);
      ack.error = callback_error.empty() ? "subscription rejected by bridge" : callback_error;
    }
  }

  Frame ack_frame;
  {
    std::lock_guard<std::mutex> lock(connection->mutex);
    ack_frame = connection->session.make_subscribe_ack_frame(ack);
  }
  send_frame(connection, Lane::Realtime, ack_frame);
  return ack.status == SubscribeStatus::Accepted;
}

bool HorusLinkConnectionManager::send_frame(
  const std::shared_ptr<Connection> & connection,
  Lane lane,
  const Frame & frame)
{
  if (!connection || !connection->connected.load()) {
    return false;
  }

  const int fd = lane == Lane::Bulk ? connection->bulk_fd : connection->realtime_fd;
  const auto bytes = serialize_frame(frame.header, frame.payload.data(), frame.payload.size());
  if (!send_all(fd, bytes.data(), bytes.size())) {
    disconnect_connection(connection->id);
    return false;
  }

  return true;
}

bool HorusLinkConnectionManager::drain_lane(
  const std::shared_ptr<Connection> & connection,
  Lane lane)
{
  std::vector<Frame> frames;
  {
    std::lock_guard<std::mutex> lock(connection->mutex);
    while (auto routed = connection->outbound.pop(lane)) {
      frames.push_back(std::move(routed->frame));
    }
  }

  for (const auto & frame : frames) {
    if (!send_frame(connection, lane, frame)) {
      return false;
    }
  }

  return true;
}

void HorusLinkConnectionManager::disconnect_connection(int connection_id)
{
  std::shared_ptr<Connection> connection;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = connections_.find(connection_id);
    if (it == connections_.end()) {
      return;
    }
    connection = it->second;
    connections_.erase(it);
  }

  if (!connection->connected.exchange(false)) {
    return;
  }

  close_fd(connection->realtime_fd);
  close_fd(connection->bulk_fd);

  if (disconnection_callback_) {
    disconnection_callback_(connection_id, connection->ip);
  }
}

std::shared_ptr<HorusLinkConnectionManager::Connection>
HorusLinkConnectionManager::find_connection(int connection_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = connections_.find(connection_id);
  if (it == connections_.end()) {
    return nullptr;
  }

  return it->second;
}

void HorusLinkConnectionManager::configure_socket(int socket_fd) const
{
  if (config_.tcp_nodelay) {
    int flag = 1;
    setsockopt(socket_fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
  }

  int buffer_size = static_cast<int>(config_.socket_buffer_size);
  setsockopt(socket_fd, SOL_SOCKET, SO_RCVBUF, &buffer_size, sizeof(buffer_size));
  setsockopt(socket_fd, SOL_SOCKET, SO_SNDBUF, &buffer_size, sizeof(buffer_size));
}

void HorusLinkConnectionManager::close_fd(int & fd)
{
  if (fd >= 0) {
    shutdown(fd, SHUT_RDWR);
    close(fd);
    fd = -1;
  }
}

bool HorusLinkConnectionManager::set_nonblocking(int fd)
{
  const int flags = fcntl(fd, F_GETFL, 0);
  return flags >= 0 && fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

bool HorusLinkConnectionManager::send_all(int fd, const uint8_t * data, size_t size)
{
  size_t offset = 0;
  while (offset < size) {
    const ssize_t sent = send(fd, data + offset, size - offset, MSG_NOSIGNAL);
    if (sent < 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    if (sent == 0) {
      return false;
    }

    offset += static_cast<size_t>(sent);
  }

  return true;
}

}  // namespace horus_unity_bridge::horuslink
