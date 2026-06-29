// Copyright 2025 RICE Lab, University of Genoa
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

// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#include "horus_unity_bridge/connection_manager.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <iostream>

#include "horus_unity_bridge/bridge_metrics.hpp"

namespace horus_unity_bridge
{

OutboundMessageQueue::EnqueueResult OutboundMessageQueue::enqueue(
  std::string destination,
  std::vector<uint8_t> serialized,
  OutboundMessagePolicy policy,
  size_t message_limit)
{
  EnqueueResult result;
  const size_t effective_limit = std::max<size_t>(1, message_limit);
  const bool bulk = is_bulk_policy(policy);
  const bool replaceable = is_replaceable_policy(policy);
  const bool strict = !replaceable;
  const std::int64_t enqueue_ns =
    BridgeMetrics::instance().enabled() ? BridgeMetrics::unix_time_ns() : 0;
  QueueList & lane = lane_for(bulk);

  if (replaceable) {
    auto existing_it = replaceable_by_destination_.find(destination);
    if (existing_it != replaceable_by_destination_.end()) {
      QueueIterator it = existing_it->second.it;
      if (it->offset == 0) {
        // Not yet on the wire: coalesce to the newest payload in place. Lossless for maps - only
        // the latest full-resolution frame is ever rendered.
        it->serialized = std::move(serialized);
        it->offset = 0;
        it->enqueue_ns = enqueue_ns;
        result.accepted = true;
        result.replaced_existing = true;
        return result;
      }
      // The current frame for this destination is in-flight and must finish (TCP framing). Drop the
      // stale coalescing slot so the frame we append below becomes the new slot: coalescing then
      // continues "through" the in-flight frame instead of stalling behind it.
      replaceable_by_destination_.erase(existing_it);
    }

    if (!enforce_lane_limit(bulk, true, effective_limit, result)) {
      return result;
    }

    lane.push_back(OutboundMessage{std::move(destination), std::move(serialized), 0, false, true,
        bulk, enqueue_ns});
    QueueIterator inserted_it = std::prev(lane.end());
    replaceable_by_destination_[inserted_it->destination] = ReplaceableRef{bulk, inserted_it};
    result.accepted = true;
    return result;
  }

  if (!enforce_lane_limit(bulk, false, effective_limit, result)) {
    return result;
  }

  lane.push_back(OutboundMessage{std::move(destination), std::move(serialized), 0, strict, false,
      bulk, enqueue_ns});
  result.accepted = true;
  return result;
}

OutboundMessageQueue::QueueIterator OutboundMessageQueue::select_front_iterator(bool & out_bulk)
{
  // Pin an in-flight frame (offset>0) first, whichever lane it is in; otherwise prefer Realtime
  // over Bulk. Only one frame is ever in-flight because the writer finishes (or pins) a frame
  // before moving on.
  if (!realtime_.empty() && realtime_.front().offset > 0) {
    out_bulk = false; return realtime_.begin();
  }
  if (!bulk_.empty() && bulk_.front().offset > 0) {out_bulk = true; return bulk_.begin();}
  if (!realtime_.empty()) {out_bulk = false; return realtime_.begin();}
  if (!bulk_.empty()) {out_bulk = true; return bulk_.begin();}
  out_bulk = false;
  return realtime_.end();
}

OutboundMessage * OutboundMessageQueue::front()
{
  bool bulk = false;
  QueueIterator it = select_front_iterator(bulk);
  if (it == lane_for(bulk).end()) {
    return nullptr;
  }
  return &(*it);
}

const OutboundMessage * OutboundMessageQueue::front() const
{
  if (!realtime_.empty() && realtime_.front().offset > 0) {return &realtime_.front();}
  if (!bulk_.empty() && bulk_.front().offset > 0) {return &bulk_.front();}
  if (!realtime_.empty()) {return &realtime_.front();}
  if (!bulk_.empty()) {return &bulk_.front();}
  return nullptr;
}

bool OutboundMessageQueue::front_is_replaceable() const
{
  const OutboundMessage * f = front();
  return f != nullptr && f->replaceable;
}

void OutboundMessageQueue::mark_front_in_flight()
{
  bool bulk = false;
  QueueIterator it = select_front_iterator(bulk);
  if (it == lane_for(bulk).end() || !it->replaceable) {
    return;
  }

  auto replaceable_it = replaceable_by_destination_.find(it->destination);
  if (replaceable_it != replaceable_by_destination_.end() && replaceable_it->second.it == it) {
    replaceable_by_destination_.erase(replaceable_it);
  }
  it->replaceable = false;
}

bool OutboundMessageQueue::advance_front(size_t bytes_sent)
{
  bool bulk = false;
  QueueIterator it = select_front_iterator(bulk);
  if (it == lane_for(bulk).end()) {
    return false;
  }

  it->offset += bytes_sent;
  if (it->offset < it->serialized.size()) {
    return false;
  }

  erase_message(bulk, it);
  return true;
}

bool OutboundMessageQueue::enforce_lane_limit(
  bool bulk, bool replaceable, size_t limit,
  EnqueueResult & result)
{
  QueueList & lane = lane_for(bulk);
  while (lane.size() >= limit) {
    if (bulk) {
      // Map back-pressure: shed the oldest not-yet-sent map frame instead of force-disconnecting
      // the client. The SDK re-latches under TRANSIENT_LOCAL / periodic republish, so the map
      // re-converges without tearing down the whole connection.
      QueueIterator victim = find_first_evictable(bulk_);
      if (victim == bulk_.end()) {
        result.dropped_new_message = true;
        return false;
      }
      erase_message(true, victim);
      result.evicted_old_message = true;
    } else if (replaceable) {
      QueueIterator victim = find_first_replaceable_evictable(realtime_);
      if (victim == realtime_.end()) {
        result.dropped_new_message = true;
        return false;
      }
      erase_message(false, victim);
      result.evicted_old_message = true;
    } else {
      // Realtime strict overflow: the client is not draining its socket, i.e. genuinely broken.
      result.should_disconnect = true;
      return false;
    }
  }
  return true;
}

OutboundMessageQueue::QueueIterator OutboundMessageQueue::find_first_evictable(QueueList & lane)
{
  return std::find_if(lane.begin(), lane.end(),
           [](const OutboundMessage & message) {return message.offset == 0;});
}

OutboundMessageQueue::QueueIterator OutboundMessageQueue::find_first_replaceable_evictable(
  QueueList & lane)
{
  return std::find_if(lane.begin(), lane.end(),
           [](const OutboundMessage & message) {
             return message.replaceable && message.offset == 0;
      });
}

void OutboundMessageQueue::erase_message(bool bulk, QueueIterator it)
{
  QueueList & lane = lane_for(bulk);
  if (it == lane.end()) {
    return;
  }

  if (it->replaceable) {
    auto replaceable_it = replaceable_by_destination_.find(it->destination);
    if (replaceable_it != replaceable_by_destination_.end() && replaceable_it->second.it == it) {
      replaceable_by_destination_.erase(replaceable_it);
    }
  }

  lane.erase(it);
}

bool OutboundMessageQueue::is_replaceable_policy(OutboundMessagePolicy policy)
{
  return policy == OutboundMessagePolicy::Replaceable ||
         policy == OutboundMessagePolicy::BulkReplaceable;
}

bool OutboundMessageQueue::is_strict_policy(OutboundMessagePolicy policy)
{
  return policy == OutboundMessagePolicy::Strict ||
         policy == OutboundMessagePolicy::BulkStrict;
}

bool OutboundMessageQueue::is_bulk_policy(OutboundMessagePolicy policy)
{
  return policy == OutboundMessagePolicy::BulkStrict ||
         policy == OutboundMessagePolicy::BulkReplaceable;
}

ConnectionManager::ConnectionManager(const Config & config)
: config_(config),
  server_fd_(-1),
  epoll_fd_(-1),
  running_(false)
{
  stats_ = Statistics{};
}

ConnectionManager::~ConnectionManager()
{
  stop();
}

bool ConnectionManager::start()
{
  if (running_.load()) {
    return true;
  }

  if (!setup_server_socket()) {
    std::cerr << "Failed to setup server socket" << std::endl;
    return false;
  }

  if (!setup_epoll()) {
    std::cerr << "Failed to setup epoll" << std::endl;
    close(server_fd_);
    return false;
  }

  running_ = true;

  // Start a single epoll consumer for both accepts and client I/O.
  io_thread_ = std::thread(&ConnectionManager::io_loop, this);

  std::cout << "Connection manager started on " << config_.bind_address
            << ":" << config_.port << std::endl;

  return true;
}

void ConnectionManager::stop()
{
  if (!running_.load()) {
    return;
  }

  running_ = false;

  // Wait for worker thread to finish.
  if (io_thread_.joinable()) {
    io_thread_.join();
  }

  // Close all client connections
  {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    for (auto & pair : connections_) {
      close(pair.first);
    }
    connections_.clear();
  }

  // Close epoll and server socket
  if (epoll_fd_ >= 0) {
    close(epoll_fd_);
    epoll_fd_ = -1;
  }

  if (server_fd_ >= 0) {
    close(server_fd_);
    server_fd_ = -1;
  }

  std::cout << "Connection manager stopped" << std::endl;
}

bool ConnectionManager::setup_server_socket()
{
  server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd_ < 0) {
    std::cerr << "Failed to create socket: " << strerror(errno) << std::endl;
    return false;
  }

  // Set SO_REUSEADDR
  int opt = 1;
  if (setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
    std::cerr << "Failed to set SO_REUSEADDR: " << strerror(errno) << std::endl;
    close(server_fd_);
    return false;
  }

  // Bind
  struct sockaddr_in address;
  memset(&address, 0, sizeof(address));
  address.sin_family = AF_INET;
  address.sin_port = htons(config_.port);

  if (config_.bind_address == "0.0.0.0") {
    address.sin_addr.s_addr = INADDR_ANY;
  } else {
    if (inet_pton(AF_INET, config_.bind_address.c_str(), &address.sin_addr) <= 0) {
      std::cerr << "Invalid bind address" << std::endl;
      close(server_fd_);
      return false;
    }
  }

  if (bind(server_fd_, (struct sockaddr *)&address, sizeof(address)) < 0) {
    std::cerr << "Failed to bind: " << strerror(errno) << std::endl;
    close(server_fd_);
    return false;
  }

  // Listen
  if (listen(server_fd_, config_.max_connections) < 0) {
    std::cerr << "Failed to listen: " << strerror(errno) << std::endl;
    close(server_fd_);
    return false;
  }

  // Set non-blocking
  int flags = fcntl(server_fd_, F_GETFL, 0);
  fcntl(server_fd_, F_SETFL, flags | O_NONBLOCK);

  return true;
}

bool ConnectionManager::setup_epoll()
{
  epoll_fd_ = epoll_create1(EPOLL_CLOEXEC);
  if (epoll_fd_ < 0) {
    std::cerr << "Failed to create epoll: " << strerror(errno) << std::endl;
    return false;
  }

  // Add server socket to epoll
  return add_to_epoll(server_fd_, EPOLLIN);
}

bool ConnectionManager::accept_connection()
{
  struct sockaddr_in client_addr;
  socklen_t addr_len = sizeof(client_addr);

  int client_fd = accept(server_fd_, (struct sockaddr *)&client_addr, &addr_len);
  if (client_fd < 0) {
    if (errno != EAGAIN && errno != EWOULDBLOCK) {
      std::cerr << "Accept error: " << strerror(errno) << std::endl;
      std::lock_guard<std::mutex> lock(stats_mutex_);
      stats_.connection_errors++;
    }
    return false;
  }

  // Configure socket
  configure_socket(client_fd);

  // Get client info
  char ip_str[INET_ADDRSTRLEN];
  inet_ntop(AF_INET, &client_addr.sin_addr, ip_str, INET_ADDRSTRLEN);
  uint16_t port = ntohs(client_addr.sin_port);

  // Create connection object
  auto connection = std::make_shared<ClientConnection>(client_fd, ip_str, port);

  {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    connections_[client_fd] = std::move(connection);

    std::lock_guard<std::mutex> stats_lock(stats_mutex_);
    stats_.total_connections++;
    stats_.active_connections++;
  }

  // Add to epoll for I/O
  add_to_epoll(client_fd, EPOLLIN | EPOLLET);

  std::cout << "Client connected: " << ip_str << ":" << port << std::endl;

  // Notify callback
  if (on_connect_callback_) {
    on_connect_callback_(client_fd, ip_str, port);
  }

  return true;
}

void ConnectionManager::io_loop()
{
  const int MAX_EVENTS = 64;
  struct epoll_event events[MAX_EVENTS];
  std::vector<uint8_t> buffer(config_.socket_buffer_size);

  while (running_.load()) {
    int nfds = epoll_wait(epoll_fd_, events, MAX_EVENTS, 100);  // 100ms timeout

    if (nfds < 0) {
      if (errno == EINTR) {continue;}
      std::cerr << "epoll_wait error in I/O: " << strerror(errno) << std::endl;
      break;
    }

    for (int i = 0; i < nfds; i++) {
      int fd = events[i].data.fd;

      if (fd == server_fd_) {
        while (accept_connection()) {
        }
        continue;
      }

      if (events[i].events & (EPOLLERR | EPOLLHUP)) {
        disconnect_client(fd);
        continue;
      }

      if (events[i].events & EPOLLIN) {
        handle_client_read(fd);
      }

      if (events[i].events & EPOLLOUT) {
        handle_client_write(fd);
      }
    }
  }
}

void ConnectionManager::handle_client_read(int client_fd)
{
  auto conn = find_connection(client_fd);
  if (!conn || !conn->connected.load()) {
    return;
  }

  std::vector<uint8_t> buffer(config_.socket_buffer_size);

  while (true) {
    ssize_t bytes_read = recv(client_fd, buffer.data(), buffer.size(), 0);

    if (bytes_read < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        // No more data available
        break;
      }
      // Error - disconnect
      disconnect_client(client_fd);
      return;
    }

    if (bytes_read == 0) {
      // Client closed connection
      disconnect_client(client_fd);
      return;
    }

    // Parse messages
    std::vector<ProtocolMessage> messages;
    conn->protocol->parse_messages(buffer.data(), bytes_read, messages);
    if (conn->protocol->has_parse_error()) {
      std::cerr << "Protocol parse error from client " << client_fd
                << "; disconnecting" << std::endl;
      disconnect_client(client_fd);
      return;
    }

    // Process each message
    for (const auto & msg : messages) {
      if (message_callback_) {
        message_callback_(client_fd, msg);
      }
    }

    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      stats_.messages_received += messages.size();
      stats_.bytes_received += bytes_read;
    }
  }
}

void ConnectionManager::handle_client_write(int client_fd)
{
  auto conn = find_connection(client_fd);
  if (!conn || !conn->connected.load()) {
    return;
  }

  bool should_disconnect = false;
  bool stop = false;

  // Per-invocation drain budget: bound how much of this fd's outbound backlog is flushed before
  // returning to epoll_wait, so a sustained 3D-map (Bulk) stream on the single io_thread cannot
  // starve inbound reads (operator tasks) or other clients. Realtime-priority frame selection is
  // unchanged; we only yield early and resume (re-arming EPOLLOUT in the tail) on the next loop.
  bool budget_hit = false;
  std::size_t drained_bytes = 0;
  std::size_t drained_frames = 0;
  constexpr std::size_t kMaxDrainBytesPerInvocation = 256 * 1024;  // ~4x SO_SNDBUF (65536)
  constexpr std::size_t kMaxDrainFramesPerInvocation = 32;

  // Drain until EAGAIN (socket buffer full) or the queue empties. The queue lock is taken and
  // released PER FRAME, not held across the whole drain, so Realtime producers (TF/camera/scan)
  // can enqueue between frames. Frame selection is Realtime-priority, so once a (small) map frame
  // completes, any queued Realtime frame is sent before the next map frame.
  while (conn->connected.load() && !stop) {
    bool sent_completed = false;
    std::int64_t sent_enqueue_ns = 0;
    std::string sent_destination;
    bool sent_bulk = false;
    std::size_t sent_bytes = 0;
    std::size_t realtime_depth_after = 0;
    std::size_t bulk_depth_after = 0;

    {
      std::lock_guard<std::mutex> queue_lock(conn->queue_mutex);
      auto * message = conn->send_queue.front();
      if (message == nullptr) {
        break;
      }

      ssize_t bytes_sent = send(
        client_fd,
        message->data(),
        message->remaining_size(),
        MSG_NOSIGNAL);

      if (bytes_sent < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          stop = true;
        } else {
          should_disconnect = true;
        }
      } else if (bytes_sent == 0) {
        should_disconnect = true;
      } else {
        const size_t message_size = message->remaining_size();
        // Capture before advance_front may pop the frame, for the B5 time-in-queue metric.
        const std::int64_t enqueue_ns = message->enqueue_ns;
        const bool bulk = message->bulk;
        std::string destination = message->destination;
        const std::size_t message_bytes = message->serialized.size();

        if (static_cast<size_t>(bytes_sent) < message_size &&
          conn->send_queue.front_is_replaceable())
        {
          conn->send_queue.mark_front_in_flight();
        }

        const bool completed = conn->send_queue.advance_front(static_cast<size_t>(bytes_sent));
        if (completed) {
          sent_completed = true;
          sent_enqueue_ns = enqueue_ns;
          sent_destination = std::move(destination);
          sent_bulk = bulk;
          sent_bytes = message_bytes;
          realtime_depth_after = conn->send_queue.realtime_depth();
          bulk_depth_after = conn->send_queue.bulk_depth();
        }

        std::lock_guard<std::mutex> lock(stats_mutex_);
        if (completed) {
          stats_.messages_sent++;
        }
        stats_.bytes_sent += static_cast<uint64_t>(bytes_sent);
        drained_bytes += static_cast<std::size_t>(bytes_sent);
      }
    }

    // B5: emit the completed frame's time-in-queue (queueing latency) + per-lane backlog. Recorded
    // OUTSIDE the queue lock so the CSV write never serializes Realtime producers on the send path.
    if (sent_completed && BridgeMetrics::instance().enabled()) {
      const std::int64_t now_ns = BridgeMetrics::unix_time_ns();
      const double time_in_queue_ms =
        sent_enqueue_ns > 0 ? static_cast<double>(now_ns - sent_enqueue_ns) / 1.0e6 : 0.0;
      char extra[192];
      std::snprintf(
        extra, sizeof(extra),
        "{\"lane\":\"%s\",\"time_in_queue_ms\":%.3f,\"realtime_depth\":%zu,\"bulk_depth\":%zu}",
        sent_bulk ? "bulk" : "realtime", time_in_queue_ms,
        realtime_depth_after, bulk_depth_after);
      BridgeMetrics::instance().record(
        "outbound", "sent", client_fd, sent_destination, sent_bytes, sent_bytes,
        realtime_depth_after + bulk_depth_after, extra);
    }

    if (sent_completed) {
      drained_frames++;
    }
    if (drained_bytes >= kMaxDrainBytesPerInvocation ||
      drained_frames >= kMaxDrainFramesPerInvocation)
    {
      budget_hit = true;
    }

    if (should_disconnect) {
      break;
    }
    if (budget_hit) {
      break;
    }
  }

  if (should_disconnect) {
    disconnect_client(client_fd);
    return;
  }

  bool queue_empty;
  {
    std::lock_guard<std::mutex> queue_lock(conn->queue_mutex);
    queue_empty = conn->send_queue.empty();
  }

  if (queue_empty && conn->connected.load()) {
    modify_epoll(client_fd, EPOLLIN | EPOLLET);
  } else if (budget_hit && !stop && conn->connected.load()) {
    // Yielded mid-drain on the per-invocation budget (socket still writable, not EAGAIN). Under
    // EPOLLET no fresh EPOLLOUT edge arrives on its own, so re-assert EPOLLOUT to make the next
    // epoll_wait resume draining -- after it first services pending reads (operator tasks).
    modify_epoll(client_fd, EPOLLIN | EPOLLOUT | EPOLLET);
  }
}

void ConnectionManager::disconnect_client(int client_fd)
{
  std::shared_ptr<ClientConnection> conn;

  {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    auto it = connections_.find(client_fd);
    if (it == connections_.end()) {
      return;
    }
    conn = it->second;
    conn->connected.store(false);
    connections_.erase(it);

    std::lock_guard<std::mutex> stats_lock(stats_mutex_);
    stats_.active_connections--;
  }

  remove_from_epoll(client_fd);
  close(client_fd);

  std::cout << "Client disconnected: " << conn->client_ip << ":" << conn->client_port << std::endl;

  if (on_disconnect_callback_) {
    on_disconnect_callback_(client_fd, conn->client_ip, conn->client_port);
  }
}

void ConnectionManager::configure_socket(int socket_fd)
{
  // Set TCP_NODELAY
  if (config_.tcp_nodelay) {
    int flag = 1;
    setsockopt(socket_fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
  }

  // Set socket buffer sizes
  int bufsize = config_.socket_buffer_size;
  setsockopt(socket_fd, SOL_SOCKET, SO_RCVBUF, &bufsize, sizeof(bufsize));
  setsockopt(socket_fd, SOL_SOCKET, SO_SNDBUF, &bufsize, sizeof(bufsize));

  // Set non-blocking
  int flags = fcntl(socket_fd, F_GETFL, 0);
  fcntl(socket_fd, F_SETFL, flags | O_NONBLOCK);
}

bool ConnectionManager::add_to_epoll(int fd, uint32_t events)
{
  struct epoll_event ev;
  memset(&ev, 0, sizeof(ev));
  ev.events = events;
  ev.data.fd = fd;

  if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, fd, &ev) < 0) {
    std::cerr << "Failed to add fd to epoll: " << strerror(errno) << std::endl;
    return false;
  }

  return true;
}

bool ConnectionManager::modify_epoll(int fd, uint32_t events)
{
  struct epoll_event ev;
  memset(&ev, 0, sizeof(ev));
  ev.events = events;
  ev.data.fd = fd;

  if (epoll_ctl(epoll_fd_, EPOLL_CTL_MOD, fd, &ev) < 0) {
    std::cerr << "Failed to modify fd in epoll: " << strerror(errno) << std::endl;
    return false;
  }

  return true;
}

bool ConnectionManager::remove_from_epoll(int fd)
{
  if (epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, fd, nullptr) < 0) {
    // Don't log error if fd is already removed
    if (errno != ENOENT) {
      std::cerr << "Failed to remove fd from epoll: " << strerror(errno) << std::endl;
    }
    return false;
  }

  return true;
}

bool ConnectionManager::send_to_client(
  int client_fd, const std::string & destination,
  const std::vector<uint8_t> & payload,
  OutboundMessagePolicy policy)
{
  auto conn = find_connection(client_fd);
  if (!conn || !conn->connected.load()) {
    return false;
  }

  auto serialized = conn->protocol->serialize_message(destination, payload);
  const std::size_t serialized_bytes = serialized.size();
  const auto resolved_policy = resolve_policy(destination, policy);
  const bool bulk_lane =
    resolved_policy == OutboundMessagePolicy::BulkStrict ||
    resolved_policy == OutboundMessagePolicy::BulkReplaceable;

  OutboundMessageQueue::EnqueueResult enqueue_result;
  std::size_t realtime_depth = 0;
  std::size_t bulk_depth = 0;
  {
    std::lock_guard<std::mutex> queue_lock(conn->queue_mutex);
    if (!conn->connected.load()) {
      return false;
    }
    enqueue_result = conn->send_queue.enqueue(
      destination,
      std::move(serialized),
      resolved_policy,
      config_.message_queue_size);
    realtime_depth = conn->send_queue.realtime_depth();
    bulk_depth = conn->send_queue.bulk_depth();
  }

  // B5: emit a metric row whenever a 3D-map frame is shed (bulk overflow) or a new frame is dropped,
  // so per-lane loss under backpressure is observable. Recorded outside the queue lock.
  if (BridgeMetrics::instance().enabled() &&
    (enqueue_result.evicted_old_message || enqueue_result.dropped_new_message))
  {
    char extra[160];
    std::snprintf(
      extra, sizeof(extra),
      "{\"lane\":\"%s\",\"realtime_depth\":%zu,\"bulk_depth\":%zu}",
      bulk_lane ? "bulk" : "realtime", realtime_depth, bulk_depth);
    BridgeMetrics::instance().record(
      "outbound",
      enqueue_result.dropped_new_message ? "dropped" : "evicted",
      client_fd, destination, payload.size(), serialized_bytes,
      realtime_depth + bulk_depth, extra);
  }

  if (enqueue_result.should_disconnect) {
    disconnect_client(client_fd);
    return false;
  }

  if (!enqueue_result.accepted && enqueue_result.dropped_new_message) {
    return true;
  }

  if (enqueue_result.accepted && conn->connected.load()) {
    modify_epoll(client_fd, EPOLLIN | EPOLLOUT | EPOLLET);
  }

  return enqueue_result.accepted || enqueue_result.dropped_new_message;
}

void ConnectionManager::broadcast_message(
  const std::string & destination,
  const std::vector<uint8_t> & payload,
  OutboundMessagePolicy policy)
{
  std::vector<std::pair<int, std::shared_ptr<ClientConnection>>> connections;
  {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    connections.reserve(connections_.size());
    for (const auto & pair : connections_) {
      connections.push_back(pair);
    }
  }

  const auto resolved_policy = resolve_policy(destination, policy);
  std::vector<int> clients_to_disconnect;

  for (const auto & pair : connections) {
    const int client_fd = pair.first;
    const auto & conn = pair.second;
    if (!conn || !conn->connected.load()) {
      continue;
    }

    auto serialized = conn->protocol->serialize_message(destination, payload);

    OutboundMessageQueue::EnqueueResult enqueue_result;
    {
      std::lock_guard<std::mutex> queue_lock(conn->queue_mutex);
      if (!conn->connected.load()) {
        continue;
      }
      enqueue_result = conn->send_queue.enqueue(
        destination,
        std::move(serialized),
        resolved_policy,
        config_.message_queue_size);
    }

    if (enqueue_result.should_disconnect) {
      clients_to_disconnect.push_back(client_fd);
      continue;
    }

    if (enqueue_result.accepted && conn->connected.load()) {
      modify_epoll(client_fd, EPOLLIN | EPOLLOUT | EPOLLET);
    }
  }

  for (int client_fd : clients_to_disconnect) {
    disconnect_client(client_fd);
  }
}

size_t ConnectionManager::get_connection_count() const
{
  std::lock_guard<std::mutex> lock(connections_mutex_);
  return connections_.size();
}

ConnectionManager::Statistics ConnectionManager::get_statistics() const
{
  std::lock_guard<std::mutex> lock(stats_mutex_);
  return stats_;
}

std::shared_ptr<ClientConnection> ConnectionManager::find_connection(int client_fd) const
{
  std::lock_guard<std::mutex> lock(connections_mutex_);
  auto it = connections_.find(client_fd);
  if (it == connections_.end()) {
    return nullptr;
  }
  return it->second;
}

OutboundMessagePolicy ConnectionManager::resolve_policy(
  const std::string & destination,
  OutboundMessagePolicy requested_policy)
{
  if (requested_policy != OutboundMessagePolicy::Auto) {
    return requested_policy;
  }

  (void)destination;
  return OutboundMessagePolicy::Strict;
}

} // namespace horus_unity_bridge
