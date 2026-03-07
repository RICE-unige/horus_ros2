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
#include <cstring>
#include <iostream>

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
  const bool strict = is_strict_policy(policy);
  const bool replaceable = is_replaceable_policy(policy);

  if (replaceable) {
    auto existing_it = replaceable_by_destination_.find(destination);
    if (existing_it != replaceable_by_destination_.end()) {
      existing_it->second->serialized = std::move(serialized);
      existing_it->second->offset = 0;
      result.accepted = true;
      result.replaced_existing = true;
      return result;
    }

    while (queue_.size() >= effective_limit) {
      auto evict_it = find_oldest_replaceable_evictable();
      if (evict_it == queue_.end()) {
        result.accepted = false;
        result.dropped_new_message = true;
        return result;
      }
      erase_iterator(evict_it);
      result.evicted_old_message = true;
    }
  } else if (queue_.size() >= effective_limit) {
    result.should_disconnect = true;
    return result;
  }

  queue_.push_back(OutboundMessage{
    std::move(destination),
    std::move(serialized),
    0,
    strict,
    replaceable
  });
  auto inserted_it = std::prev(queue_.end());
  if (replaceable) {
    replaceable_by_destination_[inserted_it->destination] = inserted_it;
  }

  result.accepted = true;
  return result;
}

OutboundMessage* OutboundMessageQueue::front()
{
  if (queue_.empty()) {
    return nullptr;
  }

  return &queue_.front();
}

const OutboundMessage* OutboundMessageQueue::front() const
{
  if (queue_.empty()) {
    return nullptr;
  }

  return &queue_.front();
}

bool OutboundMessageQueue::front_is_replaceable() const
{
  return !queue_.empty() && queue_.front().replaceable;
}

void OutboundMessageQueue::mark_front_in_flight()
{
  if (queue_.empty()) {
    return;
  }

  auto it = queue_.begin();
  if (!it->replaceable) {
    return;
  }

  auto replaceable_it = replaceable_by_destination_.find(it->destination);
  if (replaceable_it != replaceable_by_destination_.end() && replaceable_it->second == it) {
    replaceable_by_destination_.erase(replaceable_it);
  }
  it->replaceable = false;
}

bool OutboundMessageQueue::advance_front(size_t bytes_sent)
{
  if (queue_.empty()) {
    return false;
  }

  auto it = queue_.begin();
  it->offset += bytes_sent;
  if (it->offset < it->serialized.size()) {
    return false;
  }

  erase_iterator(it);
  return true;
}

OutboundMessageQueue::QueueIterator OutboundMessageQueue::find_oldest_replaceable_evictable()
{
  return std::find_if(
    queue_.begin(),
    queue_.end(),
    [](const OutboundMessage& message) {
      return message.replaceable && message.offset == 0;
    });
}

void OutboundMessageQueue::erase_iterator(QueueIterator it)
{
  if (it == queue_.end()) {
    return;
  }

  if (it->replaceable) {
    auto replaceable_it = replaceable_by_destination_.find(it->destination);
    if (replaceable_it != replaceable_by_destination_.end() && replaceable_it->second == it) {
      replaceable_by_destination_.erase(replaceable_it);
    }
  }

  queue_.erase(it);
}

bool OutboundMessageQueue::is_replaceable_policy(OutboundMessagePolicy policy)
{
  return policy == OutboundMessagePolicy::Replaceable;
}

bool OutboundMessageQueue::is_strict_policy(OutboundMessagePolicy policy)
{
  return policy == OutboundMessagePolicy::Strict;
}

ConnectionManager::ConnectionManager(const Config& config)
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
  
  // Start accept thread
  accept_thread_ = std::thread(&ConnectionManager::accept_loop, this);
  
  // Start I/O thread
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
  
  // Wait for threads to finish
  if (accept_thread_.joinable()) {
    accept_thread_.join();
  }
  
  if (io_thread_.joinable()) {
    io_thread_.join();
  }
  
  // Close all client connections
  {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    for (auto& pair : connections_) {
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
  
  if (bind(server_fd_, (struct sockaddr*)&address, sizeof(address)) < 0) {
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

void ConnectionManager::accept_loop()
{
  while (running_.load()) {
    struct epoll_event events[1];
    int nfds = epoll_wait(epoll_fd_, events, 1, 100);  // 100ms timeout
    
    if (nfds < 0) {
      if (errno == EINTR) continue;
      std::cerr << "epoll_wait error in accept: " << strerror(errno) << std::endl;
      break;
    }
    
    for (int i = 0; i < nfds; i++) {
      if (events[i].data.fd == server_fd_) {
        accept_connection();
      }
    }
  }
}

bool ConnectionManager::accept_connection()
{
  struct sockaddr_in client_addr;
  socklen_t addr_len = sizeof(client_addr);
  
  int client_fd = accept(server_fd_, (struct sockaddr*)&client_addr, &addr_len);
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
      if (errno == EINTR) continue;
      std::cerr << "epoll_wait error in I/O: " << strerror(errno) << std::endl;
      break;
    }
    
    for (int i = 0; i < nfds; i++) {
      int fd = events[i].data.fd;
      
      // Skip server socket (handled in accept loop)
      if (fd == server_fd_) {
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
    
    // Process each message
    for (const auto& msg : messages) {
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
  bool queue_empty = false;

  {
    std::lock_guard<std::mutex> queue_lock(conn->queue_mutex);

    while (conn->connected.load() && !conn->send_queue.empty()) {
      auto* message = conn->send_queue.front();
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
          break;
        }
        should_disconnect = true;
        break;
      }

      if (bytes_sent == 0) {
        should_disconnect = true;
        break;
      }

      const size_t message_size = message->remaining_size();
      if (bytes_sent > 0 &&
          static_cast<size_t>(bytes_sent) < message_size &&
          conn->send_queue.front_is_replaceable())
      {
        conn->send_queue.mark_front_in_flight();
      }

      const bool completed = conn->send_queue.advance_front(static_cast<size_t>(bytes_sent));
      if (completed) {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.messages_sent++;
      }

      {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.bytes_sent += static_cast<uint64_t>(bytes_sent);
      }

      if (!completed) {
        break;
      }
    }

    queue_empty = conn->send_queue.empty();
  }

  if (should_disconnect) {
    disconnect_client(client_fd);
    return;
  }

  if (queue_empty && conn->connected.load()) {
    modify_epoll(client_fd, EPOLLIN | EPOLLET);
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

bool ConnectionManager::send_to_client(int client_fd, const std::string& destination,
                                      const std::vector<uint8_t>& payload,
                                      OutboundMessagePolicy policy)
{
  auto conn = find_connection(client_fd);
  if (!conn || !conn->connected.load()) {
    return false;
  }
  
  auto serialized = conn->protocol->serialize_message(destination, payload);
  const auto resolved_policy = resolve_policy(destination, policy);

  OutboundMessageQueue::EnqueueResult enqueue_result;
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

void ConnectionManager::broadcast_message(const std::string& destination,
                                         const std::vector<uint8_t>& payload,
                                         OutboundMessagePolicy policy)
{
  std::vector<std::pair<int, std::shared_ptr<ClientConnection>>> connections;
  {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    connections.reserve(connections_.size());
    for (const auto& pair : connections_) {
      connections.push_back(pair);
    }
  }

  const auto resolved_policy = resolve_policy(destination, policy);
  std::vector<int> clients_to_disconnect;

  for (const auto& pair : connections) {
    const int client_fd = pair.first;
    const auto& conn = pair.second;
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
  const std::string& destination,
  OutboundMessagePolicy requested_policy)
{
  if (requested_policy != OutboundMessagePolicy::Auto) {
    return requested_policy;
  }

  (void)destination;
  return OutboundMessagePolicy::Strict;
}

} // namespace horus_unity_bridge
