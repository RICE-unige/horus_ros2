// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "protocol_handler.hpp"

#include <sys/epoll.h>
#include <atomic>
#include <functional>
#include <memory>
#include <thread>
#include <mutex>
#include <unordered_map>
#include <list>
#include <condition_variable>
#include <vector>

namespace horus_unity_bridge
{

// Forward declaration
class MessageRouter;

enum class OutboundMessagePolicy {
  Auto,
  Strict,
  Replaceable
};

struct OutboundMessage
{
  std::string destination;
  std::vector<uint8_t> serialized;
  size_t offset = 0;
  bool strict = true;
  bool replaceable = false;

  const uint8_t* data() const { return serialized.data() + offset; }
  size_t remaining_size() const { return serialized.size() - offset; }
};

class OutboundMessageQueue
{
public:
  struct EnqueueResult {
    bool accepted = false;
    bool should_disconnect = false;
    bool replaced_existing = false;
    bool dropped_new_message = false;
    bool evicted_old_message = false;
  };

  EnqueueResult enqueue(std::string destination,
                        std::vector<uint8_t> serialized,
                        OutboundMessagePolicy policy,
                        size_t message_limit);

  bool empty() const { return queue_.empty(); }
  size_t size() const { return queue_.size(); }

  OutboundMessage* front();
  const OutboundMessage* front() const;
  bool front_is_replaceable() const;
  void mark_front_in_flight();
  bool advance_front(size_t bytes_sent);

private:
  using QueueList = std::list<OutboundMessage>;
  using QueueIterator = QueueList::iterator;

  QueueIterator find_oldest_replaceable_evictable();
  void erase_iterator(QueueIterator it);
  static bool is_replaceable_policy(OutboundMessagePolicy policy);
  static bool is_strict_policy(OutboundMessagePolicy policy);

  QueueList queue_;
  std::unordered_map<std::string, QueueIterator> replaceable_by_destination_;
};

/**
 * @brief Represents a single client connection
 */
struct ClientConnection
{
  int socket_fd;
  std::string client_ip;
  uint16_t client_port;
  std::unique_ptr<ProtocolHandler> protocol;
  
  // Send queue for this connection
  OutboundMessageQueue send_queue;
  std::mutex queue_mutex;
  
  std::atomic<bool> connected;
  
  ClientConnection(int fd, const std::string& ip, uint16_t port)
    : socket_fd(fd), client_ip(ip), client_port(port),
      protocol(std::make_unique<ProtocolHandler>()),
      connected(true) {}
};

/**
 * @brief High-performance connection manager using epoll for async I/O
 * 
 * Features:
 * - Epoll-based async I/O for multiple clients
 * - Lock-free send queues per connection
 * - Automatic connection lifecycle management
 * - TCP optimizations (TCP_NODELAY, socket buffer sizing)
 * - Graceful connection handling
 */
class ConnectionManager
{
public:
  using MessageCallback = std::function<void(int client_fd, const ProtocolMessage&)>;
  using ConnectionEventCallback = std::function<void(int client_fd, const std::string& ip, uint16_t port)>;
  
  struct Config {
    std::string bind_address = "0.0.0.0";
    uint16_t port = 10000;
    int max_connections = 10;
    size_t socket_buffer_size = 65536;
    size_t message_queue_size = 1000;
    bool tcp_nodelay = true;
    int connection_timeout_ms = 5000;
  };
  
  explicit ConnectionManager(const Config& config);
  ~ConnectionManager();
  
  /**
   * @brief Start the connection manager
   * Begins listening for connections and processing I/O
   */
  bool start();
  
  /**
   * @brief Stop the connection manager
   * Gracefully closes all connections
   */
  void stop();
  
  /**
   * @brief Send a message to a specific client
   */
  bool send_to_client(int client_fd, const std::string& destination,
                      const std::vector<uint8_t>& payload,
                      OutboundMessagePolicy policy = OutboundMessagePolicy::Auto);
  
  /**
   * @brief Broadcast a message to all connected clients
   */
  void broadcast_message(const std::string& destination,
                        const std::vector<uint8_t>& payload,
                        OutboundMessagePolicy policy = OutboundMessagePolicy::Auto);
  
  /**
   * @brief Set callback for incoming messages
   */
  void set_message_callback(MessageCallback callback) {
    message_callback_ = std::move(callback);
  }
  
  /**
   * @brief Set callbacks for connection events
   */
  void set_connection_callback(ConnectionEventCallback on_connect) {
    on_connect_callback_ = std::move(on_connect);
  }
  
  void set_disconnection_callback(ConnectionEventCallback on_disconnect) {
    on_disconnect_callback_ = std::move(on_disconnect);
  }
  
  /**
   * @brief Get number of active connections
   */
  size_t get_connection_count() const;
  
  /**
   * @brief Get connection statistics
   */
  struct Statistics {
    uint64_t total_connections = 0;
    uint64_t active_connections = 0;
    uint64_t messages_sent = 0;
    uint64_t messages_received = 0;
    uint64_t bytes_sent = 0;
    uint64_t bytes_received = 0;
    uint64_t connection_errors = 0;
  };
  
  Statistics get_statistics() const;

private:
  Config config_;
  
  // Socket management
  int server_fd_;
  int epoll_fd_;
  std::atomic<bool> running_;
  
  // Client connections
  std::unordered_map<int, std::shared_ptr<ClientConnection>> connections_;
  mutable std::mutex connections_mutex_;
  
  // Worker threads
  std::thread accept_thread_;
  std::thread io_thread_;
  
  // Callbacks
  MessageCallback message_callback_;
  ConnectionEventCallback on_connect_callback_;
  ConnectionEventCallback on_disconnect_callback_;
  
  // Statistics
  mutable std::mutex stats_mutex_;
  Statistics stats_;
  
  // Private methods
  bool setup_server_socket();
  bool setup_epoll();
  void accept_loop();
  void io_loop();
  
  bool accept_connection();
  void handle_client_read(int client_fd);
  void handle_client_write(int client_fd);
  void disconnect_client(int client_fd);
  std::shared_ptr<ClientConnection> find_connection(int client_fd) const;
  
  bool add_to_epoll(int fd, uint32_t events);
  bool modify_epoll(int fd, uint32_t events);
  bool remove_from_epoll(int fd);
  
  void configure_socket(int socket_fd);
  static OutboundMessagePolicy resolve_policy(const std::string& destination,
                                             OutboundMessagePolicy requested_policy);
  
  // Message processing
  void process_message(int client_fd, const ProtocolMessage& message);
};

} // namespace horus_unity_bridge
