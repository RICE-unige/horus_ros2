// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#include "horus_backend/tcp_server.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include <poll.h>
#include <chrono>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <iostream>

namespace horus_backend
{

namespace
{
constexpr std::size_t kMaxBufferedMessageBytes = 1024u * 1024u;
}  // namespace

TcpServer::TcpServer(int port)
: port_(port), server_socket_(-1), running_(false) {}

TcpServer::~TcpServer() {stop();}

void TcpServer::start()
{
  std::lock_guard<std::mutex> lifecycle(lifecycle_mutex_);
  if (running_) {
    return;
  }

  if (!create_socket() || !bind_socket() || !listen_socket()) {
    close_socket();
    throw std::runtime_error("Failed to start TCP server");
  }

  running_ = true;
  try {
    server_thread_ = std::thread(&TcpServer::server_loop, this);
  } catch (...) {
    running_ = false;
    close_socket();
    throw;
  }

  std::cout << "TCP Server started on port " << port_ << std::endl;
}

void TcpServer::stop()
{
  std::lock_guard<std::mutex> lifecycle(lifecycle_mutex_);
  if (!running_.exchange(false)) {
    close_socket();
    return;
  }

  // Wake accept before joining; do not close/reuse its descriptor under the thread.
  shutdown(server_socket_, SHUT_RDWR);

  if (server_thread_.joinable()) {
    server_thread_.join();
  }
  close_socket();
  shutdown_client_sockets();

  std::vector<std::unique_ptr<ClientWorker>> client_threads;
  {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    client_threads.swap(client_threads_);
    client_sockets_.clear();
  }

  for (auto & client_thread : client_threads) {
    if (client_thread->thread.joinable()) {
      client_thread->thread.join();
    }
  }

  std::cout << "TCP Server stopped" << std::endl;
}

bool TcpServer::is_running() const {return running_;}

void TcpServer::set_message_callback(MessageCallback callback)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  message_callback_ = std::move(callback);
}

bool TcpServer::create_socket()
{
  server_socket_ = socket(AF_INET, SOCK_STREAM, 0);
  if (server_socket_ < 0) {
    std::cerr << "Failed to create socket" << std::endl;
    return false;
  }

  // Set socket options
  int opt = 1;
  if (setsockopt(server_socket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) <
    0)
  {
    std::cerr << "Failed to set socket options" << std::endl;
    return false;
  }

  return true;
}

bool TcpServer::bind_socket()
{
  struct sockaddr_in address;
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(port_);

  if (bind(server_socket_, (struct sockaddr *)&address, sizeof(address)) < 0) {
    std::cerr << "Failed to bind socket to port " << port_ << std::endl;
    return false;
  }
  socklen_t size = sizeof(address);
  if (getsockname(server_socket_, reinterpret_cast<sockaddr *>(&address), &size) == 0) {
    port_ = ntohs(address.sin_port);
  }

  return true;
}

bool TcpServer::listen_socket()
{
  if (listen(server_socket_, 3) < 0) {
    std::cerr << "Failed to listen on socket" << std::endl;
    return false;
  }

  return true;
}

void TcpServer::close_socket()
{
  if (server_socket_ >= 0) {
    shutdown(server_socket_, SHUT_RDWR);
    close(server_socket_);
    server_socket_ = -1;
  }
}

void TcpServer::shutdown_client_sockets()
{
  std::lock_guard<std::mutex> lock(clients_mutex_);
  for (int client_socket : client_sockets_) {
    if (client_socket >= 0) {
      shutdown(client_socket, SHUT_RDWR);
    }
  }
}

void TcpServer::unregister_client_socket(int client_socket)
{
  std::lock_guard<std::mutex> lock(clients_mutex_);
  client_sockets_.erase(
    std::remove(client_sockets_.begin(), client_sockets_.end(), client_socket),
    client_sockets_.end());
  close(client_socket);
}

void TcpServer::reap_finished_clients()
{
  std::lock_guard<std::mutex> lock(clients_mutex_);
  for (auto it = client_threads_.begin(); it != client_threads_.end(); ) {
    if ((*it)->done.load()) {
      (*it)->thread.join();
      it = client_threads_.erase(it);
    } else {
      ++it;
    }
  }
}

void TcpServer::server_loop()
{
  while (running_) {
    reap_finished_clients();
    pollfd listener{server_socket_, POLLIN, 0};
    if (poll(&listener, 1, 200) <= 0 || !(listener.revents & POLLIN)) {
      continue;
    }
    struct sockaddr_in address;
    socklen_t addrlen = sizeof(address);

    int client_socket =
      accept(server_socket_, (struct sockaddr *)&address, &addrlen);
    if (client_socket < 0) {
      if (running_) {
        std::cerr << "Failed to accept client connection" << std::endl;
      }
      continue;
    }
    if (!running_) {
      close(client_socket);
      break;
    }

    std::cout << "Client connected from " << inet_ntoa(address.sin_addr)
              << std::endl;

    {
      std::lock_guard<std::mutex> lock(clients_mutex_);
      client_sockets_.push_back(client_socket);
      auto worker = std::make_unique<ClientWorker>();
      auto * state = worker.get();
      client_threads_.push_back(std::move(worker));
      try {
        state->thread = std::thread([this, client_socket, state]() {
          try {
            handle_client(client_socket);
          } catch (const std::exception & e) {
            std::cerr << "TCP client failed: " << e.what() << std::endl;
          } catch (...) {
            std::cerr << "TCP client failed" << std::endl;
          }
          unregister_client_socket(client_socket);
          state->done = true;
        });
      } catch (...) {
        client_threads_.pop_back();
        client_sockets_.pop_back();
        close(client_socket);
      }
    }
  }
}

bool TcpServer::send_response(int client_socket, const std::string & response)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  size_t offset = 0;
  while (running_ && offset < response.size() && std::chrono::steady_clock::now() < deadline) {
    const auto sent = send(
      client_socket, response.data() + offset, response.size() - offset, MSG_NOSIGNAL | MSG_DONTWAIT);
    if (sent > 0) {
      offset += static_cast<size_t>(sent);
    } else if (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
      pollfd writable{client_socket, POLLOUT, 0};
      poll(&writable, 1, 50);
    } else {
      return false;
    }
  }
  return offset == response.size();
}

void TcpServer::handle_client(int client_socket)
{
  struct timeval timeout;
  timeout.tv_sec = 0;
  timeout.tv_usec = 200000;
  setsockopt(client_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

  char buffer[1024] = {0};
  std::string pending_message;

  while (running_) {
    ssize_t bytes_read = recv(client_socket, buffer, sizeof(buffer) - 1, 0);
    if (bytes_read <= 0) {
      if (bytes_read < 0 && running_ &&
        (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR))
      {
        continue;
      }
      break;
    }

    buffer[bytes_read] = '\0';
    pending_message.append(buffer, static_cast<std::size_t>(bytes_read));
    if (pending_message.size() > kMaxBufferedMessageBytes) {
      const std::string response = "{\"status\":\"error\",\"message\":\"message too large\"}\n";
      send_response(client_socket, response);
      break;
    }

    std::size_t newline_pos = std::string::npos;
    while (running_ && (newline_pos = pending_message.find('\n')) != std::string::npos) {
      std::string message = pending_message.substr(0, newline_pos);
      if (!message.empty() && message.back() == '\r') {
        message.pop_back();
      }
      pending_message.erase(0, newline_pos + 1);
      if (message.empty()) {
        continue;
      }

      const std::string response = process_message(message) + "\n";
      if (!send_response(client_socket, response)) {
        return;
      }
    }
  }

  // An unterminated frame is incomplete, including at EOF and during shutdown.
  std::cout << "Client disconnected" << std::endl;
}

std::string TcpServer::process_message(const std::string & message)
{
  MessageCallback callback;
  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    callback = message_callback_;
  }
  if (callback) {
    try {
      return callback(message);
    } catch (...) {
      return "{\"status\":\"error\",\"message\":\"request processing failed\"}";
    }
  }

  // Default response
  return "{\"status\":\"unsupported\",\"message\":\"No request handler configured\"}";
}

}  // namespace horus_backend
