// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "protocol_handler.hpp"
#include "topic_manager.hpp"
#include "service_manager.hpp"
#include "control_lease_manager.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <memory>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <mutex>

#ifdef ENABLE_WEBRTC
#include "horus_unity_bridge/webrtc_manager.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <condition_variable>
#include <variant>
#include <deque>
#include <thread>
#include <atomic>
#include <chrono>
#endif

namespace horus_unity_bridge
{

/**
 * @brief Central message router for Unity-ROS2 communication
 * 
 * Single-node architecture that manages all publishers, subscribers, and services.
 * This avoids the overhead of creating separate nodes for each topic.
 * 
 * Features:
 * - Centralized ROS2 node for all topics
 * - Dynamic topic registration
 * - Efficient message routing
 * - Type introspection for dynamic message handling
 * - QoS configuration per topic
 */
class MessageRouter : public rclcpp::Node
{
public:
  using SendCallback = std::function<bool(int client_fd, const std::string&, const std::vector<uint8_t>&)>;
  using BroadcastCallback = std::function<void(const std::string&, const std::vector<uint8_t>&)>;
  
  explicit MessageRouter();
  ~MessageRouter();
  
  /**
   * @brief Set callbacks for sending messages to Unity clients
   */
  void set_send_callback(SendCallback callback) {
    send_callback_ = std::move(callback);
  }
  
  void set_broadcast_callback(BroadcastCallback callback) {
    broadcast_callback_ = std::move(callback);
  }
  
  /**
   * @brief Route an incoming message from Unity
   * 
   * @param client_fd Client that sent the message
   * @param message Protocol message
   * @return true if message was handled successfully
   */
  bool route_message(int client_fd, const ProtocolMessage& message);
  
  /**
   * @brief Handle system commands from Unity
   */
  bool handle_system_command(int client_fd, const ProtocolMessage& message);
  
  /**
   * @brief Get topic manager
   */
  TopicManager& get_topic_manager() { return *topic_manager_; }
  
  /**
   * @brief Get service manager
   */
  ServiceManager& get_service_manager() { return *service_manager_; }
  
  /**
   * @brief Send handshake payload to a client
   */
  void send_handshake(int client_fd);
  
  /**
   * @brief Get routing statistics
   */
  struct Statistics {
    uint64_t messages_routed = 0;
    uint64_t messages_published = 0;
    uint64_t messages_received = 0;
    uint64_t system_commands_processed = 0;
    uint64_t routing_errors = 0;
  };
  
  const Statistics& get_statistics() const { return stats_; }

private:
  // Managers
  std::unique_ptr<TopicManager> topic_manager_;
  std::unique_ptr<ServiceManager> service_manager_;
  std::unique_ptr<ControlLeaseManager> control_lease_manager_;
  
#ifdef ENABLE_WEBRTC
  using FrameVariant = std::variant<
    sensor_msgs::msg::Image::SharedPtr,
    sensor_msgs::msg::CompressedImage::SharedPtr>;

  struct WebRtcSession {
    std::string session_id;
    int owner_client_fd = -1;
    std::string stream_topic;
    std::string image_type;
    std::shared_ptr<WebRTCManager> manager;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_sub;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_sub;
    std::deque<FrameVariant> frame_queue;
    std::mutex frame_queue_mutex;
    std::condition_variable frame_queue_cv;
    std::thread worker_thread;
    std::atomic<bool> running{false};
    std::chrono::steady_clock::time_point session_start_time{};
    std::chrono::steady_clock::time_point last_activity;
    uint64_t incoming_frames = 0;
    uint64_t pushed_frames = 0;
    uint64_t dropped_queue_frames = 0;
    std::chrono::steady_clock::time_point last_incoming_frame_time{};
    std::chrono::steady_clock::time_point last_push_time{};
    uint64_t last_seen_rtp_packets = 0;
    std::chrono::steady_clock::time_point last_rtp_progress_time{};
    std::chrono::steady_clock::time_point last_keyframe_request_time{};
    std::chrono::steady_clock::time_point last_telemetry_log_time{};
    bool warned_unsupported_encoding = false;
  };

  bool webrtc_enabled_ = false;
  std::string webrtc_client_signal_topic_ = "/horus/webrtc/client_signal";
  std::string webrtc_server_signal_topic_ = "/horus/webrtc/server_signal";
  int webrtc_default_bitrate_kbps_ = 2000;
  int webrtc_default_framerate_ = 30;
  std::string webrtc_default_encoder_ = "x264enc";
  std::string webrtc_default_pipeline_;
  std::vector<std::string> webrtc_default_stun_servers_;
  int webrtc_session_timeout_sec_ = 20;
  size_t webrtc_max_queue_size_ = 2;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr webrtc_signal_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr webrtc_signal_pub_;
  rclcpp::TimerBase::SharedPtr webrtc_cleanup_timer_;
  std::unordered_map<std::string, std::shared_ptr<WebRtcSession>> webrtc_sessions_;
  std::mutex webrtc_sessions_mutex_;

  void handle_webrtc_signal_msg(const std_msgs::msg::String::SharedPtr msg);
  void handle_webrtc_offer(const nlohmann::json& payload);
  void handle_webrtc_candidate(const nlohmann::json& payload);
  void handle_webrtc_stop(const nlohmann::json& payload);
  void process_webrtc_frames(const std::shared_ptr<WebRtcSession>& session);
  void enqueue_webrtc_raw_frame(
    const std::shared_ptr<WebRtcSession>& session,
    const sensor_msgs::msg::Image::SharedPtr& msg);
  void enqueue_webrtc_compressed_frame(
    const std::shared_ptr<WebRtcSession>& session,
    const sensor_msgs::msg::CompressedImage::SharedPtr& msg);
  bool convert_to_rgb(
    const sensor_msgs::msg::Image& msg,
    std::vector<uint8_t>& output,
    bool& warned_flag);
  bool decompress_to_rgb(
    const sensor_msgs::msg::CompressedImage& msg,
    std::vector<uint8_t>& output,
    int& width,
    int& height,
    bool& warned_flag);
  void remove_webrtc_session(const std::string& session_id);
  void shutdown_webrtc_sessions();
  void cleanup_stale_webrtc_sessions();
  void publish_webrtc_signal(const nlohmann::json& payload);
#endif

  // Callbacks
  SendCallback send_callback_;
  BroadcastCallback broadcast_callback_;
  
  // Statistics
  Statistics stats_;
  
  // System command handlers
  void handle_subscribe_command(int client_fd, const std::string& params);
  void handle_publish_command(int client_fd, const std::string& params);
  void handle_remove_subscriber_command(int client_fd, const std::string& params);
  void handle_remove_publisher_command(int client_fd, const std::string& params);
  void handle_remove_ros_service_command(int client_fd, const std::string& params);
  void handle_remove_unity_service_command(int client_fd, const std::string& params);
  void handle_ros_service_command(int client_fd, const std::string& params);
  void handle_unity_service_command(int client_fd, const std::string& params);
  void handle_request_command(int client_fd, const std::string& params);
  void handle_response_command(int client_fd, const std::string& params);
  void handle_topic_list_command(int client_fd);
  void handle_handshake_command(int client_fd);
  
  // Service request/response tracking
  uint32_t pending_service_request_id_ = 0;
  uint32_t pending_service_response_id_ = 0;
  // Map service request IDs to client FDs for routing responses
  std::unordered_map<uint32_t, int> service_response_client_;
  std::mutex service_response_mutex_;
  std::mutex send_mutex_; // Protects socket writes for atomicity
  
  // Utility
  void send_error_to_client(int client_fd, const std::string& error_message);
  void send_log_to_client(int client_fd, const std::string& level, const std::string& message);
  
  bool parse_json_params(const std::string& json_str, 
                        std::unordered_map<std::string, std::string>& params);
public:
  void on_client_disconnected(int client_fd);
private:
  struct ClientResourceState {
    std::unordered_set<std::string> subscribed_topics;
    std::unordered_set<std::string> published_topics;
    std::unordered_set<std::string> ros_services;
    std::unordered_set<std::string> unity_services;
    std::unordered_set<std::string> webrtc_session_ids;
  };
  std::unordered_map<int, ClientResourceState> client_resources_;
  std::mutex client_resources_mutex_;
  void track_client_subscriber(int client_fd, const std::string& topic);
  void track_client_publisher(int client_fd, const std::string& topic);
  void track_client_ros_service(int client_fd, const std::string& service_name);
  void track_client_unity_service(int client_fd, const std::string& service_name);
  void track_client_webrtc_session(int client_fd, const std::string& session_id);
  void untrack_client_webrtc_session(int client_fd, const std::string& session_id);
  void untrack_client_subscriber(int client_fd, const std::string& topic);
  void untrack_client_publisher(int client_fd, const std::string& topic);
  void untrack_client_ros_service(int client_fd, const std::string& service_name);
  void untrack_client_unity_service(int client_fd, const std::string& service_name);
};

} // namespace horus_unity_bridge
