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

#pragma once

#include "horus_unity_bridge/horuslink_channel_table.hpp"
#include "horus_unity_bridge/horuslink_control_messages.hpp"
#include "topic_manager.hpp"
#include "service_manager.hpp"
#include "control_lease_manager.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <atomic>
#include <memory>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <mutex>
#include <vector>

#ifdef ENABLE_WEBRTC
#include "horus_unity_bridge/webrtc_manager.hpp"
#include "horus_unity_bridge/native_media_relay.hpp"
#include "horus_unity_bridge/control_link.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <condition_variable>
#include <variant>
#include <deque>
#include <thread>
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
  using SendCallback = std::function<bool(
        int connection_id,
        const std::string &,
        const std::vector<uint8_t> &)>;
  using HorusLinkServiceResponseCallback = std::function<bool(
        int connection_id,
        const std::string & service_name,
        uint32_t corr_id,
        const std::vector<uint8_t> & response)>;

  explicit MessageRouter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  MessageRouter(const rclcpp::NodeOptions & options, bool media_only);
  ~MessageRouter();

  /**
   * @brief Set callbacks for sending messages to Unity clients
   */
  void set_send_callback(SendCallback callback)
  {
    send_callback_ = std::move(callback);
  }

  void set_horuslink_service_response_callback(HorusLinkServiceResponseCallback callback)
  {
    horuslink_service_response_callback_ = std::move(callback);
  }

  /**
   * @brief Build the HorusLink topic discovery table from the current ROS graph.
   */
  std::vector<horuslink::TopicEntry> get_horuslink_topic_table();

  /**
   * @brief Register a HorusLink topic subscription after channel negotiation.
   */
  bool register_horuslink_subscriber(
    int connection_id,
    const horuslink::ChannelDescriptor & channel);

  /**
   * @brief Replay cached retained-state payloads to a client after its channel is active.
   *
   * Called once a HorusLink subscribe has been acknowledged (channel active) so that a
   * late-joining or reconnecting client is deterministically re-synced with the latched
   * registration / multi-operator replay / static-TF state, without disturbing already
   * connected clients. A no-op for topics that carry no cached retained state.
   */
  void replay_retained_payloads(int connection_id, const std::string & topic);

  /**
   * @brief Register a HorusLink publisher after channel negotiation.
   */
  bool register_horuslink_publisher(
    int connection_id,
    const horuslink::ChannelDescriptor & channel);

  /**
   * @brief Register a HorusLink ROS service client after channel negotiation.
   */
  bool register_horuslink_ros_service(
    int connection_id,
    const horuslink::ChannelDescriptor & channel);

  /**
   * @brief Unregister a HorusLink topic subscription after channel close.
   */
  bool unregister_horuslink_subscriber(
    int connection_id,
    const horuslink::ChannelDescriptor & channel);

  /**
   * @brief Unregister a HorusLink publisher after channel close.
   */
  bool unregister_horuslink_publisher(
    int connection_id,
    const horuslink::ChannelDescriptor & channel);

  /**
   * @brief Unregister a HorusLink ROS service client after channel close.
   */
  bool unregister_horuslink_ros_service(
    int connection_id,
    const horuslink::ChannelDescriptor & channel);

  /**
   * @brief Route a HorusLink data frame to a ROS topic.
   */
  bool route_horuslink_data_frame(
    int connection_id,
    const horuslink::ChannelDescriptor & channel,
    const std::vector<uint8_t> & payload,
    uint8_t frame_flags);

  /**
   * @brief Route a HorusLink service request frame to a ROS service.
   */
  bool route_horuslink_service_request(
    int connection_id,
    const horuslink::ChannelDescriptor & channel,
    uint32_t corr_id,
    const std::vector<uint8_t> & request);

  /**
   * @brief Get topic manager
   */
  TopicManager & get_topic_manager() {return *topic_manager_;}

  /**
   * @brief Get service manager
   */
  ServiceManager & get_service_manager() {return *service_manager_;}

  /**
   * @brief Get routing statistics
   */
  struct Statistics
  {
    uint64_t messages_routed = 0;
    uint64_t messages_published = 0;
    uint64_t messages_received = 0;
    uint64_t system_commands_processed = 0;
    uint64_t routing_errors = 0;
  };

  Statistics get_statistics() const
  {
    return Statistics{
      stats_.messages_routed.load(), stats_.messages_published.load(),
      stats_.messages_received.load(), stats_.system_commands_processed.load(),
      stats_.routing_errors.load()};
  }
  void set_log_protocol_messages(bool enabled);

private:
  // Managers
  std::unique_ptr<TopicManager> topic_manager_;
  std::unique_ptr<ServiceManager> service_manager_;
  std::unique_ptr<ControlLeaseManager> control_lease_manager_;

#ifdef ENABLE_WEBRTC
  using FrameVariant = std::variant<
    sensor_msgs::msg::Image::SharedPtr,
    sensor_msgs::msg::CompressedImage::SharedPtr>;

  struct SharedEncoderGroup;
  struct WebRtcSession
  {
    std::string session_id;
    std::string offer_identity;
    std::mutex signaling_mutex;
    nlohmann::json last_answer;
    int owner_client_fd = -1;
    std::string stream_topic;
    std::string auxiliary_topic;
    std::string image_type;
    bool preencoded_h264 = false;
    bool packet_relay = false;
    std::atomic<bool> h264_queue_discontinuity{false};
    std::shared_ptr<WebRTCManager> manager;
    std::shared_ptr<SharedEncoderGroup> encoder_group;
    std::mutex encoder_delivery_mutex;
    WebRTCManager::Settings source_settings;
    std::atomic<int> requested_bitrate{2000}, requested_framerate{30}, capacity_bitrate{100000};
    std::atomic<int> budget_bitrate{100000}, media_priority{1};
    std::atomic<int64_t> last_encoder_rebind_ms{0};
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_sub;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_sub;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr auxiliary_sub;
    std::deque<FrameVariant> frame_queue;
    std::mutex frame_queue_mutex;
    std::condition_variable frame_queue_cv;
    std::thread worker_thread;
    std::atomic<bool> running{false};
    std::chrono::steady_clock::time_point session_start_time{};
    std::atomic<int64_t> last_peer_ready_ms{0};
    uint64_t incoming_frames = 0;
    uint64_t pushed_frames = 0;
    uint64_t dropped_queue_frames = 0;
    std::atomic<uint64_t> incoming_auxiliary_frames{0};
    std::atomic<uint64_t> sent_auxiliary_frames{0};
    std::atomic<uint64_t> dropped_auxiliary_frames{0};
    std::chrono::steady_clock::time_point last_incoming_frame_time{};
    std::chrono::steady_clock::time_point last_push_time{};
    uint64_t last_seen_encoded_units = 0;
    std::chrono::steady_clock::time_point last_encoded_progress_time{};
    std::chrono::steady_clock::time_point last_keyframe_request_time{};
    std::chrono::steady_clock::time_point last_telemetry_log_time{};
    bool waned_unsupported_encoding = false;
  };
  struct SharedEncoderGroup
  {
    std::string key, topic;
    std::shared_ptr<WebRtcSession> worker;
    std::mutex mutex;
    std::unordered_map<std::string, std::weak_ptr<WebRtcSession>> consumers;
  };
  bool share_encoders_ = false;
  size_t max_shared_encoder_groups_ = 24;
  size_t max_webrtc_sessions_ = 64;
  size_t max_shared_encoder_groups_per_camera_ = 3;
  int media_link_budget_kbps_ = 0, media_reserved_kbps_ = 512, media_bulk_floor_kbps_ = 256;
  std::string media_bulk_budget_file_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr media_budget_status_;
  std::mutex shared_encoder_mutex_;
  std::unordered_map<std::string, std::shared_ptr<SharedEncoderGroup>> shared_encoder_groups_;
  rclcpp::TimerBase::SharedPtr shared_encoder_timer_;
  bool attach_shared_encoder(const std::shared_ptr<WebRtcSession> & session);
  void release_shared_encoder(const std::shared_ptr<WebRtcSession> & session);
  void refresh_shared_encoders();
  void stop_shared_encoder(const std::shared_ptr<SharedEncoderGroup> & group);

  std::mutex owned_media_signals_mutex_;
  std::deque<std_msgs::msg::String::SharedPtr> owned_media_signals_;
  rclcpp::CallbackGroup::SharedPtr media_signaling_group_;
  rclcpp::TimerBase::SharedPtr owned_media_signals_timer_;

  bool webrtc_enabled_ = false;
  std::string webrtc_client_signal_topic_ = "/horus/webrtc/client_signal";
  std::string webrtc_server_signal_topic_ = "/horus/webrtc/server_signal";
  int webrtc_default_bitrate_kbps_ = 2000;
  int webrtc_default_framerate_ = 30;
  std::string webrtc_default_encoder_ = "x264enc";
  std::string webrtc_default_pipeline_;
  std::vector<std::string> webrtc_preencoded_topics_;
  std::vector<std::string> webrtc_default_stun_servers_;
  int webrtc_session_timeout_sec_ = 20;
  size_t webrtc_max_queue_size_ = 2;
  std::unique_ptr<NativeMediaRelay> native_media_relay_;
  rclcpp::TimerBase::SharedPtr native_media_relay_timer_;
  rclcpp::TimerBase::SharedPtr native_media_status_timer_;
  rclcpp::TimerBase::SharedPtr media_health_timer_;
  std::unique_ptr<ControlLink> control_link_;
  rclcpp::TimerBase::SharedPtr control_link_timer_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr webrtc_signal_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr webrtc_signal_pub_;
  rclcpp::TimerBase::SharedPtr webrtc_cleanup_timer_;
  std::unordered_map<std::string, std::shared_ptr<WebRtcSession>> webrtc_sessions_;
  std::mutex webrtc_sessions_mutex_;

  void handle_webrtc_signal_msg(const std_msgs::msg::String::SharedPtr msg);
  void handle_webrtc_offer(const nlohmann::json & payload);
  void handle_webrtc_candidate(const nlohmann::json & payload);
  void handle_webrtc_stop(const nlohmann::json & payload);
  void process_webrtc_frames(const std::shared_ptr<WebRtcSession> & session);
  void enqueue_webrtc_raw_frame(
    const std::shared_ptr<WebRtcSession> & session,
    const sensor_msgs::msg::Image::SharedPtr & msg);
  void enqueue_webrtc_compressed_frame(
    const std::shared_ptr<WebRtcSession> & session,
    const sensor_msgs::msg::CompressedImage::SharedPtr & msg);
  bool convert_to_rgb(
    const sensor_msgs::msg::Image & msg,
    std::vector<uint8_t> & output,
    bool & waned_flag);
  bool decompress_to_rgb(
    const sensor_msgs::msg::CompressedImage & msg,
    std::vector<uint8_t> & output,
    int & width,
    int & height,
    bool & waned_flag);
  void remove_webrtc_session(const std::string & session_id);
  void shutdown_webrtc_sessions();
  void cleanup_stale_webrtc_sessions();
  void publish_webrtc_signal(const nlohmann::json & payload);
#endif

  // Callbacks
  SendCallback send_callback_;
  HorusLinkServiceResponseCallback horuslink_service_response_callback_;

  // Statistics
  struct AtomicStatistics
  {
    std::atomic<uint64_t> messages_routed{0};
    std::atomic<uint64_t> messages_published{0};
    std::atomic<uint64_t> messages_received{0};
    std::atomic<uint64_t> system_commands_processed{0};
    std::atomic<uint64_t> routing_errors{0};
  } stats_;

  struct ServiceResponseRoute
  {
    int connection_id = -1;
    uint32_t correlation_id = 0;
    std::string service_name;
  };
  // Wire IDs are connection-local. Only bridge-generated IDs reach ServiceManager.
  std::unordered_map<uint32_t, ServiceResponseRoute> service_response_routes_;
  uint32_t next_ros_service_request_id_ = 0;
  std::mutex service_response_mutex_;
  bool log_protocol_messages_ = true;

public:
  void on_client_disconnected(int connection_id);

private:
  struct ClientResourceState
  {
    std::unordered_set<std::string> subscribed_topics;
    std::unordered_set<std::string> published_topics;
    std::unordered_set<std::string> ros_services;
    std::unordered_set<std::string> unity_services;
    std::unordered_set<std::string> webrtc_session_ids;
  };
  std::unordered_map<int, ClientResourceState> client_resources_;
  std::mutex client_resources_mutex_;
  void track_client_subscriber(int connection_id, const std::string & topic);
  void track_client_publisher(int connection_id, const std::string & topic);
  void track_client_ros_service(int connection_id, const std::string & service_name);
  void track_client_unity_service(int connection_id, const std::string & service_name);
  void track_client_webrtc_session(int connection_id, const std::string & session_id);
  void untrack_client_webrtc_session(int connection_id, const std::string & session_id);
  void untrack_client_subscriber(int connection_id, const std::string & topic);
  void untrack_client_publisher(int connection_id, const std::string & topic);
  void untrack_client_ros_service(int connection_id, const std::string & service_name);
  void untrack_client_unity_service(int connection_id, const std::string & service_name);
};

} // namespace horus_unity_bridge
