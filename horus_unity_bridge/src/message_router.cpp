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

#include "horus_unity_bridge/message_router.hpp"
#include "horus_unity_bridge/remote_render_media_metadata.hpp"
#include "horus_unity_bridge/serialized_payload.hpp"
#include "horus_unity_bridge/input_timing.hpp"
#include <rclcpp/serialization.hpp>
#include <cinttypes>

#include <sstream>
#include <nlohmann/json.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <rclcpp/qos.hpp>
#include <algorithm>
#include <cstring>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>

#ifdef ENABLE_WEBRTC
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#endif

namespace horus_unity_bridge
{

MessageRouter::MessageRouter(const rclcpp::NodeOptions & options)
: MessageRouter(options, false)
{}

MessageRouter::MessageRouter(const rclcpp::NodeOptions & options, bool media_only)
: Node("horus_unity_bridge", options)
{
  topic_manager_ = std::make_unique<TopicManager>(this);
  service_manager_ = std::make_unique<ServiceManager>(this);
  const auto control_mode = declare_parameter<std::string>("control.mode", "local");
  if (control_mode != "local" && control_mode != "proxy" && control_mode != "authority") {
    throw std::invalid_argument("control.mode must be local, proxy or authority");
  }
#ifndef ENABLE_WEBRTC
  if (control_mode != "local") {throw std::invalid_argument("Remote control requires ENABLE_WEBRTC");}
#endif
  if ((!media_only || control_mode == "authority") && control_mode != "proxy") {
    control_lease_manager_ = std::make_unique<ControlLeaseManager>(this);
  }

#ifdef ENABLE_WEBRTC
  if (control_mode != "local") {
    control_link_ = std::make_unique<ControlLink>(this, control_mode,
      [this](int client, const horuslink::ChannelDescriptor & channel,
        const std::vector<uint8_t> & data, uint8_t flags) {
        if (control_lease_manager_ && control_lease_manager_->is_internal_topic(channel.topic)) {
          return route_horuslink_data_frame(client, channel, data, flags);
        }
        if (!control_lease_manager_ ||
          !control_lease_manager_->owns_active_remote_lease(client, channel.topic)) {return false;}
        bool registered = false;
        {
          std::lock_guard<std::mutex> lock(client_resources_mutex_);
          const auto found = client_resources_.find(client);
          registered = found != client_resources_.end() &&
            found->second.published_topics.count(channel.topic);
        }
        return (registered || register_horuslink_publisher(client, channel)) &&
               route_horuslink_data_frame(client, channel, data, flags);
      }, [this](int client) {on_client_disconnected(client);},
      [this](const std::string & data) {
        std_msgs::msg::String message; message.data = data;
        rclcpp::SerializedMessage serialized;
        rclcpp::Serialization<std_msgs::msg::String>().serialize_message(&message, &serialized);
        const auto & raw = serialized.get_rcl_serialized_message();
        topic_manager_->publish_inprocess_snapshot(ControlLeaseManager::lease_state_topic(),
          std::vector<uint8_t>(raw.buffer + 4, raw.buffer + raw.buffer_length));
      }, [this]() {return control_lease_manager_ ? control_lease_manager_->protected_topics() :
          std::vector<std::string>{};});
    if (control_lease_manager_) {
      control_lease_manager_->set_snapshot_sink([this](const std::string & state) {
        if (control_link_) {control_link_->publish_lease_state(state);}
      });
    }
    control_link_timer_ = create_wall_timer(std::chrono::milliseconds(5),
      [this]() {control_link_->tick();});
  }
  webrtc_enabled_ = this->declare_parameter<bool>("webrtc.enabled", false);
  webrtc_client_signal_topic_ = this->declare_parameter<std::string>(
    "webrtc.client_signal_topic", "/horus/webrtc/client_signal");
  webrtc_server_signal_topic_ = this->declare_parameter<std::string>(
    "webrtc.server_signal_topic", "/horus/webrtc/server_signal");
  webrtc_default_bitrate_kbps_ = this->declare_parameter<int>(
    "webrtc.default_bitrate_kbps", 2000);
  webrtc_default_framerate_ = this->declare_parameter<int>(
    "webrtc.default_framerate", 30);
  webrtc_default_encoder_ = this->declare_parameter<std::string>(
    "webrtc.default_encoder", "x264enc");
  webrtc_default_pipeline_ = this->declare_parameter<std::string>(
    "webrtc.default_pipeline", "");
  webrtc_preencoded_topics_ = this->declare_parameter<std::vector<std::string>>(
    "webrtc.preencoded_topics", std::vector<std::string>{});
  webrtc_default_stun_servers_ = this->declare_parameter<std::vector<std::string>>(
    "webrtc.default_stun_servers",
    std::vector<std::string>{"stun:stun.l.google.com:19302"});
  webrtc_session_timeout_sec_ = this->declare_parameter<int>(
    "webrtc.session_timeout_sec", 20);
  share_encoders_ = declare_parameter<bool>("webrtc.share_encoders", false);
  max_webrtc_sessions_ = std::clamp<int64_t>(declare_parameter<int>("webrtc.max_sessions", 64), 1, 1024);
  max_shared_encoder_groups_ = std::clamp<int64_t>(declare_parameter<int>("webrtc.max_shared_encoder_groups", 24), 1, 64);
  max_shared_encoder_groups_per_camera_ = std::clamp<int64_t>(
    declare_parameter<int>("webrtc.max_shared_encoder_groups_per_camera", 3), 1, 16);
  media_link_budget_kbps_ = declare_parameter<int>("webrtc.link_budget_kbps", 0);
  media_reserved_kbps_ = declare_parameter<int>("webrtc.reserved_kbps", 512);
  media_bulk_floor_kbps_ = declare_parameter<int>("webrtc.bulk_floor_kbps", 256);
  media_bulk_budget_file_ = declare_parameter<std::string>("webrtc.bulk_budget_file", "");
  if (media_link_budget_kbps_ < 0 || media_link_budget_kbps_ > 10000000 ||
    media_reserved_kbps_ < 0 || media_reserved_kbps_ > 10000000 ||
    media_bulk_floor_kbps_ < 0 || media_bulk_floor_kbps_ > 10000000 ||
    (media_link_budget_kbps_ && (!share_encoders_ ||
    media_bulk_floor_kbps_ < 16 ||
    int64_t(media_link_budget_kbps_) < int64_t(media_reserved_kbps_) + media_bulk_floor_kbps_ + 100)))
  {throw std::invalid_argument("Link governor requires shared encoding and a budget above its reservations");}
  if (media_link_budget_kbps_) {
    media_budget_status_ = create_publisher<std_msgs::msg::String>("/horus/remote/media/bandwidth_budget", 10);
  }

  if (webrtc_enabled_) {
    media_signaling_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    const char * health = std::getenv("HORUS_MEDIA_STATUS_FILE");
    if (health && *health) {
      const std::string path = health;
      auto progress = std::make_shared<std::map<std::string, std::pair<uint64_t, std::chrono::steady_clock::time_point>>>();
      media_health_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this, path, progress]() {
        nlohmann::json tracks = nlohmann::json::object();
        if (native_media_relay_) {
          tracks = native_media_relay_->statistics();
          for (auto & entry : tracks.items()) {
            entry.value()["state"] = entry.value()["connected"] == true &&
              entry.value()["last_packet_age_ms"].get<int64_t>() >= 0 &&
              entry.value()["last_packet_age_ms"].get<int64_t>() < 1000 ? "flowing" : "waiting";
          }
        } else {
          const auto now = std::chrono::steady_clock::now();
          std::lock_guard<std::mutex> lock(webrtc_sessions_mutex_);
          for (const auto & entry : webrtc_sessions_) {
            const auto & session = entry.second;
            const auto frames = session->manager->get_encoded_units_sent();
            auto & previous = (*progress)[entry.first];
            if (frames != previous.first) {previous = {frames, now};}
            tracks[entry.first] = {{"topic", session->stream_topic}, {"forwarded_rtp_frames", frames},
              {"bytes", session->manager->get_encoded_bytes_sent()},
              {"state", frames && now - previous.second < std::chrono::seconds(1) &&
                session->manager->is_peer_connected() ? "flowing" : "waiting"},
              {"transport", session->manager->get_transport_statistics()}};
          }
          for (auto it = progress->begin(); it != progress->end(); ) {
            if (!webrtc_sessions_.count(it->first)) {it = progress->erase(it);} else {++it;}
          }
        }
        std::ofstream output(path + ".new", std::ios::trunc);
        if (!output) {return;}
        output << nlohmann::json({{"pid", getpid()}, {"updated_wall_sec",
          std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count()},
          {"engine", native_media_relay_ ? "native_rtp_no_dds" : "native_source"}, {"tracks", tracks}}).dump();
        output.close();
        if (output) {std::rename((path + ".new").c_str(), path.c_str());}
      }, media_signaling_group_);
    }
    owned_media_signals_timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() {
      std_msgs::msg::String::SharedPtr signal;
      {
        std::lock_guard<std::mutex> lock(owned_media_signals_mutex_);
        if (!owned_media_signals_.empty()) {signal = owned_media_signals_.front(); owned_media_signals_.pop_front();}
      }
      if (signal) {handle_webrtc_signal_msg(signal);}
    }, media_signaling_group_);
    if (share_encoders_) {
      auto status = create_publisher<std_msgs::msg::String>("/horus/remote/media/encoders/status", 10);
      shared_encoder_timer_ = create_wall_timer(std::chrono::milliseconds(200),
        std::function<void()>([this, status, last = std::chrono::steady_clock::now()]() mutable {
          refresh_shared_encoders();
          const auto now = std::chrono::steady_clock::now();
          if (now - last < std::chrono::seconds(1)) {return;}
          last = now;
          nlohmann::json groups = nlohmann::json::array();
          std::lock_guard<std::mutex> lock(shared_encoder_mutex_);
          for (const auto & entry : shared_encoder_groups_) {
            std::lock_guard<std::mutex> clients(entry.second->mutex);
            const auto & worker = entry.second->worker;
            std::lock_guard<std::mutex> frames(worker->frame_queue_mutex);
            groups.push_back({{"topic", entry.second->topic}, {"profile", entry.first},
              {"consumers", entry.second->consumers.size()}, {"input_frames", worker->incoming_frames},
              {"pushed_frames", worker->pushed_frames}, {"queue_dropped_frames", worker->dropped_queue_frames}});
          }
          std_msgs::msg::String message;
          message.data = nlohmann::json({{"groups", groups}, {"group_limit", max_shared_encoder_groups_},
            {"per_camera_limit", max_shared_encoder_groups_per_camera_}}).dump();
          status->publish(message);
        }), media_signaling_group_);
    }
    const auto relay_config = declare_parameter<std::string>("webrtc.relay_config", "");
    if (!relay_config.empty()) {
      native_media_relay_ = std::make_unique<NativeMediaRelay>(this, relay_config);
      native_media_relay_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
        native_media_relay_->tick();
      }, media_signaling_group_);
      auto status = create_publisher<std_msgs::msg::String>("/horus/remote/media/native/status", 10);
      native_media_status_timer_ = create_wall_timer(std::chrono::seconds(1), [this, status]() {
        std_msgs::msg::String message; message.data = native_media_relay_->statistics().dump();
        status->publish(message);
      }, media_signaling_group_);
    }
    auto webrtc_signal_pub_qos = rclcpp::QoS(rclcpp::KeepLast(100))
      .reliability(rclcpp::ReliabilityPolicy::Reliable)
      .durability(rclcpp::DurabilityPolicy::TransientLocal);
    auto webrtc_signal_sub_qos = rclcpp::QoS(rclcpp::KeepLast(100))
      .reliability(rclcpp::ReliabilityPolicy::Reliable)
      .durability(rclcpp::DurabilityPolicy::Volatile);
    webrtc_signal_pub_ = this->create_publisher<std_msgs::msg::String>(
      webrtc_server_signal_topic_, webrtc_signal_pub_qos);
    rclcpp::SubscriptionOptions signaling_options;
    signaling_options.callback_group = media_signaling_group_;
    webrtc_signal_sub_ = this->create_subscription<std_msgs::msg::String>(
      webrtc_client_signal_topic_,
      webrtc_signal_sub_qos,
      std::bind(&MessageRouter::handle_webrtc_signal_msg, this, std::placeholders::_1), signaling_options);
    webrtc_cleanup_timer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&MessageRouter::cleanup_stale_webrtc_sessions, this), media_signaling_group_);
    RCLCPP_INFO(
      get_logger(),
      "WebRTC signaling enabled: client='%s' server='%s'",
      webrtc_client_signal_topic_.c_str(),
      webrtc_server_signal_topic_.c_str());
  } else {
    RCLCPP_INFO(get_logger(), "WebRTC signaling disabled (webrtc.enabled=false)");
  }
#else
  bool webrtc_enabled = this->declare_parameter<bool>("webrtc.enabled", false);
  if (webrtc_enabled) {
    RCLCPP_WARN(
      get_logger(),
      "webrtc.enabled is true but this bridge was built without WebRTC support");
  }
#endif

  service_manager_->set_failure_callback([this](uint32_t request_id) {
    std::lock_guard<std::mutex> lock(service_response_mutex_);
    service_response_routes_.erase(request_id);
  });

  // Setup service manager callback for sending responses to Unity
  service_manager_->set_response_callback(
    [this](uint32_t corr_id, const std::vector<uint8_t> & response) {
      ServiceResponseRoute route;
      {
        std::lock_guard<std::mutex> lock(service_response_mutex_);
        auto it = service_response_routes_.find(corr_id);
        if (it == service_response_routes_.end()) {
          return;  // Timed out, unsubscribed, or disconnected.
        }
        route = std::move(it->second);
        service_response_routes_.erase(it);
      }
      if (horuslink_service_response_callback_) {
        horuslink_service_response_callback_(
          route.connection_id, route.service_name, route.correlation_id,
          detail::strip_cdr_header(response));
        return;
      }

      stats_.routing_errors++;
      RCLCPP_WARN(
        get_logger(),
        "Dropping ROS service response corr_id=%u because no HorusLink response channel is registered",
        corr_id);
    }
  );

  RCLCPP_INFO(get_logger(), "Message router initialized");
}

MessageRouter::~MessageRouter()
{
#ifdef ENABLE_WEBRTC
  control_link_timer_.reset();
  control_link_.reset();
  webrtc_cleanup_timer_.reset();
  media_health_timer_.reset();
  owned_media_signals_timer_.reset();
  shared_encoder_timer_.reset();
  native_media_relay_timer_.reset();
  native_media_status_timer_.reset();
  shutdown_webrtc_sessions();
  native_media_relay_.reset();
#endif
}

std::vector<horuslink::TopicEntry> MessageRouter::get_horuslink_topic_table()
{
  auto topic_names_and_types = get_topic_names_and_types();
  std::vector<horuslink::TopicEntry> entries;
  entries.reserve(topic_names_and_types.size());

  for (const auto & [topic, types] : topic_names_and_types) {
    const std::string type_name = types.empty() ? "unknown" : types.front();
    entries.push_back(horuslink::TopicEntry{
        0,
        topic,
        type_name,
        horuslink::Lane::Realtime,
        horuslink::Delivery::ReliableFifo
      });
  }

  std::sort(
    entries.begin(),
    entries.end(),
    [](const horuslink::TopicEntry & lhs, const horuslink::TopicEntry & rhs) {
      if (lhs.topic != rhs.topic) {
        return lhs.topic < rhs.topic;
      }
      return lhs.type_name < rhs.type_name;
    });

  return entries;
}

bool MessageRouter::register_horuslink_subscriber(
  int connection_id,
  const horuslink::ChannelDescriptor & channel)
{
  if (channel.topic.empty() || channel.type_name.empty()) {
    return false;
  }

  auto callback = [this, connection_id](
      const std::string & topic,
      const std::vector<uint8_t> & data)
    {
      if (send_callback_) {
        send_callback_(connection_id, topic, data);
      }
    };
  bool inprocess = false;
#ifdef ENABLE_WEBRTC
  inprocess = control_link_ && control_link_->is_proxy() &&
    channel.topic == ControlLeaseManager::lease_state_topic();
#endif
  const bool success = inprocess ?
    topic_manager_->register_inprocess_subscriber(channel.topic, channel.type_name, connection_id, callback) :
    topic_manager_->register_horuslink_subscriber(channel.topic, channel.type_name, connection_id,
      callback, rclcpp::QoS(10), channel.live_map);

  if (success) {
    track_client_subscriber(connection_id, channel.topic);
    RCLCPP_INFO(
      get_logger(),
      "Registered HorusLink subscriber: channel=%u topic=%s [%s]",
      channel.channel_id,
      channel.topic.c_str(),
      channel.type_name.c_str());
  } else {
    RCLCPP_WARN(
      get_logger(),
      "Failed to register HorusLink subscriber: channel=%u topic=%s [%s]",
      channel.channel_id,
      channel.topic.c_str(),
      channel.type_name.c_str());
  }

  return success;
}

void MessageRouter::replay_retained_payloads(
  int connection_id,
  const std::string & topic)
{
  if (!send_callback_) {
    return;
  }

  const auto payloads = topic_manager_->get_retained_payloads(topic);
  if (payloads.empty()) {
    return;
  }

  size_t delivered = 0;
  for (const auto & payload : payloads) {
    if (send_callback_(connection_id, topic, payload)) {
      ++delivered;
    }
  }

  RCLCPP_INFO(
    get_logger(),
    "Replayed %zu/%zu retained payload(s) to connection=%d topic=%s",
    delivered,
    payloads.size(),
    connection_id,
    topic.c_str());
}

bool MessageRouter::register_horuslink_publisher(
  int connection_id,
  const horuslink::ChannelDescriptor & channel)
{
#ifdef ENABLE_WEBRTC
  if (control_link_ && control_link_->is_proxy() &&
    (control_link_->handles(channel.topic) ||
    (channel.delivery == horuslink::Delivery::ReplaceLatest &&
    (channel.type_name == "geometry_msgs/msg/Twist" || channel.type_name == "sensor_msgs/msg/Joy" ||
    channel.type_name == "geometry_msgs/msg/PoseStamped"))))
  {
    track_client_publisher(connection_id, channel.topic);
    return true;
  }
#endif
  if (channel.topic.empty() || channel.type_name.empty()) {
    return false;
  }

  const bool success = topic_manager_->register_publisher(
    channel.topic,
    channel.type_name,
    connection_id);

  if (success) {
    track_client_publisher(connection_id, channel.topic);
    RCLCPP_INFO(
      get_logger(),
      "Registered HorusLink publisher: channel=%u topic=%s [%s]",
      channel.channel_id,
      channel.topic.c_str(),
      channel.type_name.c_str());
  } else {
    RCLCPP_WARN(
      get_logger(),
      "Failed to register HorusLink publisher: channel=%u topic=%s [%s]",
      channel.channel_id,
      channel.topic.c_str(),
      channel.type_name.c_str());
  }

  return success;
}

bool MessageRouter::register_horuslink_ros_service(
  int connection_id,
  const horuslink::ChannelDescriptor & channel)
{
  if (channel.topic.empty() || channel.type_name.empty()) {
    return false;
  }

  std::lock_guard<std::mutex> lock(client_resources_mutex_);
  const bool success = service_manager_->register_ros_service(
    channel.topic,
    channel.type_name);

  if (success) {
    client_resources_[connection_id].ros_services.insert(channel.topic);
    RCLCPP_INFO(
      get_logger(),
      "Registered HorusLink ROS service: channel=%u service=%s [%s]",
      channel.channel_id,
      channel.topic.c_str(),
      channel.type_name.c_str());
  } else {
    RCLCPP_WARN(
      get_logger(),
      "Failed to register HorusLink ROS service: channel=%u service=%s [%s]",
      channel.channel_id,
      channel.topic.c_str(),
      channel.type_name.c_str());
  }

  return success;
}

bool MessageRouter::unregister_horuslink_subscriber(
  int connection_id,
  const horuslink::ChannelDescriptor & channel)
{
  if (channel.topic.empty()) {
    return false;
  }

  const bool success = topic_manager_->unregister_subscriber(channel.topic, connection_id);
  untrack_client_subscriber(connection_id, channel.topic);
  if (success) {
    RCLCPP_INFO(
      get_logger(),
      "Unregistered HorusLink subscriber: channel=%u topic=%s",
      channel.channel_id,
      channel.topic.c_str());
  } else {
    RCLCPP_WARN(
      get_logger(),
      "Ignored HorusLink subscriber unregister for missing channel=%u topic=%s",
      channel.channel_id,
      channel.topic.c_str());
  }

  return success;
}

bool MessageRouter::unregister_horuslink_publisher(
  int connection_id,
  const horuslink::ChannelDescriptor & channel)
{
  if (channel.topic.empty()) {
    return false;
  }

  const bool success = topic_manager_->unregister_publisher(channel.topic, connection_id);
  untrack_client_publisher(connection_id, channel.topic);
  if (success) {
    RCLCPP_INFO(
      get_logger(),
      "Unregistered HorusLink publisher: channel=%u topic=%s",
      channel.channel_id,
      channel.topic.c_str());
  } else {
    RCLCPP_WARN(
      get_logger(),
      "Ignored HorusLink publisher unregister for missing channel=%u topic=%s",
      channel.channel_id,
      channel.topic.c_str());
  }

  return success;
}

bool MessageRouter::unregister_horuslink_ros_service(
  int connection_id,
  const horuslink::ChannelDescriptor & channel)
{
  if (channel.topic.empty()) {
    return false;
  }

  std::lock_guard<std::mutex> lock(client_resources_mutex_);
  auto owner = client_resources_.find(connection_id);
  if (owner == client_resources_.end() || !owner->second.ros_services.erase(channel.topic)) {
    return false;
  }
  {
    std::lock_guard<std::mutex> routes_lock(service_response_mutex_);
    for (auto it = service_response_routes_.begin(); it != service_response_routes_.end(); ) {
      if (it->second.connection_id == connection_id && it->second.service_name == channel.topic) {
        service_manager_->cancel_ros_service_call(it->first);
        it = service_response_routes_.erase(it);
      } else {
        ++it;
      }
    }
  }
  const bool shared = std::any_of(client_resources_.begin(), client_resources_.end(),
    [&](const auto & entry) {return entry.second.ros_services.count(channel.topic) != 0;});
  const bool success = shared || service_manager_->unregister_ros_service(channel.topic);
  if (success) {
    RCLCPP_INFO(
      get_logger(),
      "Unregistered HorusLink ROS service: channel=%u service=%s",
      channel.channel_id,
      channel.topic.c_str());
  } else {
    RCLCPP_WARN(
      get_logger(),
      "Ignored HorusLink ROS service unregister for missing channel=%u service=%s",
      channel.channel_id,
      channel.topic.c_str());
  }

  return success;
}

bool MessageRouter::route_horuslink_data_frame(
  int connection_id,
  const horuslink::ChannelDescriptor & channel,
  const std::vector<uint8_t> & payload,
  uint8_t frame_flags)
{
  stats_.messages_routed++;

  int64_t remaining_input_ms = 200;
  std::vector<uint8_t> timed_body;
  const std::vector<uint8_t> * body = &payload;
  if (frame_flags & horuslink::kTimedInputFlag) {
    if (payload.size() < 16) {++stats_.routing_errors; return false;}
    const auto deadline = horuslink::read_timing_u64(payload.data());
    const auto now = uint64_t(horuslink::input_clock_ms());
    if (deadline < now || deadline - now > 300) {++stats_.routing_errors; return false;}
    remaining_input_ms = int64_t(deadline - now);
    timed_body.assign(payload.begin() + 16, payload.end());
    body = &timed_body;
    frame_flags &= ~horuslink::kTimedInputFlag;
  }

  if (channel.topic.empty()) {
    stats_.routing_errors++;
    return false;
  }

#ifdef ENABLE_WEBRTC
  if (control_link_ && control_link_->is_proxy() &&
    (control_link_->handles(channel.topic) ||
    (channel.delivery == horuslink::Delivery::ReplaceLatest &&
    (channel.type_name == "geometry_msgs/msg/Twist" || channel.type_name == "sensor_msgs/msg/Joy" ||
    channel.type_name == "geometry_msgs/msg/PoseStamped"))))
  {
    return control_link_->forward(connection_id, channel, *body, frame_flags, remaining_input_ms);
  }
#endif

  if (control_lease_manager_ && control_lease_manager_->is_internal_topic(channel.topic)) {
    const bool handled = control_lease_manager_->handle_internal_message(
      connection_id,
      channel.topic,
      payload);
    if (!handled) {
      stats_.routing_errors++;
    }
    return handled;
  }

#ifdef ENABLE_WEBRTC
  if (webrtc_enabled_ && channel.topic == webrtc_client_signal_topic_) {
    // Bind local signaling to its actual owner, not a client-supplied fd or a
    // publisher-side activity timer. External ROS signaling remains supported.
    try {
      rclcpp::SerializedMessage serialized;
      detail::fill_serialized_message(detail::add_cdr_header(payload), serialized);
      std_msgs::msg::String message;
      rclcpp::Serialization<std_msgs::msg::String>().deserialize_message(&serialized, &message);
      auto signal = nlohmann::json::parse(message.data);
      signal["client_fd"] = connection_id;
      message.data = signal.dump();
      {
        std::lock_guard<std::mutex> lock(owned_media_signals_mutex_);
        if (owned_media_signals_.size() >= 64) {return false;}
        owned_media_signals_.push_back(std::make_shared<std_msgs::msg::String>(message));
      }
      stats_.messages_published++;
      return true;
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "Rejected malformed owned media signal: %s", error.what());
      stats_.routing_errors++;
      return false;
    }
  }
#endif

  std::string denied_reason;
  std::string denied_robot_name;
  if (control_lease_manager_ &&
    !control_lease_manager_->authorize_command_publish(
      connection_id,
      channel.topic,
      &denied_reason,
      &denied_robot_name))
  {
    stats_.routing_errors++;
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "Denied HorusLink publish from connection %d on protected topic '%s' (robot='%s', reason='%s')",
      connection_id,
      channel.topic.c_str(),
      denied_robot_name.c_str(),
      denied_reason.c_str());
    return false;
  }

  const bool success = topic_manager_->publish_horuslink_message(
    channel.topic,
    *body,
    frame_flags);

  if (success) {
    stats_.messages_published++;
  } else {
    stats_.routing_errors++;
  }

  return success;
}

bool MessageRouter::route_horuslink_service_request(
  int connection_id,
  const horuslink::ChannelDescriptor & channel,
  uint32_t corr_id,
  const std::vector<uint8_t> & request)
{
  stats_.messages_routed++;

  if (channel.topic.empty() || corr_id == 0) {
    stats_.routing_errors++;
    return false;
  }

  std::lock_guard<std::mutex> owners_lock(client_resources_mutex_);
  const auto owner = client_resources_.find(connection_id);
  if (owner == client_resources_.end() || !owner->second.ros_services.count(channel.topic)) {
    stats_.routing_errors++;
    return false;
  }
  std::lock_guard<std::mutex> routes_lock(service_response_mutex_);
  for (const auto & [id, route] : service_response_routes_) {
    if (route.connection_id == connection_id && route.correlation_id == corr_id) {
      stats_.routing_errors++;
      return false;
    }
  }
  do {
    ++next_ros_service_request_id_;
  } while (next_ros_service_request_id_ == 0 ||
    service_response_routes_.count(next_ros_service_request_id_));
  const uint32_t request_id = next_ros_service_request_id_;
  service_response_routes_.emplace(
    request_id, ServiceResponseRoute{connection_id, corr_id, channel.topic});
  if (!service_manager_->call_ros_service_async(request_id, channel.topic, request)) {
    service_response_routes_.erase(request_id);
    stats_.routing_errors++;
    return false;
  }
  return true;
}

void MessageRouter::set_log_protocol_messages(bool enabled)
{
  log_protocol_messages_ = enabled;
  topic_manager_->set_protocol_logging_enabled(enabled);
}

void MessageRouter::track_client_subscriber(int connection_id, const std::string & topic)
{
  std::lock_guard<std::mutex> lock(client_resources_mutex_);
  auto & state = client_resources_[connection_id];
  state.subscribed_topics.insert(topic);
  RCLCPP_INFO(get_logger(), "Client %d subscribed topics=%zu (last=%s)",
              connection_id, state.subscribed_topics.size(), topic.c_str());
}

void MessageRouter::track_client_publisher(int connection_id, const std::string & topic)
{
  std::lock_guard<std::mutex> lock(client_resources_mutex_);
  client_resources_[connection_id].published_topics.insert(topic);
}

void MessageRouter::track_client_ros_service(int connection_id, const std::string & service_name)
{
  std::lock_guard<std::mutex> lock(client_resources_mutex_);
  client_resources_[connection_id].ros_services.insert(service_name);
}

void MessageRouter::track_client_unity_service(int connection_id, const std::string & service_name)
{
  std::lock_guard<std::mutex> lock(client_resources_mutex_);
  client_resources_[connection_id].unity_services.insert(service_name);
}

void MessageRouter::track_client_webrtc_session(int connection_id, const std::string & session_id)
{
  std::lock_guard<std::mutex> lock(client_resources_mutex_);
  client_resources_[connection_id].webrtc_session_ids.insert(session_id);
}

void MessageRouter::untrack_client_webrtc_session(int connection_id, const std::string & session_id)
{
  std::lock_guard<std::mutex> lock(client_resources_mutex_);
  auto it = client_resources_.find(connection_id);
  if (it == client_resources_.end()) {
    return;
  }
  it->second.webrtc_session_ids.erase(session_id);
}

void MessageRouter::untrack_client_subscriber(int connection_id, const std::string & topic)
{
  std::lock_guard<std::mutex> lock(client_resources_mutex_);
  auto it = client_resources_.find(connection_id);
  if (it == client_resources_.end()) {
    return;
  }
  it->second.subscribed_topics.erase(topic);
  RCLCPP_INFO(get_logger(), "Client %d subscribed topics=%zu after remove (%s)",
              connection_id, it->second.subscribed_topics.size(), topic.c_str());
}

void MessageRouter::untrack_client_publisher(int connection_id, const std::string & topic)
{
  std::lock_guard<std::mutex> lock(client_resources_mutex_);
  auto it = client_resources_.find(connection_id);
  if (it == client_resources_.end()) {
    return;
  }
  it->second.published_topics.erase(topic);
}

void MessageRouter::untrack_client_ros_service(int connection_id, const std::string & service_name)
{
  std::lock_guard<std::mutex> lock(client_resources_mutex_);
  auto it = client_resources_.find(connection_id);
  if (it == client_resources_.end()) {
    return;
  }
  it->second.ros_services.erase(service_name);
}

void MessageRouter::untrack_client_unity_service(
  int connection_id,
  const std::string & service_name)
{
  std::lock_guard<std::mutex> lock(client_resources_mutex_);
  auto it = client_resources_.find(connection_id);
  if (it == client_resources_.end()) {
    return;
  }
  it->second.unity_services.erase(service_name);
}

void MessageRouter::on_client_disconnected(int connection_id)
{
#ifdef ENABLE_WEBRTC
  if (control_link_) {control_link_->disconnected(connection_id);}
#endif
  ClientResourceState resources;
  bool had_resources = false;
  size_t removed_ros_services = 0;
  {
    std::lock_guard<std::mutex> lock(client_resources_mutex_);
    auto it = client_resources_.find(connection_id);
    if (it != client_resources_.end()) {
      resources = it->second;
      client_resources_.erase(it);
      had_resources = true;
    }
    {
      std::lock_guard<std::mutex> routes_lock(service_response_mutex_);
      for (auto route = service_response_routes_.begin(); route != service_response_routes_.end(); ) {
        if (route->second.connection_id == connection_id) {
          service_manager_->cancel_ros_service_call(route->first);
          route = service_response_routes_.erase(route);
        } else {
          ++route;
        }
      }
    }
    for (const auto & service_name : resources.ros_services) {
      const bool shared = std::any_of(client_resources_.begin(), client_resources_.end(),
        [&](const auto & entry) {return entry.second.ros_services.count(service_name) != 0;});
      if (!shared && service_manager_->unregister_ros_service(service_name)) {
        ++removed_ros_services;
      }
    }
  }

  size_t removed_subscriber_topics =
    topic_manager_->unregister_all_client_subscribers(connection_id);
  size_t removed_publisher_owners = topic_manager_->unregister_all_client_publishers(connection_id);

  if (control_lease_manager_) {
    control_lease_manager_->on_client_disconnected(connection_id);
  }

  size_t removed_unity_services = 0;
  for (const auto & service_name : resources.unity_services) {
    if (service_manager_->unregister_unity_service(service_name)) {
      removed_unity_services++;
    }
  }



#ifdef ENABLE_WEBRTC
  for (const auto & session_id : resources.webrtc_session_ids) {
    remove_webrtc_session(session_id);
  }
#endif

  if (had_resources || removed_subscriber_topics > 0) {
    RCLCPP_INFO(
      get_logger(),
      "Client %d cleanup: subscribers=%zu publishers=%zu ros_services=%zu unity_services=%zu webrtc_sessions=%zu",
      connection_id,
      removed_subscriber_topics,
      removed_publisher_owners,
      removed_ros_services,
      removed_unity_services,
      resources.webrtc_session_ids.size());
  }
}

#ifdef ENABLE_WEBRTC
void MessageRouter::publish_webrtc_signal(const nlohmann::json & payload)
{
  if (!webrtc_signal_pub_) {
    return;
  }

  const std::string type = payload.value("type", "");
  const std::string session_id = payload.value("session_id", "");
  if (type == "answer" || type == "error" || type == "ready") {
    RCLCPP_INFO(
      get_logger(),
      "Publishing WebRTC signal: type='%s' session='%s'",
      type.c_str(),
      session_id.c_str());
  } else if (type == "candidate") {
    RCLCPP_DEBUG(
      get_logger(),
      "Publishing WebRTC candidate: session='%s' mid='%s'",
      session_id.c_str(),
      payload.value("sdpMid", "").c_str());
  }

  std_msgs::msg::String msg;
  msg.data = payload.dump();
  webrtc_signal_pub_->publish(msg);
}

void MessageRouter::handle_webrtc_signal_msg(const std_msgs::msg::String::SharedPtr msg)
{
  if (!webrtc_enabled_ || msg == nullptr || msg->data.empty()) {
    return;
  }

  std::string clean_json = msg->data;
  clean_json.erase(
    std::remove(clean_json.begin(), clean_json.end(), '\0'),
    clean_json.end());
  // Trim leading/trailing whitespace after sanitizing CDR null terminators.
  auto not_space = [](unsigned char ch) {return !std::isspace(ch);};
  clean_json.erase(clean_json.begin(),
      std::find_if(clean_json.begin(), clean_json.end(), not_space));
  clean_json.erase(std::find_if(clean_json.rbegin(), clean_json.rend(), not_space).base(),
      clean_json.end());
  if (clean_json.empty()) {
    return;
  }

  nlohmann::json payload;
  try {
    payload = nlohmann::json::parse(clean_json);
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "Failed to parse WebRTC signal JSON: %s", e.what());
    return;
  }

  std::string type = payload.value("type", "");
  std::transform(type.begin(), type.end(), type.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
  });

  if (type == "offer") {
    handle_webrtc_offer(payload);
  } else if (type == "candidate") {
    handle_webrtc_candidate(payload);
  } else if (type == "stop") {
    handle_webrtc_stop(payload);
  } else if (type == "keyframe" || type == "profile") {
    std::shared_ptr<WebRtcSession> session;
    {
      std::lock_guard<std::mutex> lock(webrtc_sessions_mutex_);
      const auto found = webrtc_sessions_.find(payload.value("session_id", ""));
      if (found != webrtc_sessions_.end()) {session = found->second;}
    }
    if (session && session->running.load() && session->manager) {
      if (type == "keyframe") {
        session->manager->request_keyframe();
      } else {
        const int bitrate = std::clamp(payload.value("bitrate_kbps", webrtc_default_bitrate_kbps_), 100, 100000);
        const int fps = std::clamp(payload.value("framerate", webrtc_default_framerate_), 1, 60);
        session->manager->update_profile(bitrate, fps);
        session->requested_bitrate.store(bitrate);
        session->requested_framerate.store(fps);
        session->media_priority.store(std::clamp(payload.value("priority", 1), 1, 16));
        session->last_encoder_rebind_ms.store(0);
        if (session->packet_relay && native_media_relay_) {
          native_media_relay_->update_profile(session->stream_topic, session->session_id, bitrate, fps,
            session->media_priority.load());
        }
      }
    }
  } else {
    RCLCPP_WARN(get_logger(), "Unsupported WebRTC signal type: %s", type.c_str());
  }
}

void MessageRouter::handle_webrtc_offer(const nlohmann::json & payload)
{
  const std::string session_id = payload.value("session_id", "");
  const std::string sdp = payload.value("sdp", "");
  const std::string stream_topic = payload.value("stream_topic", "");
  const std::string auxiliary_topic = payload.value("auxiliary_topic", "");
  std::string image_type = payload.value("image_type", "");
  std::transform(image_type.begin(), image_type.end(), image_type.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
  });

  if (session_id.empty() || sdp.empty() || stream_topic.empty()) {
    nlohmann::json err;
    err["session_id"] = session_id;
    err["type"] = "error";
    err["error"] = "Missing required fields for offer (session_id, sdp, stream_topic)";
    publish_webrtc_signal(err);
    return;
  }

  const auto offer_identity = payload.dump();
  std::shared_ptr<WebRtcSession> existing;
  {
    std::lock_guard<std::mutex> lock(webrtc_sessions_mutex_);
    auto found = webrtc_sessions_.find(session_id);
    if (found != webrtc_sessions_.end()) {
      existing = found->second;
    }
  }
  if (existing && existing->running.load() && existing->offer_identity == offer_identity) {
    nlohmann::json answer;
    {
      std::lock_guard<std::mutex> lock(existing->signaling_mutex);
      answer = existing->last_answer;
    }
    // DDS history/reconnect and client retries may repeat an offer. Reusing its
    // answer preserves ICE credentials; replacing the live peer breaks them.
    if (!answer.is_null()) {
      publish_webrtc_signal(answer);
    }
    return;
  }
  remove_webrtc_session(session_id);
  {
    std::lock_guard<std::mutex> lock(webrtc_sessions_mutex_);
    if (webrtc_sessions_.size() >= max_webrtc_sessions_) {
      publish_webrtc_signal({{"session_id", session_id}, {"type", "error"},
        {"error", "Native media session capacity reached"}});
      return;
    }
  }
  RCLCPP_INFO(
    get_logger(),
    "Received WebRTC offer: session='%s' topic='%s' image_type='%s'",
    session_id.c_str(),
    stream_topic.c_str(),
    image_type.c_str());

  auto session = std::make_shared<WebRtcSession>();
  session->session_id = session_id;
  session->offer_identity = offer_identity;
  if (payload.contains("client_fd")) {
    try {
      session->owner_client_fd = payload.at("client_fd").is_number_integer() ?
        payload.at("client_fd").get<int>() :
        std::stoi(payload.at("client_fd").get<std::string>());
    } catch (...) {
      session->owner_client_fd = -1;
    }
  }
  session->stream_topic = stream_topic;
  session->auxiliary_topic = auxiliary_topic;
  session->image_type = image_type;
  session->packet_relay = native_media_relay_ && native_media_relay_->contains(stream_topic);
  session->preencoded_h264 = session->packet_relay || image_type == "h264" ||
    std::find(webrtc_preencoded_topics_.begin(), webrtc_preencoded_topics_.end(),
    stream_topic) != webrtc_preencoded_topics_.end();
  session->session_start_time = std::chrono::steady_clock::now();
  session->last_peer_ready_ms.store(std::chrono::duration_cast<std::chrono::milliseconds>(
      session->session_start_time.time_since_epoch()).count());
  session->last_incoming_frame_time = session->session_start_time;
  session->last_push_time = session->session_start_time;
  session->last_encoded_progress_time = session->session_start_time;
  session->last_keyframe_request_time = session->session_start_time;
  session->last_telemetry_log_time = session->session_start_time;
  session->manager = std::make_shared<WebRTCManager>();

  WebRTCManager::Settings settings;
  settings.stun_servers = webrtc_default_stun_servers_;
  settings.encoder = webrtc_default_encoder_;
  settings.bitrate_kbps = webrtc_default_bitrate_kbps_;
  settings.framerate = webrtc_default_framerate_;
  settings.pipeline = webrtc_default_pipeline_;
  settings.wait_for_complete_gathering = true;
  settings.preencoded_h264 = session->preencoded_h264;
  settings.packet_relay = session->packet_relay;
  if (session->packet_relay) {
    settings.request_preencoded_keyframe = [this, stream_topic]() {
      native_media_relay_->request_keyframe(stream_topic);
    };
    settings.bandwidth_estimate = [this, stream_topic, session_id](int bitrate) {
      native_media_relay_->update_capacity(stream_topic, session_id, bitrate);
    };
  } else if (session->preencoded_h264) {
    // A pass-through sender cannot create an IDR itself. Forward receiver PLI
    // requests to the encoder instead of silently ignoring packet-loss recovery.
    auto keyframe_publisher = create_publisher<std_msgs::msg::String>(
      stream_topic + "/request_keyframe", rclcpp::QoS(1));
    settings.request_preencoded_keyframe = [keyframe_publisher]() {
        std_msgs::msg::String request;
        request.data = "idr";
        keyframe_publisher->publish(request);
      };
  }

  auto get_int = [&payload](const char * key, int fallback) {
      if (!payload.contains(key)) {
        return fallback;
      }
      try {
        return payload.at(key).is_number_integer() ?
               payload.at(key).get<int>() :
               std::stoi(payload.at(key).get<std::string>());
      } catch (...) {
        return fallback;
      }
    };

  if (payload.contains("encoder") && payload.at("encoder").is_string()) {
    settings.encoder = payload.at("encoder").get<std::string>();
  }
  if (payload.contains("wait_for_complete_gathering")) {
    try {
      settings.wait_for_complete_gathering =
        payload.at("wait_for_complete_gathering").is_boolean() ?
        payload.at("wait_for_complete_gathering").get<bool>() :
        (payload.at("wait_for_complete_gathering").get<std::string>() != "false");
    } catch (...) {
      settings.wait_for_complete_gathering = true;
    }
  }
  if (payload.contains("stun_servers")) {
    try {
      settings.stun_servers.clear();
      if (payload.at("stun_servers").is_array()) {
        for (const auto & entry : payload.at("stun_servers")) {
          if (entry.is_string()) {
            const auto value = entry.get<std::string>();
            if (!value.empty()) {
              settings.stun_servers.push_back(value);
            }
          }
        }
      } else if (payload.at("stun_servers").is_string()) {
        const auto value = payload.at("stun_servers").get<std::string>();
        if (!value.empty()) {
          settings.stun_servers.push_back(value);
        }
      }
    } catch (...) {
      settings.stun_servers = webrtc_default_stun_servers_;
    }
  }
  settings.bitrate_kbps = std::clamp(
    get_int("bitrate_kbps", settings.bitrate_kbps), 100, 100000);
  settings.framerate = std::clamp(
    get_int("framerate", settings.framerate), 1, 60);
  session->media_priority.store(std::clamp(get_int("priority", 1), 1, 16));
  if (media_link_budget_kbps_) {
    size_t count;
    {std::lock_guard<std::mutex> lock(webrtc_sessions_mutex_); count = webrtc_sessions_.size() + 1;}
    const auto allowance = media_link_budget_kbps_ - media_reserved_kbps_ - media_bulk_floor_kbps_;
    if (int64_t(count) * 100 > allowance) {
      publish_webrtc_signal({{"session_id", session_id}, {"type", "error"},
        {"error", "Shared link budget cannot admit another camera at the minimum bitrate"}});
      return;
    }
    session->budget_bitrate.store(std::min(settings.bitrate_kbps, int(allowance / count)));
  }
  const bool shared_encoding = share_encoders_ && !session->preencoded_h264 && auxiliary_topic.empty();
  session->requested_bitrate.store(settings.bitrate_kbps);
  session->requested_framerate.store(settings.framerate);
  if (shared_encoding) {
    session->source_settings = settings;
    session->requested_bitrate.store(settings.bitrate_kbps);
    session->requested_framerate.store(settings.framerate);
    settings.preencoded_h264 = true;
    const auto weak = std::weak_ptr<WebRtcSession>(session);
    settings.request_preencoded_keyframe = [weak]() {
      if (auto peer = weak.lock()) {
        if (auto group = std::atomic_load(&peer->encoder_group)) {group->worker->manager->request_keyframe();}
      }
    };
    settings.bandwidth_estimate = [weak](int bitrate) {
      if (auto peer = weak.lock()) {peer->capacity_bitrate.store(bitrate);}
    };
  }

  RCLCPP_INFO(
    get_logger(),
    "WebRTC session '%s' config: encoder=%s bitrate=%dkbps fps=%d stun_servers=%zu wait_for_gathering=%s",
    session_id.c_str(),
    settings.encoder.c_str(),
    settings.bitrate_kbps,
    settings.framerate,
    settings.stun_servers.size(),
    settings.wait_for_complete_gathering ? "true" : "false");

  if (!session->manager->initialize(settings)) {
    nlohmann::json err;
    err["session_id"] = session_id;
    err["type"] = "error";
    err["error"] = "Bridge failed to initialize WebRTC session";
    publish_webrtc_signal(err);
    return;
  }

  session->manager->set_signaling_callback(
    [this, session_id, weak_session = std::weak_ptr<WebRtcSession>(session)](
      const std::string & type, const nlohmann::json & data) {
      const auto current = weak_session.lock();
      if (!current || !current->running.load()) {
        return;
      }
      nlohmann::json out;
      out["session_id"] = session_id;
      out["type"] = type;
      if (data.is_object()) {
        for (auto it = data.begin(); it != data.end(); ++it) {
          out[it.key()] = it.value();
        }
      }
      if (type == "answer") {
        std::lock_guard<std::mutex> lock(current->signaling_mutex);
        current->last_answer = out;
      }
      publish_webrtc_signal(out);
    });

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
    .best_effort()
    .durability_volatile();
  if (session->preencoded_h264) {
    qos = rclcpp::QoS(rclcpp::KeepLast(32)).reliable().durability_volatile();
  }
  session->running = true;
  const bool compressed =
    image_type == "compressed" ||
    image_type == "h264" ||
    stream_topic.find("/compressed") != std::string::npos;
  if (shared_encoding) {
    if (!attach_shared_encoder(session)) {
      session->running.store(false);
      publish_webrtc_signal({{"session_id", session_id}, {"type", "error"},
          {"error", "Source encoder capacity unavailable"}});
      return;
    }
  } else if (session->packet_relay) {
    std::weak_ptr<WebRtcSession> weak = session;
    native_media_relay_->subscribe(stream_topic, session_id,
      [weak](const uint8_t * data, size_t size, uint64_t generation) {
        if (const auto session = weak.lock(); session && session->running.load()) {
          session->manager->push_rtp(data, size, generation);
        }
      });
    native_media_relay_->update_profile(stream_topic, session_id, settings.bitrate_kbps, settings.framerate,
      session->media_priority.load());
  } else if (compressed) {
    session->compressed_sub = create_subscription<sensor_msgs::msg::CompressedImage>(
      stream_topic,
      qos,
      [this, session](const sensor_msgs::msg::CompressedImage::SharedPtr image_msg) {
        enqueue_webrtc_compressed_frame(session, image_msg);
      });
  } else {
    session->raw_sub = create_subscription<sensor_msgs::msg::Image>(
      stream_topic,
      qos,
      [this, session](const sensor_msgs::msg::Image::SharedPtr image_msg) {
        enqueue_webrtc_raw_frame(session, image_msg);
      });
  }
  if (!auxiliary_topic.empty()) {
    session->auxiliary_sub =
      create_subscription<sensor_msgs::msg::CompressedImage>(
      auxiliary_topic,
      qos,
      [session](const sensor_msgs::msg::CompressedImage::SharedPtr auxiliary_msg) {
        if (!session->running.load() || !auxiliary_msg) {
          return;
        }
        session->incoming_auxiliary_frames.fetch_add(1);
        if (session->manager &&
        session->manager->send_auxiliary_data(auxiliary_msg->data))
        {
          session->sent_auxiliary_frames.fetch_add(1);
        } else {
          session->dropped_auxiliary_frames.fetch_add(1);
        }
      });
  }

  if (!session->packet_relay && !shared_encoding) {
    session->worker_thread = std::thread(&MessageRouter::process_webrtc_frames, this, session);
  }

  {
    std::lock_guard<std::mutex> lock(webrtc_sessions_mutex_);
    webrtc_sessions_[session_id] = session;
  }
  if (session->owner_client_fd >= 0) {
    track_client_webrtc_session(session->owner_client_fd, session_id);
  }

  if (!session->manager->handle_offer(sdp)) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to negotiate WebRTC offer for session '%s' topic '%s'",
      session_id.c_str(),
      stream_topic.c_str());
    nlohmann::json err;
    err["session_id"] = session_id;
    err["type"] = "error";
    err["error"] = "Bridge failed to negotiate WebRTC offer";
    publish_webrtc_signal(err);
    remove_webrtc_session(session_id);
    return;
  }

  nlohmann::json ready;
  ready["session_id"] = session_id;
  ready["type"] = "ready";
  ready["profile_updates"] = true;
  publish_webrtc_signal(ready);
  RCLCPP_INFO(
    get_logger(),
    "Started WebRTC session '%s' for topic '%s'",
    session_id.c_str(),
    stream_topic.c_str());
}

void MessageRouter::handle_webrtc_candidate(const nlohmann::json & payload)
{
  const std::string session_id = payload.value("session_id", "");
  if (session_id.empty()) {
    return;
  }

  std::shared_ptr<WebRtcSession> session;
  {
    std::lock_guard<std::mutex> lock(webrtc_sessions_mutex_);
    auto it = webrtc_sessions_.find(session_id);
    if (it != webrtc_sessions_.end()) {
      session = it->second;
    }
  }

  if (!session || !session->manager) {
    nlohmann::json err;
    err["session_id"] = session_id;
    err["type"] = "error";
    err["error"] = "Unknown WebRTC session";
    publish_webrtc_signal(err);
    return;
  }

  const std::string candidate = payload.value("candidate", "");
  std::string sdp_mid = payload.value("sdpMid", "");
  int sdp_mline_index = 0;
  if (payload.contains("sdpMLineIndex")) {
    try {
      sdp_mline_index = payload.at("sdpMLineIndex").is_number_integer() ?
        payload.at("sdpMLineIndex").get<int>() :
        std::stoi(payload.at("sdpMLineIndex").get<std::string>());
    } catch (...) {
      sdp_mline_index = 0;
    }
  }

  if (candidate.empty()) {
    return;
  }
  std::string candidate_type = "unknown";
  const std::string type_marker = " typ ";
  const auto type_pos = candidate.find(type_marker);
  if (type_pos != std::string::npos) {
    const auto start = type_pos + type_marker.size();
    auto end = candidate.find(' ', start);
    if (end == std::string::npos) {
      end = candidate.size();
    }
    if (end > start) {
      candidate_type = candidate.substr(start, end - start);
    }
  }
  if (sdp_mid.empty()) {
    // Unity occasionally omits sdpMid while still sending sdpMLineIndex.
    sdp_mid = "0";
  }
  RCLCPP_INFO(
    get_logger(),
    "Accepted WebRTC candidate for session '%s' (mid='%s', mline=%d, type=%s)",
    session_id.c_str(),
    sdp_mid.c_str(),
    sdp_mline_index,
    candidate_type.c_str());
  session->manager->handle_candidate(candidate, sdp_mid, sdp_mline_index);
}

void MessageRouter::handle_webrtc_stop(const nlohmann::json & payload)
{
  const std::string session_id = payload.value("session_id", "");
  if (session_id.empty()) {
    return;
  }
  remove_webrtc_session(session_id);
}

void MessageRouter::enqueue_webrtc_raw_frame(
  const std::shared_ptr<WebRtcSession> & session,
  const sensor_msgs::msg::Image::SharedPtr & msg)
{
  if (!session || !session->running || !msg) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(session->frame_queue_mutex);
    if (session->frame_queue.size() >= webrtc_max_queue_size_) {
      session->frame_queue.pop_front();
      session->dropped_queue_frames++;
    }
    session->frame_queue.push_back(msg);
    session->incoming_frames++;
    session->last_incoming_frame_time = std::chrono::steady_clock::now();
  }
  session->frame_queue_cv.notify_one();
}

void MessageRouter::enqueue_webrtc_compressed_frame(
  const std::shared_ptr<WebRtcSession> & session,
  const sensor_msgs::msg::CompressedImage::SharedPtr & msg)
{
  if (!session || !session->running || !msg) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(session->frame_queue_mutex);
    const size_t capacity = session->preencoded_h264 ? 8U : webrtc_max_queue_size_;
    if (session->frame_queue.size() >= capacity) {
      if (session->preencoded_h264) {
        session->dropped_queue_frames += session->frame_queue.size();
        session->frame_queue.clear();
        session->h264_queue_discontinuity.store(true);
      } else {
        session->frame_queue.pop_front();
        session->dropped_queue_frames++;
      }
    }
    session->frame_queue.push_back(msg);
    session->incoming_frames++;
    session->last_incoming_frame_time = std::chrono::steady_clock::now();
  }
  session->frame_queue_cv.notify_one();
}

void MessageRouter::process_webrtc_frames(const std::shared_ptr<WebRtcSession> & session)
{
  remote_render::H264Continuity h264_continuity;
  uint64_t source_frame_index = 0;
  const auto camera_metadata = [&](const auto & header) {
      return nlohmann::json({{"kind", "horus_camera_frame_v1"},
        {"frame_id", ++source_frame_index},
        {"stream_epoch_ns", std::chrono::duration_cast<std::chrono::nanoseconds>(
            session->session_start_time.time_since_epoch()).count()},
        {"ros_stamp_ns", int64_t(header.stamp.sec) * 1000000000LL + header.stamp.nanosec},
        {"ros_frame", header.frame_id.substr(0, 256)}, {"stamp_clock", "ros_unsynchronized"},
        {"adapter_monotonic_ns", std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count()}}).dump();
    };
  const auto kWarmupWindow = std::chrono::milliseconds(2000);
  const auto kIncomingActivityWindow = std::chrono::milliseconds(2500);
  const auto kEncodedStallThreshold = std::chrono::milliseconds(2500);
  const auto kKeyframeCooldown = std::chrono::milliseconds(3000);
  const auto kTelemetryLogInterval = std::chrono::seconds(5);

  while (session && session->running) {
    FrameVariant frame_variant;
    {
      std::unique_lock<std::mutex> lock(session->frame_queue_mutex);
      session->frame_queue_cv.wait_for(lock, std::chrono::milliseconds(200), [&session] {
          return !session->frame_queue.empty() || !session->running;
      });
      if (!session->running) {
        break;
      }
      if (session->frame_queue.empty()) {
        continue;
      }

      if (session->preencoded_h264) {
        if (session->h264_queue_discontinuity.exchange(false)) {
          h264_continuity.reset();
          session->manager->request_keyframe();
        }
        frame_variant = session->frame_queue.front();
        session->frame_queue.pop_front();
      } else {
        frame_variant = session->frame_queue.back();
        session->frame_queue.clear();
      }
    }

    std::vector<uint8_t> rgb_data;
    bool pushed_frame = false;
    if (std::holds_alternative<sensor_msgs::msg::Image::SharedPtr>(frame_variant)) {
      auto raw = std::get<sensor_msgs::msg::Image::SharedPtr>(frame_variant);
      if (!raw) {
        continue;
      }
      const bool packed_rgb = raw->encoding == sensor_msgs::image_encodings::RGB8 &&
        raw->width > 0 && raw->height > 0 && uint64_t(raw->width) * 3 == raw->step &&
        uint64_t(raw->step) * raw->height <= raw->data.size();
      if (packed_rgb || convert_to_rgb(*raw, rgb_data, session->waned_unsupported_encoding)) {
        const std::string metadata_prefix = "horus.remote_render.v3:";
        const std::string metadata = raw->header.frame_id.rfind(metadata_prefix, 0) == 0 ?
          raw->header.frame_id.substr(metadata_prefix.size()) : camera_metadata(raw->header);
        pushed_frame = session->manager->push_frame(packed_rgb ? raw->data : rgb_data, static_cast<int>(raw->width),
            static_cast<int>(raw->height), "RGB", metadata);
      }
    } else {
      auto compressed = std::get<sensor_msgs::msg::CompressedImage::SharedPtr>(frame_variant);
      if (!compressed) {
        continue;
      }
      const std::string metadata_prefix = "horus.remote_render.v3:";
      const std::string metadata = compressed->header.frame_id.rfind(
        metadata_prefix, 0) == 0 ?
        compressed->header.frame_id.substr(metadata_prefix.size()) : camera_metadata(compressed->header);
      if (remote_render::is_h264_compressed_format(compressed->format)) {
        int64_t frame_index = -1;
        if (!metadata.empty()) {
          const auto parsed = nlohmann::json::parse(metadata, nullptr, false);
          if (parsed.is_object() && parsed.contains("encoder_frame_index") &&
            parsed["encoder_frame_index"].is_number_integer())
          {
            frame_index = parsed["encoder_frame_index"].get<int64_t>();
          }
        }
        if (h264_continuity.accept(compressed->data, frame_index)) {
          pushed_frame = session->manager->push_encoded_h264(compressed->data, metadata);
          if (!pushed_frame) {
            h264_continuity.reset();
          }
        }
      } else {
        int width = 0;
        int height = 0;
        if (decompress_to_rgb(*compressed, rgb_data, width, height,
            session->waned_unsupported_encoding))
        {
          pushed_frame = session->manager->push_frame(rgb_data, width, height, "RGB", metadata);
        }
      }
    }

    const auto now = std::chrono::steady_clock::now();
    bool should_request_keyframe = false;
    bool stalled = false;
    bool should_log_telemetry = false;
    uint64_t incoming_frames = 0;
    uint64_t pushed_frames = 0;
    uint64_t dropped_frames = 0;
    uint64_t encoded_units = 0;
    uint64_t encoded_bytes = 0;
    uint64_t metadata_inputs = 0;
    uint64_t metadata_matches = 0;
    uint64_t metadata_fifo_fallbacks = 0;
    uint64_t metadata_unmatched = 0;
    uint64_t metadata_selfcheck_failures = 0;
    bool peer_connected = false;
    bool track_open = false;

    {
      std::lock_guard<std::mutex> lock(session->frame_queue_mutex);
      if (pushed_frame) {
        session->pushed_frames++;
        session->last_push_time = now;
      }

      if (session->manager) {
        encoded_units = session->manager->get_encoded_units_sent();
        encoded_bytes = session->manager->get_encoded_bytes_sent();
        metadata_inputs = session->manager->get_metadata_inputs_received();
        metadata_matches = session->manager->get_metadata_pts_matches();
        metadata_fifo_fallbacks =
          session->manager->get_metadata_fifo_fallbacks();
        metadata_unmatched = session->manager->get_metadata_unmatched();
        metadata_selfcheck_failures =
          session->manager->get_metadata_selfcheck_failures();
        peer_connected = session->manager->is_peer_connected();
        track_open = session->manager->is_video_track_open();
      }

      if (encoded_units > session->last_seen_encoded_units) {
        session->last_seen_encoded_units = encoded_units;
        session->last_encoded_progress_time = now;
      }

      const bool warmupElapsed =
        session->session_start_time.time_since_epoch().count() > 0 &&
        (now - session->session_start_time) > kWarmupWindow;
      const bool has_recent_incoming =
        session->last_incoming_frame_time.time_since_epoch().count() > 0 &&
        (now - session->last_incoming_frame_time) < kIncomingActivityWindow;
      const bool encoded_output_stalled =
        warmupElapsed &&
        peer_connected &&
        track_open &&
        has_recent_incoming &&
        session->last_encoded_progress_time.time_since_epoch().count() > 0 &&
        (now - session->last_encoded_progress_time) > kEncodedStallThreshold;
      stalled = encoded_output_stalled;
      const bool periodic_refresh = !session->preencoded_h264 && peer_connected && track_open &&
        now - session->last_keyframe_request_time > std::chrono::seconds(5);
      if ((encoded_output_stalled || periodic_refresh) &&
        (now - session->last_keyframe_request_time) > kKeyframeCooldown)
      {
        session->last_keyframe_request_time = now;
        should_request_keyframe = true;
      }

      if ((now - session->last_telemetry_log_time) > kTelemetryLogInterval) {
        session->last_telemetry_log_time = now;
        should_log_telemetry = true;
      }

      incoming_frames = session->incoming_frames;
      pushed_frames = session->pushed_frames;
      dropped_frames = session->dropped_queue_frames;
    }

    if (should_request_keyframe && session->manager) {
      session->manager->request_keyframe();
      if (stalled) {RCLCPP_WARN(
        get_logger(),
        "WebRTC session '%s' stall detected (incoming=%" PRIu64
        " pushed=%" PRIu64 " dropped=%" PRIu64 " encoded_units=%" PRIu64
        "). Requested keyframe (cooldown=%" PRId64
        "ms, stall_threshold=%" PRId64 "ms).",
        session->session_id.c_str(),
        incoming_frames,
        pushed_frames,
        dropped_frames,
        encoded_units,
        static_cast<int64_t>(kKeyframeCooldown.count()),
        static_cast<int64_t>(kEncodedStallThreshold.count()));}
    }

    if (should_log_telemetry) {
      RCLCPP_INFO(
        get_logger(),
        "WebRTC session '%s' telemetry: incoming=%" PRIu64
        " pushed=%" PRIu64 " dropped=%" PRIu64 " encoded_units=%" PRIu64
        " encoded_bytes=%" PRIu64 " metadata_inputs=%" PRIu64
        " metadata_exact_pts=%" PRIu64 " metadata_fifo=%" PRIu64
        " metadata_unmatched=%" PRIu64
        " metadata_selfcheck_failures=%" PRIu64 " peer=%s track=%s",
        session->session_id.c_str(),
        incoming_frames,
        pushed_frames,
        dropped_frames,
        encoded_units,
        encoded_bytes,
        metadata_inputs,
        metadata_matches,
        metadata_fifo_fallbacks,
        metadata_unmatched,
        metadata_selfcheck_failures,
        peer_connected ? "connected" : "disconnected",
        track_open ? "open" : "closed");
      RCLCPP_INFO(get_logger(), "WebRTC transport '%s': %s", session->session_id.c_str(),
        session->manager->get_transport_statistics().dump().c_str());
    }

  }
}

bool MessageRouter::convert_to_rgb(
  const sensor_msgs::msg::Image & msg,
  std::vector<uint8_t> & output,
  bool & waned_flag)
{
  const auto width = static_cast<size_t>(msg.width);
  const auto height = static_cast<size_t>(msg.height);
  const auto row_stride = static_cast<size_t>(msg.step);
  if (width == 0 || height == 0 || row_stride == 0) {
    return false;
  }
  const size_t channels = msg.encoding == "mono8" ? 1 : 3;
  if (row_stride < width * channels || height > msg.data.size() / row_stride) {return false;}

  auto equals_ignore_case = [](const std::string & lhs, const std::string & rhs) {
      if (lhs.size() != rhs.size()) {
        return false;
      }
      for (size_t i = 0; i < lhs.size(); ++i) {
        if (std::tolower(static_cast<unsigned char>(lhs[i])) !=
          std::tolower(static_cast<unsigned char>(rhs[i])))
        {
          return false;
        }
      }
      return true;
    };

  if (equals_ignore_case(msg.encoding, sensor_msgs::image_encodings::RGB8)) {
    output.resize(width * height * 3);
    for (size_t y = 0; y < height; ++y) {
      const uint8_t * row = msg.data.data() + (y * row_stride);
      std::memcpy(output.data() + (y * width * 3), row, width * 3);
    }
    return true;
  }

  if (equals_ignore_case(msg.encoding, sensor_msgs::image_encodings::BGR8)) {
    output.resize(width * height * 3);
    for (size_t y = 0; y < height; ++y) {
      const uint8_t * row = msg.data.data() + (y * row_stride);
      for (size_t x = 0; x < width; ++x) {
        const uint8_t * pixel = row + (x * 3);
        size_t dst_index = (y * width + x) * 3;
        output[dst_index] = pixel[2];
        output[dst_index + 1] = pixel[1];
        output[dst_index + 2] = pixel[0];
      }
    }
    return true;
  }

  if (equals_ignore_case(msg.encoding, sensor_msgs::image_encodings::MONO8)) {
    output.resize(width * height * 3);
    for (size_t y = 0; y < height; ++y) {
      const uint8_t * row = msg.data.data() + (y * row_stride);
      for (size_t x = 0; x < width; ++x) {
        const uint8_t value = row[x];
        const size_t dst_index = (y * width + x) * 3;
        output[dst_index] = value;
        output[dst_index + 1] = value;
        output[dst_index + 2] = value;
      }
    }
    return true;
  }

  if (!waned_flag) {
    RCLCPP_WARN(get_logger(), "Unsupported image encoding for WebRTC stream: %s",
        msg.encoding.c_str());
    waned_flag = true;
  }
  return false;
}

bool MessageRouter::decompress_to_rgb(
  const sensor_msgs::msg::CompressedImage & msg,
  std::vector<uint8_t> & output,
  int & width,
  int & height,
  bool & waned_flag)
{
  try {
    cv::Mat encoded(1, static_cast<int>(msg.data.size()), CV_8UC1,
      const_cast<uint8_t *>(msg.data.data()));
    cv::Mat decoded = cv::imdecode(encoded, cv::IMREAD_COLOR);
    if (decoded.empty()) {
      if (!waned_flag) {
        RCLCPP_WARN(get_logger(), "Failed to decode compressed image for WebRTC session");
        waned_flag = true;
      }
      return false;
    }

    cv::Mat rgb;
    cv::cvtColor(decoded, rgb, cv::COLOR_BGR2RGB);
    width = rgb.cols;
    height = rgb.rows;
    output.assign(rgb.data, rgb.data + (rgb.total() * rgb.elemSize()));
    return true;
  } catch (const cv::Exception & e) {
    if (!waned_flag) {
      RCLCPP_ERROR(get_logger(), "OpenCV decode exception in WebRTC session: %s", e.what());
      waned_flag = true;
    }
    return false;
  }
}

void MessageRouter::remove_webrtc_session(const std::string & session_id)
{
  std::shared_ptr<WebRtcSession> session;
  {
    std::lock_guard<std::mutex> lock(webrtc_sessions_mutex_);
    auto it = webrtc_sessions_.find(session_id);
    if (it == webrtc_sessions_.end()) {
      return;
    }
    session = it->second;
    webrtc_sessions_.erase(it);
  }

  if (session) {
    if (session->owner_client_fd >= 0) {
      untrack_client_webrtc_session(session->owner_client_fd, session_id);
    }
    uint64_t incoming_frames = 0;
    uint64_t pushed_frames = 0;
    uint64_t dropped_frames = 0;
    {
      std::lock_guard<std::mutex> lock(session->frame_queue_mutex);
      incoming_frames = session->incoming_frames;
      pushed_frames = session->pushed_frames;
      dropped_frames = session->dropped_queue_frames;
    }
    uint64_t encoded_units =
      session->manager ? session->manager->get_encoded_units_sent() : 0;
    uint64_t encoded_bytes =
      session->manager ? session->manager->get_encoded_bytes_sent() : 0;
    const uint64_t incoming_auxiliary = session->incoming_auxiliary_frames.load();
    const uint64_t sent_auxiliary = session->sent_auxiliary_frames.load();
    const uint64_t dropped_auxiliary = session->dropped_auxiliary_frames.load();
    RCLCPP_INFO(
      get_logger(),
      "Stopping WebRTC session '%s' topic='%s' telemetry: incoming=%" PRIu64
      " pushed=%" PRIu64 " dropped=%" PRIu64 " encoded_units=%" PRIu64
      " encoded_bytes=%" PRIu64 " auxiliary_in=%" PRIu64
      " auxiliary_sent=%" PRIu64 " auxiliary_dropped=%" PRIu64,
      session->session_id.c_str(),
      session->stream_topic.c_str(),
      incoming_frames,
      pushed_frames,
      dropped_frames,
      encoded_units,
      encoded_bytes,
      incoming_auxiliary,
      sent_auxiliary,
      dropped_auxiliary);

    session->running = false;
    release_shared_encoder(session);
    if (session->packet_relay && native_media_relay_) {
      native_media_relay_->unsubscribe(session->stream_topic, session_id);
    }
    session->auxiliary_sub.reset();
    session->frame_queue_cv.notify_all();
    if (session->worker_thread.joinable()) {
      session->worker_thread.join();
    }
    session->raw_sub.reset();
    session->compressed_sub.reset();
  }
}

void MessageRouter::shutdown_webrtc_sessions()
{
  std::vector<std::string> session_ids;
  {
    std::lock_guard<std::mutex> lock(webrtc_sessions_mutex_);
    session_ids.reserve(webrtc_sessions_.size());
    for (const auto & item : webrtc_sessions_) {
      session_ids.push_back(item.first);
    }
  }
  for (const auto & session_id : session_ids) {
    remove_webrtc_session(session_id);
  }
}

void MessageRouter::cleanup_stale_webrtc_sessions()
{
  if (!webrtc_enabled_ || webrtc_session_timeout_sec_ <= 0) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  std::vector<std::string> stale_ids;
  {
    std::lock_guard<std::mutex> lock(webrtc_sessions_mutex_);
    for (const auto & item : webrtc_sessions_) {
      const auto & session = item.second;
      if (!session) {
        stale_ids.push_back(item.first);
        continue;
      }
      const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count();
      if (session->manager && session->manager->is_peer_connected() &&
        session->manager->is_video_track_open())
      {
        session->last_peer_ready_ms.store(now_ms);
      }
      if (now_ms - session->last_peer_ready_ms.load() >
        static_cast<int64_t>(webrtc_session_timeout_sec_) * 1000)
      {
        stale_ids.push_back(item.first);
      }
    }
  }

  for (const auto & session_id : stale_ids) {
    RCLCPP_INFO(get_logger(), "Cleaning stale WebRTC session '%s'", session_id.c_str());
    remove_webrtc_session(session_id);
  }
}
#endif

} // namespace horus_unity_bridge
