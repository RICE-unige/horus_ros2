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
#include <cinttypes>

#include <sstream>
#include <nlohmann/json.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <rclcpp/qos.hpp>
#include <algorithm>
#include <cstring>

#ifdef ENABLE_WEBRTC
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#endif

namespace horus_unity_bridge
{

MessageRouter::MessageRouter(const rclcpp::NodeOptions & options)
: Node("horus_unity_bridge", options)
{
  topic_manager_ = std::make_unique<TopicManager>(this);
  service_manager_ = std::make_unique<ServiceManager>(this);
  control_lease_manager_ = std::make_unique<ControlLeaseManager>(this);

#ifdef ENABLE_WEBRTC
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
  webrtc_default_stun_servers_ = this->declare_parameter<std::vector<std::string>>(
    "webrtc.default_stun_servers",
    std::vector<std::string>{"stun:stun.l.google.com:19302"});
  webrtc_session_timeout_sec_ = this->declare_parameter<int>(
    "webrtc.session_timeout_sec", 20);

  if (webrtc_enabled_) {
    auto webrtc_signal_pub_qos = rclcpp::QoS(rclcpp::KeepLast(100))
      .reliability(rclcpp::ReliabilityPolicy::Reliable)
      .durability(rclcpp::DurabilityPolicy::Volatile);
    auto webrtc_signal_sub_qos = rclcpp::QoS(rclcpp::KeepLast(100))
      .reliability(rclcpp::ReliabilityPolicy::Reliable)
      .durability(rclcpp::DurabilityPolicy::Volatile);
    webrtc_signal_pub_ = this->create_publisher<std_msgs::msg::String>(
      webrtc_server_signal_topic_, webrtc_signal_pub_qos);
    webrtc_signal_sub_ = this->create_subscription<std_msgs::msg::String>(
      webrtc_client_signal_topic_,
      webrtc_signal_sub_qos,
      std::bind(&MessageRouter::handle_webrtc_signal_msg, this, std::placeholders::_1));
    webrtc_cleanup_timer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&MessageRouter::cleanup_stale_webrtc_sessions, this));
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

  // Setup service manager callback for sending responses to Unity
  service_manager_->set_response_callback(
    [this](uint32_t srv_id, const std::vector<uint8_t> & response) {
      // Send service response back to the requesting Unity client
      int target_fd = -1;
      std::string service_name;
      {
        std::lock_guard<std::mutex> lock(service_response_mutex_);
        auto it = service_response_client_.find(srv_id);
        if (it != service_response_client_.end()) {
          target_fd = it->second;
          service_response_client_.erase(it);
        }
        auto service_it = service_response_topic_.find(srv_id);
        if (service_it != service_response_topic_.end()) {
          service_name = service_it->second;
          service_response_topic_.erase(service_it);
        }
      }

      if (!service_name.empty() && horuslink_service_response_callback_ && target_fd != -1) {
        const auto payload = detail::strip_cdr_header(response);
        horuslink_service_response_callback_(target_fd, service_name, srv_id, payload);
        return;
      }

      stats_.routing_errors++;
      RCLCPP_WARN(
        get_logger(),
        "Dropping ROS service response id=%u because no HorusLink response channel is registered",
        srv_id);
    }
  );

  stats_ = Statistics{};

  RCLCPP_INFO(get_logger(), "Message router initialized");
}

MessageRouter::~MessageRouter()
{
#ifdef ENABLE_WEBRTC
  shutdown_webrtc_sessions();
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
  int client_fd,
  const horuslink::ChannelDescriptor & channel)
{
  if (channel.topic.empty() || channel.type_name.empty()) {
    return false;
  }

  const bool success = topic_manager_->register_horuslink_subscriber(
    channel.topic,
    channel.type_name,
    client_fd,
    [this, client_fd](
      const std::string & topic,
      const std::vector<uint8_t> & data)
    {
      if (send_callback_) {
        send_callback_(client_fd, topic, data);
      }
    });

  if (success) {
    track_client_subscriber(client_fd, channel.topic);
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

bool MessageRouter::register_horuslink_publisher(
  int client_fd,
  const horuslink::ChannelDescriptor & channel)
{
  if (channel.topic.empty() || channel.type_name.empty()) {
    return false;
  }

  const bool success = topic_manager_->register_publisher(
    channel.topic,
    channel.type_name,
    client_fd);

  if (success) {
    track_client_publisher(client_fd, channel.topic);
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
  int client_fd,
  const horuslink::ChannelDescriptor & channel)
{
  if (channel.topic.empty() || channel.type_name.empty()) {
    return false;
  }

  const bool success = service_manager_->register_ros_service(
    channel.topic,
    channel.type_name);

  if (success) {
    track_client_ros_service(client_fd, channel.topic);
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
  int client_fd,
  const horuslink::ChannelDescriptor & channel)
{
  if (channel.topic.empty()) {
    return false;
  }

  const bool success = topic_manager_->unregister_subscriber(channel.topic, client_fd);
  untrack_client_subscriber(client_fd, channel.topic);
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
  int client_fd,
  const horuslink::ChannelDescriptor & channel)
{
  if (channel.topic.empty()) {
    return false;
  }

  const bool success = topic_manager_->unregister_publisher(channel.topic, client_fd);
  untrack_client_publisher(client_fd, channel.topic);
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
  int client_fd,
  const horuslink::ChannelDescriptor & channel)
{
  if (channel.topic.empty()) {
    return false;
  }

  const bool success = service_manager_->unregister_ros_service(channel.topic);
  untrack_client_ros_service(client_fd, channel.topic);
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
  int client_fd,
  const horuslink::ChannelDescriptor & channel,
  const std::vector<uint8_t> & payload)
{
  stats_.messages_routed++;

  if (channel.topic.empty()) {
    stats_.routing_errors++;
    return false;
  }

  if (control_lease_manager_ && control_lease_manager_->is_internal_topic(channel.topic)) {
    const bool handled = control_lease_manager_->handle_internal_message(
      client_fd,
      channel.topic,
      payload);
    if (!handled) {
      stats_.routing_errors++;
    }
    return handled;
  }

  std::string denied_reason;
  std::string denied_robot_name;
  if (control_lease_manager_ &&
    !control_lease_manager_->authorize_command_publish(
      client_fd,
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
      client_fd,
      channel.topic.c_str(),
      denied_robot_name.c_str(),
      denied_reason.c_str());
    return false;
  }

  const bool success = topic_manager_->publish_horuslink_message(channel.topic, payload);

  if (success) {
    stats_.messages_published++;
  } else {
    stats_.routing_errors++;
  }

  return success;
}

bool MessageRouter::route_horuslink_service_request(
  int client_fd,
  const horuslink::ChannelDescriptor & channel,
  uint32_t corr_id,
  const std::vector<uint8_t> & request)
{
  stats_.messages_routed++;

  if (channel.topic.empty() || corr_id == 0) {
    stats_.routing_errors++;
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(service_response_mutex_);
    service_response_client_[corr_id] = client_fd;
    service_response_topic_[corr_id] = channel.topic;
  }

  const bool success = service_manager_->call_ros_service_async(
    corr_id,
    channel.topic,
    request);
  if (!success) {
    std::lock_guard<std::mutex> lock(service_response_mutex_);
    service_response_client_.erase(corr_id);
    service_response_topic_.erase(corr_id);
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

void MessageRouter::track_client_subscriber(int client_fd, const std::string & topic)
{
  std::lock_guard<std::mutex> lock(client_resources_mutex_);
  auto & state = client_resources_[client_fd];
  state.subscribed_topics.insert(topic);
  RCLCPP_INFO(get_logger(), "Client %d subscribed topics=%zu (last=%s)",
              client_fd, state.subscribed_topics.size(), topic.c_str());
}

void MessageRouter::track_client_publisher(int client_fd, const std::string & topic)
{
  std::lock_guard<std::mutex> lock(client_resources_mutex_);
  client_resources_[client_fd].published_topics.insert(topic);
}

void MessageRouter::track_client_ros_service(int client_fd, const std::string & service_name)
{
  std::lock_guard<std::mutex> lock(client_resources_mutex_);
  client_resources_[client_fd].ros_services.insert(service_name);
}

void MessageRouter::track_client_unity_service(int client_fd, const std::string & service_name)
{
  std::lock_guard<std::mutex> lock(client_resources_mutex_);
  client_resources_[client_fd].unity_services.insert(service_name);
}

void MessageRouter::track_client_webrtc_session(int client_fd, const std::string & session_id)
{
  std::lock_guard<std::mutex> lock(client_resources_mutex_);
  client_resources_[client_fd].webrtc_session_ids.insert(session_id);
}

void MessageRouter::untrack_client_webrtc_session(int client_fd, const std::string & session_id)
{
  std::lock_guard<std::mutex> lock(client_resources_mutex_);
  auto it = client_resources_.find(client_fd);
  if (it == client_resources_.end()) {
    return;
  }
  it->second.webrtc_session_ids.erase(session_id);
}

void MessageRouter::untrack_client_subscriber(int client_fd, const std::string & topic)
{
  std::lock_guard<std::mutex> lock(client_resources_mutex_);
  auto it = client_resources_.find(client_fd);
  if (it == client_resources_.end()) {
    return;
  }
  it->second.subscribed_topics.erase(topic);
  RCLCPP_INFO(get_logger(), "Client %d subscribed topics=%zu after remove (%s)",
              client_fd, it->second.subscribed_topics.size(), topic.c_str());
}

void MessageRouter::untrack_client_publisher(int client_fd, const std::string & topic)
{
  std::lock_guard<std::mutex> lock(client_resources_mutex_);
  auto it = client_resources_.find(client_fd);
  if (it == client_resources_.end()) {
    return;
  }
  it->second.published_topics.erase(topic);
}

void MessageRouter::untrack_client_ros_service(int client_fd, const std::string & service_name)
{
  std::lock_guard<std::mutex> lock(client_resources_mutex_);
  auto it = client_resources_.find(client_fd);
  if (it == client_resources_.end()) {
    return;
  }
  it->second.ros_services.erase(service_name);
}

void MessageRouter::untrack_client_unity_service(int client_fd, const std::string & service_name)
{
  std::lock_guard<std::mutex> lock(client_resources_mutex_);
  auto it = client_resources_.find(client_fd);
  if (it == client_resources_.end()) {
    return;
  }
  it->second.unity_services.erase(service_name);
}

void MessageRouter::on_client_disconnected(int client_fd)
{
  ClientResourceState resources;
  bool had_resources = false;
  {
    std::lock_guard<std::mutex> lock(client_resources_mutex_);
    auto it = client_resources_.find(client_fd);
    if (it != client_resources_.end()) {
      resources = it->second;
      client_resources_.erase(it);
      had_resources = true;
    }
  }

  size_t removed_subscriber_topics = topic_manager_->unregister_all_client_subscribers(client_fd);
  size_t removed_publisher_owners = topic_manager_->unregister_all_client_publishers(client_fd);

  if (control_lease_manager_) {
    control_lease_manager_->on_client_disconnected(client_fd);
  }

  size_t removed_ros_services = 0;
  for (const auto & service_name : resources.ros_services) {
    if (service_manager_->unregister_ros_service(service_name)) {
      removed_ros_services++;
    }
  }

  size_t removed_unity_services = 0;
  for (const auto & service_name : resources.unity_services) {
    if (service_manager_->unregister_unity_service(service_name)) {
      removed_unity_services++;
    }
  }

  {
    std::lock_guard<std::mutex> lock(service_response_mutex_);
    for (auto it = service_response_client_.begin(); it != service_response_client_.end(); ) {
      if (it->second == client_fd) {
        service_response_topic_.erase(it->first);
        it = service_response_client_.erase(it);
      } else {
        ++it;
      }
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
      client_fd,
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
  } else {
    RCLCPP_WARN(get_logger(), "Unsupported WebRTC signal type: %s", type.c_str());
  }
}

void MessageRouter::handle_webrtc_offer(const nlohmann::json & payload)
{
  const std::string session_id = payload.value("session_id", "");
  const std::string sdp = payload.value("sdp", "");
  const std::string stream_topic = payload.value("stream_topic", "");
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

  remove_webrtc_session(session_id);
  RCLCPP_INFO(
    get_logger(),
    "Received WebRTC offer: session='%s' topic='%s' image_type='%s'",
    session_id.c_str(),
    stream_topic.c_str(),
    image_type.c_str());

  auto session = std::make_shared<WebRtcSession>();
  session->session_id = session_id;
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
  session->image_type = image_type;
  session->session_start_time = std::chrono::steady_clock::now();
  session->last_activity = session->session_start_time;
  session->last_incoming_frame_time = session->last_activity;
  session->last_push_time = session->last_activity;
  session->last_rtp_progress_time = session->last_activity;
  session->last_keyframe_request_time = session->last_activity;
  session->last_telemetry_log_time = session->last_activity;
  session->manager = std::make_shared<WebRTCManager>();

  WebRTCManager::Settings settings;
  settings.stun_servers = webrtc_default_stun_servers_;
  settings.encoder = webrtc_default_encoder_;
  settings.bitrate_kbps = webrtc_default_bitrate_kbps_;
  settings.framerate = webrtc_default_framerate_;
  settings.pipeline = webrtc_default_pipeline_;
  settings.wait_for_complete_gathering = true;

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
  settings.bitrate_kbps = std::max(100, get_int("bitrate_kbps", settings.bitrate_kbps));
  settings.framerate = std::max(1, get_int("framerate", settings.framerate));

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
    [this, session_id](const std::string & type, const nlohmann::json & data) {
      nlohmann::json out;
      out["session_id"] = session_id;
      out["type"] = type;
      if (data.is_object()) {
        for (auto it = data.begin(); it != data.end(); ++it) {
          out[it.key()] = it.value();
        }
      }
      publish_webrtc_signal(out);
    });

  auto qos = rclcpp::SensorDataQoS();
  const bool compressed =
    image_type == "compressed" ||
    stream_topic.find("/compressed") != std::string::npos;
  if (compressed) {
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

  session->running = true;
  session->worker_thread = std::thread(&MessageRouter::process_webrtc_frames, this, session);

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
  session->last_activity = std::chrono::steady_clock::now();
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
  session->last_activity = std::chrono::steady_clock::now();
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
    if (session->frame_queue.size() >= webrtc_max_queue_size_) {
      session->frame_queue.pop_front();
      session->dropped_queue_frames++;
    }
    session->frame_queue.push_back(msg);
    session->incoming_frames++;
    session->last_incoming_frame_time = std::chrono::steady_clock::now();
  }
  session->last_activity = std::chrono::steady_clock::now();
  session->frame_queue_cv.notify_one();
}

void MessageRouter::process_webrtc_frames(const std::shared_ptr<WebRtcSession> & session)
{
  const auto kWarmupWindow = std::chrono::milliseconds(2000);
  const auto kIncomingActivityWindow = std::chrono::milliseconds(2500);
  const auto kRtpStallThreshold = std::chrono::milliseconds(2500);
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

      frame_variant = session->frame_queue.back();
      session->frame_queue.clear();
    }

    std::vector<uint8_t> rgb_data;
    bool pushed_frame = false;
    if (std::holds_alternative<sensor_msgs::msg::Image::SharedPtr>(frame_variant)) {
      auto raw = std::get<sensor_msgs::msg::Image::SharedPtr>(frame_variant);
      if (!raw) {
        continue;
      }
      if (convert_to_rgb(*raw, rgb_data, session->waned_unsupported_encoding)) {
        session->manager->push_frame(rgb_data, static_cast<int>(raw->width),
            static_cast<int>(raw->height), "RGB");
        pushed_frame = true;
      }
    } else {
      auto compressed = std::get<sensor_msgs::msg::CompressedImage::SharedPtr>(frame_variant);
      if (!compressed) {
        continue;
      }
      int width = 0;
      int height = 0;
      if (decompress_to_rgb(*compressed, rgb_data, width, height,
          session->waned_unsupported_encoding))
      {
        session->manager->push_frame(rgb_data, width, height, "RGB");
        pushed_frame = true;
      }
    }

    const auto now = std::chrono::steady_clock::now();
    bool should_request_keyframe = false;
    bool should_log_telemetry = false;
    uint64_t incoming_frames = 0;
    uint64_t pushed_frames = 0;
    uint64_t dropped_frames = 0;
    uint64_t rtp_packets = 0;
    uint64_t rtp_bytes = 0;
    bool peer_connected = false;
    bool track_open = false;

    {
      std::lock_guard<std::mutex> lock(session->frame_queue_mutex);
      if (pushed_frame) {
        session->pushed_frames++;
        session->last_push_time = now;
      }

      if (session->manager) {
        rtp_packets = session->manager->get_rtp_packets_sent();
        rtp_bytes = session->manager->get_rtp_bytes_sent();
        peer_connected = session->manager->is_peer_connected();
        track_open = session->manager->is_video_track_open();
      }

      if (rtp_packets > session->last_seen_rtp_packets) {
        session->last_seen_rtp_packets = rtp_packets;
        session->last_rtp_progress_time = now;
      }

      const bool warmupElapsed =
        session->session_start_time.time_since_epoch().count() > 0 &&
        (now - session->session_start_time) > kWarmupWindow;
      const bool has_recent_incoming =
        session->last_incoming_frame_time.time_since_epoch().count() > 0 &&
        (now - session->last_incoming_frame_time) < kIncomingActivityWindow;
      const bool rtp_stalled =
        warmupElapsed &&
        peer_connected &&
        track_open &&
        has_recent_incoming &&
        session->last_rtp_progress_time.time_since_epoch().count() > 0 &&
        (now - session->last_rtp_progress_time) > kRtpStallThreshold;
      if (rtp_stalled &&
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
      RCLCPP_WARN(
        get_logger(),
        "WebRTC session '%s' stall detected (incoming=%" PRIu64
        " pushed=%" PRIu64 " dropped=%" PRIu64 " rtp_packets=%" PRIu64
        "). Requested keyframe (cooldown=%" PRId64
        "ms, stall_threshold=%" PRId64 "ms).",
        session->session_id.c_str(),
        incoming_frames,
        pushed_frames,
        dropped_frames,
        rtp_packets,
        static_cast<int64_t>(kKeyframeCooldown.count()),
        static_cast<int64_t>(kRtpStallThreshold.count()));
    }

    if (should_log_telemetry) {
      RCLCPP_INFO(
        get_logger(),
        "WebRTC session '%s' telemetry: incoming=%" PRIu64
        " pushed=%" PRIu64 " dropped=%" PRIu64 " rtp_packets=%" PRIu64
        " rtp_bytes=%" PRIu64 " peer=%s track=%s",
        session->session_id.c_str(),
        incoming_frames,
        pushed_frames,
        dropped_frames,
        rtp_packets,
        rtp_bytes,
        peer_connected ? "connected" : "disconnected",
        track_open ? "open" : "closed");
    }

    session->last_activity = now;
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
    uint64_t rtp_packets = session->manager ? session->manager->get_rtp_packets_sent() : 0;
    uint64_t rtp_bytes = session->manager ? session->manager->get_rtp_bytes_sent() : 0;
    RCLCPP_INFO(
      get_logger(),
      "Stopping WebRTC session '%s' topic='%s' telemetry: incoming=%" PRIu64
      " pushed=%" PRIu64 " dropped=%" PRIu64 " rtp_packets=%" PRIu64
      " rtp_bytes=%" PRIu64,
      session->session_id.c_str(),
      session->stream_topic.c_str(),
      incoming_frames,
      pushed_frames,
      dropped_frames,
      rtp_packets,
      rtp_bytes);

    session->running = false;
    session->frame_queue_cv.notify_all();
    if (session->worker_thread.joinable()) {
      session->worker_thread.join();
    }
    session->raw_sub.reset();
    session->compressed_sub.reset();
    session->manager.reset();
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
      const auto idle = std::chrono::duration_cast<std::chrono::seconds>(
        now - session->last_activity);
      if (idle.count() > webrtc_session_timeout_sec_) {
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
