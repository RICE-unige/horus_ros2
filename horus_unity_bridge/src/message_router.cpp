// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#include "horus_unity_bridge/message_router.hpp"
#include <sstream>
#include <nlohmann/json.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <rclcpp/qos.hpp>
#include <algorithm>
#include <cctype>
#include <cstring>

#ifdef ENABLE_WEBRTC
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#endif

namespace horus_unity_bridge
{

MessageRouter::MessageRouter()
  : Node("horus_unity_bridge")
{
  topic_manager_ = std::make_unique<TopicManager>(this);
  service_manager_ = std::make_unique<ServiceManager>(this);

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
    [this](uint32_t srv_id, const std::vector<uint8_t>& response) {
      // Send service response back to the requesting Unity client
      int target_fd = -1;
      {
        std::lock_guard<std::mutex> lock(service_response_mutex_);
        auto it = service_response_client_.find(srv_id);
        if (it != service_response_client_.end()) {
          target_fd = it->second;
          service_response_client_.erase(it);
        }
      }
      if (send_callback_ && target_fd != -1) {
        std::lock_guard<std::mutex> lock(send_mutex_);
        // First send __response command with srv_id
        nlohmann::json response_cmd;
        response_cmd["srv_id"] = srv_id;
        std::string json_str = response_cmd.dump();
        std::vector<uint8_t> cmd_data(json_str.begin(), json_str.end());
        send_callback_(target_fd, "__response", cmd_data);
        
        // Then send the serialized response payload
        send_callback_(target_fd, "__response_payload", response);
      }
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

bool MessageRouter::route_message(int client_fd, const ProtocolMessage& message)
{
  stats_.messages_routed++;
  
  // Ignore keepalive messages
  if (message.is_keepalive()) {
    return true;
  }
  
  // Handle system commands
  if (message.is_system_command()) {
    return handle_system_command(client_fd, message);
  }
  
  // If a service request was announced, treat this payload as the request body
  if (pending_service_request_id_ != 0) {
    uint32_t srv_id = pending_service_request_id_;
    pending_service_request_id_ = 0;
    bool ok = service_manager_->call_ros_service_async(srv_id, message.destination, message.payload);
    if (!ok) {
      stats_.routing_errors++;
      return false;
    }
    return true;
  }
  
  // If a service response was announced, treat this payload as Unity's response
  if (pending_service_response_id_ != 0) {
    uint32_t srv_id = pending_service_response_id_;
    pending_service_response_id_ = 0;
    service_manager_->handle_unity_service_response(srv_id, message.payload);
    return true;
  }
  
  // Regular message - publish to ROS
  bool success = topic_manager_->publish_message(message.destination, message.payload);
  
  if (success) {
    stats_.messages_published++;
  } else {
    stats_.routing_errors++;
  }
  
  return success;
}

bool MessageRouter::handle_system_command(int client_fd, const ProtocolMessage& message)
{
  stats_.system_commands_processed++;
  
  std::string command = message.destination;
  std::string params_json(message.payload.begin(), message.payload.end());
  
  RCLCPP_DEBUG(get_logger(), "System command: %s", command.c_str());
  
  try {
    if (command == "__subscribe") {
      handle_subscribe_command(client_fd, params_json);
    } else if (command == "__publish") {
      handle_publish_command(client_fd, params_json);
    } else if (command == "__topic_list") {
      handle_topic_list_command(client_fd);
    } else if (command == "__handshake") {
      handle_handshake_command(client_fd);
    } else if (command == "__ros_service") {
      handle_ros_service_command(client_fd, params_json);
    } else if (command == "__unity_service") {
      handle_unity_service_command(client_fd, params_json);
    } else if (command == "__request") {
      handle_request_command(client_fd, params_json);
    } else if (command == "__response") {
      handle_response_command(client_fd, params_json);
    } else {
      RCLCPP_WARN(get_logger(), "Unknown system command: %s", command.c_str());
      send_error_to_client(client_fd, "Unknown system command: " + command);
      return false;
    }
    
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Error handling system command %s: %s", 
                 command.c_str(), e.what());
    send_error_to_client(client_fd, std::string("Error: ") + e.what());
    stats_.routing_errors++;
    return false;
  }
}

void MessageRouter::handle_subscribe_command(int client_fd, const std::string& params)
{
  // Parse JSON parameters
  std::unordered_map<std::string, std::string> param_map;
  if (!parse_json_params(params, param_map)) {
    send_error_to_client(client_fd, "Failed to parse subscribe parameters");
    return;
  }
  
  std::string topic = param_map["topic"];
  std::string message_name = param_map["message_name"];
  
  if (topic.empty() || message_name.empty()) {
    send_error_to_client(client_fd, "Missing required parameters: topic, message_name");
    return;
  }
  
  // Register subscriber to forward ROS messages to Unity
  bool success = topic_manager_->register_subscriber(
    topic,
    message_name,
    [this, client_fd](const std::string& topic, const std::vector<uint8_t>& data) {
      // Forward to Unity client
      if (send_callback_) {
        send_callback_(client_fd, topic, data);
      }
    }
  );
  
  if (success) {
    RCLCPP_INFO(get_logger(), "Registered Unity subscriber: %s [%s]", 
                topic.c_str(), message_name.c_str());
    send_log_to_client(client_fd, "info", 
                      "RegisterSubscriber(" + topic + ", " + message_name + ") OK");
  } else {
    send_error_to_client(client_fd, "Failed to register subscriber: " + topic);
  }
}

void MessageRouter::handle_publish_command(int client_fd, const std::string& params)
{
  // Parse JSON parameters
  std::unordered_map<std::string, std::string> param_map;
  if (!parse_json_params(params, param_map)) {
    send_error_to_client(client_fd, "Failed to parse publish parameters");
    return;
  }
  
  std::string topic = param_map["topic"];
  std::string message_name = param_map["message_name"];
  
  if (topic.empty() || message_name.empty()) {
    send_error_to_client(client_fd, "Missing required parameters: topic, message_name");
    return;
  }
  
  // Register publisher so Unity can publish to ROS
  bool success = topic_manager_->register_publisher(topic, message_name);
  
  if (success) {
    RCLCPP_INFO(get_logger(), "Registered Unity publisher: %s [%s]", 
                topic.c_str(), message_name.c_str());
    send_log_to_client(client_fd, "info",
                      "RegisterPublisher(" + topic + ", " + message_name + ") OK");
  } else {
    send_error_to_client(client_fd, "Failed to register publisher: " + topic);
  }
}

void MessageRouter::handle_ros_service_command(int client_fd, const std::string& params)
{
  // Parse JSON parameters
  std::unordered_map<std::string, std::string> param_map;
  if (!parse_json_params(params, param_map)) {
    send_error_to_client(client_fd, "Failed to parse ROS service parameters");
    return;
  }
  
  std::string service_name = param_map["topic"];
  std::string service_type = param_map["message_name"];
  
  if (service_name.empty() || service_type.empty()) {
    send_error_to_client(client_fd, "Missing required parameters: topic, message_name");
    return;
  }
  
  // Register ROS service client (Unity will call this ROS service)
  bool success = service_manager_->register_ros_service(service_name, service_type);
  
  if (success) {
    RCLCPP_INFO(get_logger(), "Registered ROS service client: %s [%s]",
                service_name.c_str(), service_type.c_str());
    send_log_to_client(client_fd, "info",
                      "RegisterRosService(" + service_name + ", " + service_type + ") OK");
  } else {
    send_error_to_client(client_fd, "Failed to register ROS service: " + service_name);
  }
}


void MessageRouter::handle_unity_service_command(int client_fd, const std::string& params)
{
  // Parse JSON parameters
  std::unordered_map<std::string, std::string> param_map;
  if (!parse_json_params(params, param_map)) {
    send_error_to_client(client_fd, "Failed to parse Unity service parameters");
    return;
  }
  
  std::string service_name = param_map["topic"];
  std::string service_type = param_map["message_name"];
  
  if (service_name.empty() || service_type.empty()) {
    send_error_to_client(client_fd, "Missing required parameters: topic, message_name");
    return;
  }
  
  // Register Unity service server (ROS can call this service, Unity responds)
  auto request_callback = [this, client_fd](
    uint32_t srv_id,
    const std::string& service_name,
    const std::vector<uint8_t>& request)
  {
    // Send request to Unity client
    if (send_callback_) {
      std::lock_guard<std::mutex> lock(send_mutex_);
      // Send __request command with srv_id first
      nlohmann::json req_cmd;
      req_cmd["srv_id"] = srv_id;
      std::string json_str = req_cmd.dump();
      std::vector<uint8_t> cmd_data(json_str.begin(), json_str.end());
      send_callback_(client_fd, "__request", cmd_data);
      
      // Then send the actual request data
      send_callback_(client_fd, service_name, request);
    }
  };
  
  bool success = service_manager_->register_unity_service(
    service_name, service_type, request_callback);
  
  if (success) {
    RCLCPP_INFO(get_logger(), "Registered Unity service server: %s [%s]",
                service_name.c_str(), service_type.c_str());
    send_log_to_client(client_fd, "info",
                      "RegisterUnityService(" + service_name + ", " + service_type + ") OK");
  } else {
    send_error_to_client(client_fd, "Failed to register Unity service: " + service_name);
  }
}

void MessageRouter::handle_request_command(int client_fd, const std::string& params)
{
  // Parse srv_id from JSON
  std::unordered_map<std::string, std::string> param_map;
  if (!parse_json_params(params, param_map)) {
    send_error_to_client(client_fd, "Failed to parse request parameters");
    return;
  }
  
  uint32_t srv_id = std::stoul(param_map["srv_id"]);
  
  // Store pending service request ID
  // The next non-system message will be the service request data
  pending_service_request_id_ = srv_id;
  
  // Remember which client should receive the response
  {
    std::lock_guard<std::mutex> lock(service_response_mutex_);
    service_response_client_[srv_id] = client_fd;
  }
  
  RCLCPP_DEBUG(get_logger(), "Request command received [ID: %u]", srv_id);
}

void MessageRouter::handle_response_command(int client_fd, const std::string& params)
{
  // Parse srv_id from JSON
  std::unordered_map<std::string, std::string> param_map;
  if (!parse_json_params(params, param_map)) {
    send_error_to_client(client_fd, "Failed to parse response parameters");
    return;
  }
  
  uint32_t srv_id = std::stoul(param_map["srv_id"]);
  
  // Store pending service response ID
  // The next non-system message will be the service response data
  pending_service_response_id_ = srv_id;
  
  RCLCPP_DEBUG(get_logger(), "Response command received [ID: %u]", srv_id);
}


void MessageRouter::handle_topic_list_command(int client_fd)
{
  // Get all ROS topics
  auto topic_names_and_types = get_topic_names_and_types();
  
  // Build response
  nlohmann::json response;
  response["topics"] = nlohmann::json::array();
  response["types"] = nlohmann::json::array();
  
  for (const auto& [topic, types] : topic_names_and_types) {
    response["topics"].push_back(topic);
    if (!types.empty()) {
      // PATCH: Denormalize type name for Unity (remove /msg/ or /srv/)
      // Unity expects "std_msgs/String", ROS 2 gives "std_msgs/msg/String"
      std::string type = types[0];
      std::string denormalized = type;
      
      size_t msg_pos = type.find("/msg/");
      if (msg_pos != std::string::npos) {
        denormalized = type.substr(0, msg_pos) + "/" + type.substr(msg_pos + 5);
      } else {
        size_t srv_pos = type.find("/srv/");
        if (srv_pos != std::string::npos) {
          denormalized = type.substr(0, srv_pos) + "/" + type.substr(srv_pos + 5);
        }
      }
      
      response["types"].push_back(denormalized);
    } else {
      response["types"].push_back("unknown");
    }
  }
  
  std::string json_str = response.dump();
  
  // Send as system command response
  if (send_callback_) {
    std::vector<uint8_t> data(json_str.begin(), json_str.end());
    send_callback_(client_fd, "__topic_list", data);
  }
  
  RCLCPP_DEBUG(get_logger(), "Sent topic list with %zu topics", 
               topic_names_and_types.size());
}

void MessageRouter::handle_handshake_command(int client_fd)
{
  send_handshake(client_fd);
}

void MessageRouter::send_handshake(int client_fd)
{
  if (!send_callback_) {
    return;
  }
  
  nlohmann::json handshake;
  handshake["version"] = "v0.7.0";
  
  // FIX: Flatten metadata to string for Unity compatibility
  nlohmann::json metadata;
  metadata["protocol"] = "ROS2";
  metadata["bridge"] = "HORUS";
  handshake["metadata"] = metadata.dump();
  
  std::string json_str = handshake.dump();
  std::vector<uint8_t> data(json_str.begin(), json_str.end());
  send_callback_(client_fd, "__handshake", data);
  RCLCPP_INFO(get_logger(), "Sent handshake to client %d", client_fd);
}

void MessageRouter::send_error_to_client(int client_fd, const std::string& error_message)
{
  send_log_to_client(client_fd, "error", error_message);
}

void MessageRouter::send_log_to_client(int client_fd, const std::string& level,
                                      const std::string& message)
{
  if (!send_callback_) {
    return;
  }
  
  nlohmann::json log_msg;
  log_msg["text"] = message;
  
  std::string json_str = log_msg.dump();
  std::string command = "__" + level;
  
  std::vector<uint8_t> data(json_str.begin(), json_str.end());
  send_callback_(client_fd, command, data);
}

bool MessageRouter::parse_json_params(const std::string& json_str,
                                     std::unordered_map<std::string, std::string>& params)
{
  try {
    // Remove trailing null character if present
    std::string cleaned_json = json_str;
    if (!cleaned_json.empty() && cleaned_json.back() == '\0') {
      cleaned_json.pop_back();
    }
    
    auto json_obj = nlohmann::json::parse(cleaned_json);
    
    for (auto& [key, value] : json_obj.items()) {
      if (value.is_string()) {
        params[key] = value.get<std::string>();
      } else {
        params[key] = value.dump();
      }
    }
    
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to parse JSON: %s", e.what());
    return false;
  }
}

#ifdef ENABLE_WEBRTC
void MessageRouter::publish_webrtc_signal(const nlohmann::json& payload)
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
  auto not_space = [](unsigned char ch) { return !std::isspace(ch); };
  clean_json.erase(clean_json.begin(), std::find_if(clean_json.begin(), clean_json.end(), not_space));
  clean_json.erase(std::find_if(clean_json.rbegin(), clean_json.rend(), not_space).base(), clean_json.end());
  if (clean_json.empty()) {
    return;
  }

  nlohmann::json payload;
  try {
    payload = nlohmann::json::parse(clean_json);
  } catch (const std::exception& e) {
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

void MessageRouter::handle_webrtc_offer(const nlohmann::json& payload)
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

  auto get_int = [&payload](const char* key, int fallback) {
    if (!payload.contains(key)) {
      return fallback;
    }
    try {
      return payload.at(key).is_number_integer()
               ? payload.at(key).get<int>()
               : std::stoi(payload.at(key).get<std::string>());
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
        payload.at("wait_for_complete_gathering").is_boolean()
          ? payload.at("wait_for_complete_gathering").get<bool>()
          : (payload.at("wait_for_complete_gathering").get<std::string>() != "false");
    } catch (...) {
      settings.wait_for_complete_gathering = true;
    }
  }
  if (payload.contains("stun_servers")) {
    try {
      settings.stun_servers.clear();
      if (payload.at("stun_servers").is_array()) {
        for (const auto& entry : payload.at("stun_servers")) {
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
    [this, session_id](const std::string& type, const nlohmann::json& data) {
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

void MessageRouter::handle_webrtc_candidate(const nlohmann::json& payload)
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
      sdp_mline_index = payload.at("sdpMLineIndex").is_number_integer()
        ? payload.at("sdpMLineIndex").get<int>()
        : std::stoi(payload.at("sdpMLineIndex").get<std::string>());
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

void MessageRouter::handle_webrtc_stop(const nlohmann::json& payload)
{
  const std::string session_id = payload.value("session_id", "");
  if (session_id.empty()) {
    return;
  }
  remove_webrtc_session(session_id);
}

void MessageRouter::enqueue_webrtc_raw_frame(
  const std::shared_ptr<WebRtcSession>& session,
  const sensor_msgs::msg::Image::SharedPtr& msg)
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
  const std::shared_ptr<WebRtcSession>& session,
  const sensor_msgs::msg::CompressedImage::SharedPtr& msg)
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

void MessageRouter::process_webrtc_frames(const std::shared_ptr<WebRtcSession>& session)
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
      if (convert_to_rgb(*raw, rgb_data, session->warned_unsupported_encoding)) {
        session->manager->push_frame(rgb_data, static_cast<int>(raw->width), static_cast<int>(raw->height), "RGB");
        pushed_frame = true;
      }
    } else {
      auto compressed = std::get<sensor_msgs::msg::CompressedImage::SharedPtr>(frame_variant);
      if (!compressed) {
        continue;
      }
      int width = 0;
      int height = 0;
      if (decompress_to_rgb(*compressed, rgb_data, width, height, session->warned_unsupported_encoding)) {
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
          (now - session->last_keyframe_request_time) > kKeyframeCooldown) {
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
        "WebRTC session '%s' stall detected (incoming=%llu pushed=%llu dropped=%llu rtp_packets=%llu). Requested keyframe (cooldown=%ldms, stall_threshold=%ldms).",
        session->session_id.c_str(),
        static_cast<unsigned long long>(incoming_frames),
        static_cast<unsigned long long>(pushed_frames),
        static_cast<unsigned long long>(dropped_frames),
        static_cast<unsigned long long>(rtp_packets),
        static_cast<long>(kKeyframeCooldown.count()),
        static_cast<long>(kRtpStallThreshold.count()));
    }

    if (should_log_telemetry) {
      RCLCPP_INFO(
        get_logger(),
        "WebRTC session '%s' telemetry: incoming=%llu pushed=%llu dropped=%llu rtp_packets=%llu rtp_bytes=%llu peer=%s track=%s",
        session->session_id.c_str(),
        static_cast<unsigned long long>(incoming_frames),
        static_cast<unsigned long long>(pushed_frames),
        static_cast<unsigned long long>(dropped_frames),
        static_cast<unsigned long long>(rtp_packets),
        static_cast<unsigned long long>(rtp_bytes),
        peer_connected ? "connected" : "disconnected",
        track_open ? "open" : "closed");
    }

    session->last_activity = now;
  }
}

bool MessageRouter::convert_to_rgb(
  const sensor_msgs::msg::Image& msg,
  std::vector<uint8_t>& output,
  bool& warned_flag)
{
  const auto width = static_cast<size_t>(msg.width);
  const auto height = static_cast<size_t>(msg.height);
  const auto row_stride = static_cast<size_t>(msg.step);
  if (width == 0 || height == 0 || row_stride == 0) {
    return false;
  }

  auto equals_ignore_case = [](const std::string& lhs, const std::string& rhs) {
    if (lhs.size() != rhs.size()) {
      return false;
    }
    for (size_t i = 0; i < lhs.size(); ++i) {
      if (std::tolower(static_cast<unsigned char>(lhs[i])) !=
          std::tolower(static_cast<unsigned char>(rhs[i]))) {
        return false;
      }
    }
    return true;
  };

  if (equals_ignore_case(msg.encoding, sensor_msgs::image_encodings::RGB8)) {
    output.resize(width * height * 3);
    for (size_t y = 0; y < height; ++y) {
      const uint8_t* row = msg.data.data() + (y * row_stride);
      std::memcpy(output.data() + (y * width * 3), row, width * 3);
    }
    return true;
  }

  if (equals_ignore_case(msg.encoding, sensor_msgs::image_encodings::BGR8)) {
    output.resize(width * height * 3);
    for (size_t y = 0; y < height; ++y) {
      const uint8_t* row = msg.data.data() + (y * row_stride);
      for (size_t x = 0; x < width; ++x) {
        const uint8_t* pixel = row + (x * 3);
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
      const uint8_t* row = msg.data.data() + (y * row_stride);
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

  if (!warned_flag) {
    RCLCPP_WARN(get_logger(), "Unsupported image encoding for WebRTC stream: %s", msg.encoding.c_str());
    warned_flag = true;
  }
  return false;
}

bool MessageRouter::decompress_to_rgb(
  const sensor_msgs::msg::CompressedImage& msg,
  std::vector<uint8_t>& output,
  int& width,
  int& height,
  bool& warned_flag)
{
  try {
    cv::Mat encoded(1, static_cast<int>(msg.data.size()), CV_8UC1, const_cast<uint8_t*>(msg.data.data()));
    cv::Mat decoded = cv::imdecode(encoded, cv::IMREAD_COLOR);
    if (decoded.empty()) {
      if (!warned_flag) {
        RCLCPP_WARN(get_logger(), "Failed to decode compressed image for WebRTC session");
        warned_flag = true;
      }
      return false;
    }

    cv::Mat rgb;
    cv::cvtColor(decoded, rgb, cv::COLOR_BGR2RGB);
    width = rgb.cols;
    height = rgb.rows;
    output.assign(rgb.data, rgb.data + (rgb.total() * rgb.elemSize()));
    return true;
  } catch (const cv::Exception& e) {
    if (!warned_flag) {
      RCLCPP_ERROR(get_logger(), "OpenCV decode exception in WebRTC session: %s", e.what());
      warned_flag = true;
    }
    return false;
  }
}

void MessageRouter::remove_webrtc_session(const std::string& session_id)
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
      "Stopping WebRTC session '%s' topic='%s' telemetry: incoming=%llu pushed=%llu dropped=%llu rtp_packets=%llu rtp_bytes=%llu",
      session->session_id.c_str(),
      session->stream_topic.c_str(),
      static_cast<unsigned long long>(incoming_frames),
      static_cast<unsigned long long>(pushed_frames),
      static_cast<unsigned long long>(dropped_frames),
      static_cast<unsigned long long>(rtp_packets),
      static_cast<unsigned long long>(rtp_bytes));

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
    for (const auto& item : webrtc_sessions_) {
      session_ids.push_back(item.first);
    }
  }
  for (const auto& session_id : session_ids) {
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
    for (const auto& item : webrtc_sessions_) {
      const auto& session = item.second;
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

  for (const auto& session_id : stale_ids) {
    RCLCPP_INFO(get_logger(), "Cleaning stale WebRTC session '%s'", session_id.c_str());
    remove_webrtc_session(session_id);
  }
}
#endif

} // namespace horus_unity_bridge
