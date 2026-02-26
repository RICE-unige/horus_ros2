// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#include "horus_unity_bridge/control_lease_manager.hpp"

#include <nlohmann/json.hpp>
#include <rclcpp/serialization.hpp>

#include <algorithm>
#include <chrono>
#include <cstring>

namespace horus_unity_bridge
{

namespace
{
constexpr const char* kLeaseRequestTopic = "/horus/multi_operator/control_lease_request";
constexpr const char* kControlTopicCatalogTopic = "/horus/multi_operator/control_topic_catalog";
constexpr const char* kLeaseStateTopic = "/horus/multi_operator/control_lease_state";
}

std::size_t ControlLeaseManager::DeniedRateLimitKeyHash::operator()(const DeniedRateLimitKey& value) const noexcept
{
  return std::hash<int>{}(value.client_fd) ^ (std::hash<std::string>{}(value.topic) << 1);
}

ControlLeaseManager::ControlLeaseManager(rclcpp::Node* node)
  : node_(node)
{
  if (node_ == nullptr) {
    return;
  }

  lease_ttl_ms_ = static_cast<uint64_t>(
    std::max<int64_t>(500, node_->declare_parameter<int64_t>("multi_operator.control_lease_ttl_ms", 3000)));
  denied_event_rate_limit_ms_ = static_cast<uint64_t>(
    std::max<int64_t>(100, node_->declare_parameter<int64_t>("multi_operator.control_lease_denied_event_rate_limit_ms", 500)));

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliability(rclcpp::ReliabilityPolicy::Reliable)
    .durability(rclcpp::DurabilityPolicy::TransientLocal);
  lease_state_pub_ = node_->create_publisher<std_msgs::msg::String>(lease_state_topic(), qos);

  lease_expiry_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&ControlLeaseManager::cleanup_expired_leases, this));

  RCLCPP_INFO(node_->get_logger(),
              "Control lease manager enabled (ttl=%llums)",
              static_cast<unsigned long long>(lease_ttl_ms_));
}

const std::string& ControlLeaseManager::lease_request_topic()
{
  static const std::string topic = kLeaseRequestTopic;
  return topic;
}

const std::string& ControlLeaseManager::control_topic_catalog_topic()
{
  static const std::string topic = kControlTopicCatalogTopic;
  return topic;
}

const std::string& ControlLeaseManager::lease_state_topic()
{
  static const std::string topic = kLeaseStateTopic;
  return topic;
}

bool ControlLeaseManager::is_internal_topic(const std::string& topic) const
{
  return topic == lease_request_topic() || topic == control_topic_catalog_topic();
}

bool ControlLeaseManager::handle_internal_message(int client_fd,
                                                  const std::string& topic,
                                                  const std::vector<uint8_t>& serialized_payload)
{
  std::string payload;
  if (!try_decode_string_msg(serialized_payload, payload)) {
    if (node_ != nullptr) {
      RCLCPP_WARN(node_->get_logger(), "Control lease manager failed to decode std_msgs/String payload on topic '%s'",
                  topic.c_str());
    }
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  cleanup_expired_leases_locked(now_ms(), false);

  if (topic == lease_request_topic()) {
    return handle_lease_request_message_locked(client_fd, payload);
  }
  if (topic == control_topic_catalog_topic()) {
    return handle_catalog_message_locked(client_fd, payload);
  }

  return false;
}

bool ControlLeaseManager::authorize_command_publish(int client_fd,
                                                    const std::string& topic,
                                                    std::string* denied_reason,
                                                    std::string* robot_name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  uint64_t now = now_ms();
  cleanup_expired_leases_locked(now, false);

  auto map_it = protected_topic_to_robot_.find(topic);
  if (map_it == protected_topic_to_robot_.end()) {
    return true;
  }

  const std::string& mapped_robot = map_it->second;
  if (robot_name != nullptr) {
    *robot_name = mapped_robot;
  }

  auto lease_it = leases_by_robot_.find(mapped_robot);
  if (lease_it == leases_by_robot_.end()) {
    return true;
  }

  const RobotControlLease& lease = lease_it->second;
  if (lease.holder_client_fd == client_fd) {
    return true;
  }

  if (denied_reason != nullptr) {
    *denied_reason = "robot_control_leased";
  }

  if (should_publish_denied_event_locked(client_fd, topic, now)) {
    publish_denied_locked(std::string(), mapped_robot, "robot_control_leased", lease.holder_app_id);
  }
  return false;
}

void ControlLeaseManager::on_client_disconnected(int client_fd)
{
  std::lock_guard<std::mutex> lock(mutex_);

  bool changed = false;
  for (auto it = leases_by_robot_.begin(); it != leases_by_robot_.end(); ) {
    if (it->second.holder_client_fd != client_fd) {
      ++it;
      continue;
    }
    it = leases_by_robot_.erase(it);
    changed = true;
  }

  if (changed) {
    publish_snapshot_locked("client_disconnected_release");
  }
}

bool ControlLeaseManager::handle_catalog_message_locked(int client_fd, const std::string& json_payload)
{
  (void)client_fd;
  nlohmann::json root = nlohmann::json::parse(json_payload, nullptr, false);
  if (root.is_discarded() || !root.is_object()) {
    return false;
  }

  const std::string role = root.value("role", "");
  if (!(role == "host" || role == "single")) {
    return true; // Ignore non-authoritative catalog publishers.
  }

  const std::string op = root.value("op", "snapshot");
  const std::string session_id = root.value("session_id", "");
  if (!session_id.empty()) {
    current_session_id_ = session_id;
  }

  if (op == "clear") {
    protected_topic_to_robot_.clear();
    publish_snapshot_locked("snapshot");
    return true;
  }

  if (!root.contains("robots") || !root["robots"].is_array()) {
    return false;
  }

  auto add_topics_for_robot = [this](const std::string& robot_name, const nlohmann::json& robot_obj) {
    auto add_topic = [this, &robot_name](const std::string& topic_value) {
      if (!topic_value.empty()) {
        protected_topic_to_robot_[topic_value] = robot_name;
      }
    };

    if (robot_obj.contains("protected_topics") && robot_obj["protected_topics"].is_array()) {
      for (const auto& item : robot_obj["protected_topics"]) {
        if (item.is_string()) {
          add_topic(item.get<std::string>());
        }
      }
      return;
    }

    const std::vector<std::string> fields = {
      "teleop_command_topic",
      "teleop_raw_input_topic",
      "teleop_head_pose_topic",
      "go_to_goal_topic",
      "go_to_cancel_topic",
      "waypoint_path_topic"
    };
    for (const auto& field : fields) {
      if (robot_obj.contains(field) && robot_obj[field].is_string()) {
        add_topic(robot_obj[field].get<std::string>());
      }
    }
  };

  if (op == "snapshot") {
    protected_topic_to_robot_.clear();
  }

  for (const auto& item : root["robots"]) {
    if (!item.is_object()) {
      continue;
    }
    std::string robot_name = item.value("robot_name", "");
    if (robot_name.empty()) {
      continue;
    }

    if (op == "remove") {
      for (auto it = protected_topic_to_robot_.begin(); it != protected_topic_to_robot_.end(); ) {
        if (it->second == robot_name) {
          it = protected_topic_to_robot_.erase(it);
        } else {
          ++it;
        }
      }
      continue;
    }

    if (op == "upsert" || op == "snapshot") {
      for (auto it = protected_topic_to_robot_.begin(); it != protected_topic_to_robot_.end(); ) {
        if (it->second == robot_name) {
          it = protected_topic_to_robot_.erase(it);
        } else {
          ++it;
        }
      }
      add_topics_for_robot(robot_name, item);
    }
  }

  publish_snapshot_locked("snapshot");
  return true;
}

bool ControlLeaseManager::handle_lease_request_message_locked(int client_fd, const std::string& json_payload)
{
  nlohmann::json root = nlohmann::json::parse(json_payload, nullptr, false);
  if (root.is_discarded() || !root.is_object()) {
    return false;
  }

  const std::string request_id = root.value("request_id", "");
  const std::string robot_name = root.value("robot_name", "");
  const std::string app_id = root.value("app_id", "");
  const std::string role = root.value("role", "");
  const std::string session_id = root.value("session_id", "");
  const std::string action = root.value("action", "");
  const bool panel_open = root.value("panel_open", false);
  const bool teleop_active = root.value("teleop_active", false);
  const bool task_active = root.value("task_active", false);
  const std::string task_kind = root.value("task_kind", "none");

  if (!session_id.empty()) {
    current_session_id_ = session_id;
  }

  if (robot_name.empty()) {
    return false;
  }

  const uint64_t now = now_ms();
  LeaseRequestResult result;
  if (action == "acquire") {
    result = apply_acquire_request_locked(
      client_fd, request_id, robot_name, app_id, role, session_id, panel_open, teleop_active, task_active, task_kind, now);
  } else if (action == "heartbeat") {
    result = apply_heartbeat_request_locked(
      client_fd, request_id, robot_name, app_id, role, session_id, panel_open, teleop_active, task_active, task_kind, now);
  } else if (action == "release") {
    result = apply_release_request_locked(client_fd, request_id, robot_name, now);
  } else {
    return false;
  }

  if (!result.granted) {
    publish_denied_locked(request_id,
                          robot_name,
                          result.reason.empty() ? "lease_denied" : result.reason,
                          result.current_lease.has_value() ? result.current_lease->holder_app_id : std::string());
  }
  return true;
}

ControlLeaseManager::LeaseRequestResult ControlLeaseManager::apply_acquire_request_locked(
  int client_fd,
  const std::string& request_id,
  const std::string& robot_name,
  const std::string& app_id,
  const std::string& role,
  const std::string& session_id,
  bool panel_open,
  bool teleop_active,
  bool task_active,
  const std::string& task_kind,
  uint64_t now)
{
  (void)request_id;
  LeaseRequestResult result;
  auto it = leases_by_robot_.find(robot_name);
  if (it == leases_by_robot_.end()) {
    RobotControlLease lease;
    lease.robot_name = robot_name;
    lease.holder_client_fd = client_fd;
    lease.holder_app_id = app_id;
    lease.holder_role = role;
    lease.session_id = session_id;
    lease.acquired_at_ms = now;
    lease.last_heartbeat_ms = now;
    lease.panel_open = panel_open;
    lease.teleop_active = teleop_active;
    lease.task_active = task_active;
    lease.task_kind = task_kind.empty() ? "none" : task_kind;
    lease.lease_version = next_lease_version_++;
    leases_by_robot_[robot_name] = lease;
    result.granted = true;
    result.current_lease = lease;
    publish_snapshot_locked("lease_granted", request_id, robot_name, std::string(), app_id);
    return result;
  }

  RobotControlLease& lease = it->second;
  if (lease.holder_client_fd == client_fd) {
    lease.holder_app_id = app_id.empty() ? lease.holder_app_id : app_id;
    lease.holder_role = role.empty() ? lease.holder_role : role;
    lease.session_id = session_id.empty() ? lease.session_id : session_id;
    lease.last_heartbeat_ms = now;
    lease.panel_open = panel_open;
    lease.teleop_active = teleop_active;
    lease.task_active = task_active;
    lease.task_kind = task_kind.empty() ? lease.task_kind : task_kind;
    lease.lease_version = next_lease_version_++;
    result.granted = true;
    result.already_owned_by_requester = true;
    result.current_lease = lease;
    publish_snapshot_locked("lease_updated", request_id, robot_name, std::string(), lease.holder_app_id);
    return result;
  }

  result.granted = false;
  result.reason = "robot_control_leased";
  result.current_lease = lease;
  return result;
}

ControlLeaseManager::LeaseRequestResult ControlLeaseManager::apply_heartbeat_request_locked(
  int client_fd,
  const std::string& request_id,
  const std::string& robot_name,
  const std::string& app_id,
  const std::string& role,
  const std::string& session_id,
  bool panel_open,
  bool teleop_active,
  bool task_active,
  const std::string& task_kind,
  uint64_t now)
{
  auto it = leases_by_robot_.find(robot_name);
  if (it == leases_by_robot_.end()) {
    // Heartbeat can create/refresh an implicit lease if the client already transitioned into control.
    return apply_acquire_request_locked(
      client_fd, request_id, robot_name, app_id, role, session_id, panel_open, teleop_active, task_active, task_kind, now);
  }

  LeaseRequestResult result;
  RobotControlLease& lease = it->second;
  if (lease.holder_client_fd != client_fd) {
    result.granted = false;
    result.reason = "heartbeat_not_owner";
    result.current_lease = lease;
    return result;
  }

  lease.holder_app_id = app_id.empty() ? lease.holder_app_id : app_id;
  lease.holder_role = role.empty() ? lease.holder_role : role;
  lease.session_id = session_id.empty() ? lease.session_id : session_id;
  lease.last_heartbeat_ms = now;
  lease.panel_open = panel_open;
  lease.teleop_active = teleop_active;
  lease.task_active = task_active;
  lease.task_kind = task_kind.empty() ? lease.task_kind : task_kind;
  lease.lease_version = next_lease_version_++;
  result.granted = true;
  result.current_lease = lease;
  publish_snapshot_locked("lease_updated", request_id, robot_name, std::string(), lease.holder_app_id);
  return result;
}

ControlLeaseManager::LeaseRequestResult ControlLeaseManager::apply_release_request_locked(
  int client_fd,
  const std::string& request_id,
  const std::string& robot_name,
  uint64_t now)
{
  (void)request_id;
  (void)now;
  LeaseRequestResult result;
  auto it = leases_by_robot_.find(robot_name);
  if (it == leases_by_robot_.end()) {
    result.granted = true;
    return result;
  }

  if (it->second.holder_client_fd != client_fd) {
    result.granted = false;
    result.reason = "release_not_owner";
    result.current_lease = it->second;
    return result;
  }

  const std::string holder_app_id = it->second.holder_app_id;
  leases_by_robot_.erase(it);
  result.granted = true;
  publish_snapshot_locked("lease_released", request_id, robot_name, std::string(), holder_app_id);
  return result;
}

void ControlLeaseManager::publish_snapshot_locked(const std::string& event_name,
                                                  const std::string& request_id,
                                                  const std::string& robot_name,
                                                  const std::string& denied_reason,
                                                  const std::string& holder_app_id)
{
  if (!lease_state_pub_) {
    return;
  }

  nlohmann::json root;
  root["session_id"] = current_session_id_;
  root["event"] = event_name;
  if (!request_id.empty()) {
    root["request_id"] = request_id;
  }
  if (!robot_name.empty()) {
    root["robot_name"] = robot_name;
  }
  root["lease_ttl_ms"] = lease_ttl_ms_;
  root["lease_version"] = next_lease_version_ > 0 ? (next_lease_version_ - 1) : 0;
  if (!holder_app_id.empty()) {
    root["holder_app_id"] = holder_app_id;
  }
  if (!denied_reason.empty()) {
    root["denied_reason"] = denied_reason;
  }

  nlohmann::json leases = nlohmann::json::array();
  for (const auto& pair : leases_by_robot_) {
    const RobotControlLease& lease = pair.second;
    nlohmann::json item;
    item["robot_name"] = lease.robot_name;
    item["holder_app_id"] = lease.holder_app_id;
    item["holder_role"] = lease.holder_role;
    item["session_id"] = lease.session_id;
    item["acquired_at_ms"] = lease.acquired_at_ms;
    item["last_heartbeat_ms"] = lease.last_heartbeat_ms;
    item["panel_open"] = lease.panel_open;
    item["teleop_active"] = lease.teleop_active;
    item["task_active"] = lease.task_active;
    item["task_kind"] = lease.task_kind;
    item["lease_version"] = lease.lease_version;
    leases.push_back(item);
  }
  root["leases"] = leases;

  std_msgs::msg::String msg;
  msg.data = root.dump();
  lease_state_pub_->publish(msg);
}

void ControlLeaseManager::publish_denied_locked(const std::string& request_id,
                                                const std::string& robot_name,
                                                const std::string& denied_reason,
                                                const std::string& holder_app_id)
{
  publish_snapshot_locked("lease_denied", request_id, robot_name, denied_reason, holder_app_id);
}

void ControlLeaseManager::cleanup_expired_leases()
{
  std::lock_guard<std::mutex> lock(mutex_);
  cleanup_expired_leases_locked(now_ms(), true);
}

void ControlLeaseManager::cleanup_expired_leases_locked(uint64_t now, bool publish_if_changed)
{
  bool changed = false;
  for (auto it = leases_by_robot_.begin(); it != leases_by_robot_.end(); ) {
    const uint64_t age_ms = (now >= it->second.last_heartbeat_ms) ? (now - it->second.last_heartbeat_ms) : 0;
    if (age_ms <= lease_ttl_ms_) {
      ++it;
      continue;
    }
    it = leases_by_robot_.erase(it);
    changed = true;
  }

  if (changed && publish_if_changed) {
    publish_snapshot_locked("lease_expired");
  }
}

bool ControlLeaseManager::should_publish_denied_event_locked(int client_fd, const std::string& topic, uint64_t now)
{
  DeniedRateLimitKey key;
  key.client_fd = client_fd;
  key.topic = topic;

  auto it = denied_publish_event_last_ms_.find(key);
  if (it != denied_publish_event_last_ms_.end()) {
    uint64_t delta = now >= it->second ? (now - it->second) : 0;
    if (delta < denied_event_rate_limit_ms_) {
      return false;
    }
  }

  denied_publish_event_last_ms_[key] = now;
  return true;
}

uint64_t ControlLeaseManager::now_ms()
{
  using Clock = std::chrono::steady_clock;
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now().time_since_epoch()).count());
}

bool ControlLeaseManager::try_decode_string_msg(const std::vector<uint8_t>& serialized_payload, std::string& out_data)
{
  try {
    rclcpp::SerializedMessage serialized(serialized_payload.size());
    auto& rcl_msg = serialized.get_rcl_serialized_message();
    if (serialized_payload.empty()) {
      std_msgs::msg::String msg;
      out_data = msg.data;
      return true;
    }
    std::memcpy(rcl_msg.buffer, serialized_payload.data(), serialized_payload.size());
    rcl_msg.buffer_length = serialized_payload.size();

    std_msgs::msg::String decoded;
    rclcpp::Serialization<std_msgs::msg::String> serializer;
    serializer.deserialize_message(&serialized, &decoded);
    out_data = decoded.data;
    return true;
  } catch (...) {
    out_data.clear();
    return false;
  }
}

} // namespace horus_unity_bridge
