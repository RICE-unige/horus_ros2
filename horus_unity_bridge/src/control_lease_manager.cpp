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

#include "horus_unity_bridge/control_lease_manager.hpp"
#include "horus_unity_bridge/serialized_payload.hpp"

#include <cinttypes>

#include <nlohmann/json.hpp>
#include <rclcpp/serialization.hpp>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <random>

namespace horus_unity_bridge
{

namespace
{
constexpr const char * kLeaseRequestTopic = "/horus/multi_operator/control_lease_request";
constexpr const char * kControlTopicCatalogTopic = "/horus/multi_operator/control_topic_catalog";
constexpr const char * kLeaseStateTopic = "/horus/multi_operator/control_lease_state";

std::string new_authority_generation()
{
  std::random_device random;
  return std::to_string(random()) + "-" + std::to_string(random()) + "-" +
    std::to_string(random()) + "-" + std::to_string(random());
}
}  // namespace

std::size_t ControlLeaseManager::DeniedRateLimitKeyHash::operator()(
  const DeniedRateLimitKey & value) const noexcept
{
  return std::hash<int>{}(value.client_fd) ^ (std::hash<std::string>{}(value.topic) << 1);
}

ControlLeaseManager::ControlLeaseManager(rclcpp::Node * node)
: node_(node)
{
  if (node_ == nullptr) {
    return;
  }

  lease_ttl_ms_ = static_cast<uint64_t>(
    std::max<int64_t>(500,
      node_->declare_parameter<int64_t>("multi_operator.control_lease_ttl_ms", 3000)));
  denied_event_rate_limit_ms_ = static_cast<uint64_t>(
    std::max<int64_t>(100,
      node_->declare_parameter<int64_t>("multi_operator.control_lease_denied_event_rate_limit_ms",
      500)));

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliability(rclcpp::ReliabilityPolicy::Reliable)
    .durability(rclcpp::DurabilityPolicy::TransientLocal);
  lease_state_pub_ = node_->create_publisher<std_msgs::msg::String>(lease_state_topic(), qos);

  lease_expiry_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&ControlLeaseManager::cleanup_expired_leases, this));

  RCLCPP_INFO(node_->get_logger(),
              "Control lease manager enabled (ttl=%" PRIu64 "ms)",
              static_cast<uint64_t>(lease_ttl_ms_));
}

const std::string & ControlLeaseManager::lease_request_topic()
{
  static const std::string topic = kLeaseRequestTopic;
  return topic;
}

const std::string & ControlLeaseManager::control_topic_catalog_topic()
{
  static const std::string topic = kControlTopicCatalogTopic;
  return topic;
}

const std::string & ControlLeaseManager::lease_state_topic()
{
  static const std::string topic = kLeaseStateTopic;
  return topic;
}

bool ControlLeaseManager::is_internal_topic(const std::string & topic) const
{
  return topic == lease_request_topic() || topic == control_topic_catalog_topic();
}

bool ControlLeaseManager::handle_internal_message(
  int client_fd,
  const std::string & topic,
  const std::vector<uint8_t> & serialized_payload)
{
  std::string payload;
  if (!try_decode_string_msg(serialized_payload, payload)) {
    if (node_ != nullptr) {
      RCLCPP_WARN(node_->get_logger(),
          "Control lease manager failed to decode std_msgs/String payload on topic '%s'",
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

bool ControlLeaseManager::authorize_command_publish(
  int client_fd,
  const std::string & topic,
  std::string * denied_reason,
  std::string * robot_name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  uint64_t now = now_ms();
  cleanup_expired_leases_locked(now, false);

  // Teach inputs are fail-closed even before a catalog reaches the bridge.
  const std::string teach_prefix = "/horus/teach/";
  const std::string teach_suffix = "/input";
  const bool teaching = topic.rfind(teach_prefix, 0) == 0 &&
    topic.size() > teach_prefix.size() + teach_suffix.size() &&
    topic.compare(topic.size() - teach_suffix.size(), teach_suffix.size(), teach_suffix) == 0;
  auto map_it = protected_topic_to_robot_.find(topic);
  if (!teaching && map_it == protected_topic_to_robot_.end()) {
    return true;
  }

  const std::string mapped_robot = teaching ?
    topic.substr(teach_prefix.size(), topic.size() - teach_prefix.size() - teach_suffix.size()) :
    map_it->second;
  if (robot_name != nullptr) {
    *robot_name = mapped_robot;
  }

  auto lease_it = leases_by_robot_.find(mapped_robot);
  if (lease_it == leases_by_robot_.end()) {
    if (teaching) {
      if (denied_reason != nullptr) {*denied_reason = "teaching_requires_lease";}
      return false;
    }
    return true;
  }

  const RobotControlLease & lease = lease_it->second;
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
  catalogs_by_client_.erase(client_fd);
  rebuild_catalog_locked();

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

void ControlLeaseManager::set_snapshot_sink(
  std::function<void(const std::string &)> sink, bool publish_ros)
{
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_sink_ = std::move(sink);
  if (!publish_ros) {lease_state_pub_.reset();}
}

bool ControlLeaseManager::owns_active_remote_lease(int client_fd, const std::string & topic)
{
  std::lock_guard<std::mutex> lock(mutex_);
  cleanup_expired_leases_locked(now_ms(), true);
  const auto robot = protected_topic_to_robot_.find(topic);
  std::string name;
  if (robot != protected_topic_to_robot_.end()) {name = robot->second;}
  else if (topic.rfind("/horus/teach/", 0) == 0 && topic.size() > 19 &&
    topic.compare(topic.size() - 6, 6, "/input") == 0)
  {name = topic.substr(13, topic.size() - 19);}
  if (name.empty()) {return false;}
  const auto lease = leases_by_robot_.find(name);
  return lease != leases_by_robot_.end() && lease->second.holder_client_fd == client_fd;
}

bool ControlLeaseManager::rebuild_catalog_locked()
{
  std::unordered_map<std::string, std::string> combined;
  for (const auto & catalog : catalogs_by_client_) {
    for (const auto & entry : catalog.second) {
      auto found = combined.find(entry.first);
      if (found != combined.end() && found->second != entry.second) {return false;}
      combined[entry.first] = entry.second;
    }
  }
  protected_topic_to_robot_ = std::move(combined);
  return true;
}

std::vector<std::string> ControlLeaseManager::protected_topics()
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> result;
  for (const auto & entry : protected_topic_to_robot_) {result.push_back(entry.first);}
  std::sort(result.begin(), result.end());
  return result;
}

bool ControlLeaseManager::handle_catalog_message_locked(
  int client_fd,
  const std::string & json_payload)
{
  nlohmann::json root = nlohmann::json::parse(json_payload, nullptr, false);
  if (root.is_discarded() || !root.is_object()) {
    return false;
  }

  const std::string role = root.value("role", "");
  const bool merge_only = role == "private" || role == "joiner";
  if (!(role == "host" || role == "single" || merge_only)) {
    return true; // Ignore non-authoritative catalog publishers.
  }

  const std::string op = root.value("op", "snapshot");
  if (merge_only && op != "upsert") {return false;}
  const std::string session_id = root.value("session_id", "");
  if (!session_id.empty()) {
    current_session_id_ = session_id;
  }

  if (op == "clear") {
    catalogs_by_client_.erase(client_fd);
    rebuild_catalog_locked();
    publish_snapshot_locked("snapshot");
    return true;
  }

  if (!root.contains("robots") || !root["robots"].is_array()) {
    return false;
  }

  const auto before = protected_topic_to_robot_;
  const auto own = catalogs_by_client_.find(client_fd);
  const auto own_before = own == catalogs_by_client_.end() ?
    std::unordered_map<std::string, std::string>{} : own->second;
  protected_topic_to_robot_ = own_before;
  if (root.value("replace_client_catalog", false)) {protected_topic_to_robot_.clear();}
  bool conflict = false;
  auto add_topics_for_robot = [this, merge_only, &conflict](const std::string & robot_name,
    const nlohmann::json & robot_obj) {
      auto add_topic = [this, &robot_name, merge_only, &conflict](const std::string & topic_value) {
          if (!topic_value.empty()) {
            const auto existing = protected_topic_to_robot_.find(topic_value);
            if (merge_only && existing != protected_topic_to_robot_.end() && existing->second != robot_name) {
              conflict = true; return;
            }
            protected_topic_to_robot_[topic_value] = robot_name;
          }
        };

      if (robot_obj.contains("protected_topics") && robot_obj["protected_topics"].is_array()) {
        for (const auto & item : robot_obj["protected_topics"]) {
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
      for (const auto & field : fields) {
        if (robot_obj.contains(field) && robot_obj[field].is_string()) {
          add_topic(robot_obj[field].get<std::string>());
        }
      }
    };

  if (op == "snapshot") {
    protected_topic_to_robot_.clear();
  }

  for (const auto & item : root["robots"]) {
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
        if (!merge_only && it->second == robot_name) {
          it = protected_topic_to_robot_.erase(it);
        } else {
          ++it;
        }
      }
      add_topics_for_robot(robot_name, item);
    }
  }

  if (conflict) {protected_topic_to_robot_ = before; return false;}
  catalogs_by_client_[client_fd] = protected_topic_to_robot_;
  if (!rebuild_catalog_locked()) {
    if (own_before.empty()) {catalogs_by_client_.erase(client_fd);}
    else {catalogs_by_client_[client_fd] = own_before;}
    protected_topic_to_robot_ = before;
    return false;
  }
  publish_snapshot_locked("snapshot");
  return true;
}

bool ControlLeaseManager::handle_lease_request_message_locked(
  int client_fd,
  const std::string & json_payload)
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
      client_fd, request_id, robot_name, app_id, role, session_id, panel_open, teleop_active,
        task_active, task_kind, now);
  } else if (action == "heartbeat") {
    result = apply_heartbeat_request_locked(
      client_fd, request_id, robot_name, app_id, role, session_id, panel_open, teleop_active,
        task_active, task_kind, now);
  } else if (action == "release") {
    result = apply_release_request_locked(client_fd, request_id, robot_name, now);
  } else {
    return false;
  }

  if (!result.granted) {
    publish_denied_locked(request_id,
                          robot_name,
                          result.reason.empty() ? "lease_denied" : result.reason,
                          result.current_lease.has_value() ? result.current_lease->holder_app_id :
        std::string());
  }
  return true;
}

ControlLeaseManager::LeaseRequestResult ControlLeaseManager::apply_acquire_request_locked(
  int client_fd,
  const std::string & request_id,
  const std::string & robot_name,
  const std::string & app_id,
  const std::string & role,
  const std::string & session_id,
  bool panel_open,
  bool teleop_active,
  bool task_active,
  const std::string & task_kind,
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
    lease.authority_generation = new_authority_generation();
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

  RobotControlLease & lease = it->second;
  if (lease.holder_client_fd == client_fd) {
    if ((!app_id.empty() && app_id != lease.holder_app_id) ||
      (!session_id.empty() && session_id != lease.session_id))
    {
      lease.authority_generation = new_authority_generation();
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
    result.already_owned_by_requester = true;
    result.current_lease = lease;
    publish_snapshot_locked("lease_updated", request_id, robot_name, std::string(),
        lease.holder_app_id);
    return result;
  }

  if (lease_usage_active(lease)) {
    result.granted = false;
    result.reason = "robot_control_leased";
    result.current_lease = lease;
    return result;
  }

  // The existing holder is inactive (no panel/task/teleop usage), so reassign immediately.
  lease.holder_client_fd = client_fd;
  lease.holder_app_id = app_id;
  lease.holder_role = role;
  lease.session_id = session_id;
  lease.acquired_at_ms = now;
  lease.authority_generation = new_authority_generation();
  lease.last_heartbeat_ms = now;
  lease.panel_open = panel_open;
  lease.teleop_active = teleop_active;
  lease.task_active = task_active;
  lease.task_kind = task_kind.empty() ? "none" : task_kind;
  lease.lease_version = next_lease_version_++;
  result.granted = true;
  result.current_lease = lease;
  publish_snapshot_locked("lease_reassigned_inactive", request_id, robot_name, std::string(),
      lease.holder_app_id);
  return result;
}

ControlLeaseManager::LeaseRequestResult ControlLeaseManager::apply_heartbeat_request_locked(
  int client_fd,
  const std::string & request_id,
  const std::string & robot_name,
  const std::string & app_id,
  const std::string & role,
  const std::string & session_id,
  bool panel_open,
  bool teleop_active,
  bool task_active,
  const std::string & task_kind,
  uint64_t now)
{
  auto it = leases_by_robot_.find(robot_name);
  if (it == leases_by_robot_.end()) {
    // Heartbeat can create/refresh an implicit lease if the client already transitioned into control.
    return apply_acquire_request_locked(
      client_fd, request_id, robot_name, app_id, role, session_id, panel_open, teleop_active,
        task_active, task_kind, now);
  }

  LeaseRequestResult result;
  RobotControlLease & lease = it->second;
  if (lease.holder_client_fd != client_fd) {
    result.granted = false;
    result.reason = "heartbeat_not_owner";
    result.current_lease = lease;
    return result;
  }

  if ((!app_id.empty() && app_id != lease.holder_app_id) ||
    (!session_id.empty() && session_id != lease.session_id))
  {
    lease.authority_generation = new_authority_generation();
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
  publish_snapshot_locked("lease_updated", request_id, robot_name, std::string(),
      lease.holder_app_id);
  return result;
}

ControlLeaseManager::LeaseRequestResult ControlLeaseManager::apply_release_request_locked(
  int client_fd,
  const std::string & request_id,
  const std::string & robot_name,
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

void ControlLeaseManager::publish_snapshot_locked(
  const std::string & event_name,
  const std::string & request_id,
  const std::string & robot_name,
  const std::string & denied_reason,
  const std::string & holder_app_id)
{
  if (!lease_state_pub_ && !snapshot_sink_) {
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
  for (const auto & pair : leases_by_robot_) {
    const RobotControlLease & lease = pair.second;
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
    item["authority_generation"] = lease.authority_generation;
    const uint64_t age = now_ms() - lease.last_heartbeat_ms;
    item["remaining_ms"] = age < lease_ttl_ms_ ? lease_ttl_ms_ - age : 0;
    leases.push_back(item);
  }
  root["leases"] = leases;

  std_msgs::msg::String msg;
  msg.data = root.dump();
  if (snapshot_sink_) {snapshot_sink_(msg.data);}
  if (lease_state_pub_) {lease_state_pub_->publish(msg);}
}

void ControlLeaseManager::publish_denied_locked(
  const std::string & request_id,
  const std::string & robot_name,
  const std::string & denied_reason,
  const std::string & holder_app_id)
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
    const uint64_t age_ms = (now >=
      it->second.last_heartbeat_ms) ? (now - it->second.last_heartbeat_ms) : 0;
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

bool ControlLeaseManager::should_publish_denied_event_locked(
  int client_fd,
  const std::string & topic, uint64_t now)
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

bool ControlLeaseManager::lease_usage_active(const RobotControlLease & lease)
{
  return lease.panel_open || lease.teleop_active || lease.task_active;
}

bool ControlLeaseManager::try_decode_string_msg(
  const std::vector<uint8_t> & serialized_payload,
  std::string & out_data)
{
  try {
    std::vector<uint8_t> normalized_payload;
    const std::vector<uint8_t> * payload = &serialized_payload;
    if (!detail::has_cdr_header(serialized_payload)) {
      normalized_payload = detail::add_cdr_header(serialized_payload);
      payload = &normalized_payload;
    }

    rclcpp::SerializedMessage serialized;
    detail::fill_serialized_message(*payload, serialized);

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
