// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace horus_unity_bridge
{

class ControlLeaseManager
{
public:
  explicit ControlLeaseManager(rclcpp::Node* node);
  ~ControlLeaseManager() = default;

  bool is_internal_topic(const std::string& topic) const;
  bool handle_internal_message(int client_fd,
                               const std::string& topic,
                               const std::vector<uint8_t>& serialized_payload);

  bool authorize_command_publish(int client_fd,
                                 const std::string& topic,
                                 std::string* denied_reason = nullptr,
                                 std::string* robot_name = nullptr);

  void on_client_disconnected(int client_fd);

  static const std::string& lease_request_topic();
  static const std::string& control_topic_catalog_topic();
  static const std::string& lease_state_topic();

private:
  struct RobotControlLease {
    std::string robot_name;
    int holder_client_fd = -1;
    std::string holder_app_id;
    std::string holder_role;
    std::string session_id;
    uint64_t acquired_at_ms = 0;
    uint64_t last_heartbeat_ms = 0;
    bool panel_open = false;
    bool teleop_active = false;
    bool task_active = false;
    std::string task_kind = "none";
    uint64_t lease_version = 0;
  };

  struct LeaseRequestResult {
    bool granted = false;
    std::string reason;
    bool already_owned_by_requester = false;
    std::optional<RobotControlLease> current_lease;
  };

  struct DeniedRateLimitKey {
    int client_fd = -1;
    std::string topic;

    bool operator==(const DeniedRateLimitKey& other) const
    {
      return client_fd == other.client_fd && topic == other.topic;
    }
  };

  struct DeniedRateLimitKeyHash {
    std::size_t operator()(const DeniedRateLimitKey& value) const noexcept;
  };

  rclcpp::Node* node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lease_state_pub_;
  rclcpp::TimerBase::SharedPtr lease_expiry_timer_;

  mutable std::mutex mutex_;
  std::unordered_map<std::string, RobotControlLease> leases_by_robot_;
  std::unordered_map<std::string, std::string> protected_topic_to_robot_;
  std::unordered_map<DeniedRateLimitKey, uint64_t, DeniedRateLimitKeyHash> denied_publish_event_last_ms_;

  uint64_t next_lease_version_ = 1;
  std::string current_session_id_;
  uint64_t lease_ttl_ms_ = 3000;
  uint64_t denied_event_rate_limit_ms_ = 500;

  bool handle_catalog_message_locked(int client_fd, const std::string& json_payload);
  bool handle_lease_request_message_locked(int client_fd, const std::string& json_payload);

  LeaseRequestResult apply_acquire_request_locked(int client_fd,
                                                  const std::string& request_id,
                                                  const std::string& robot_name,
                                                  const std::string& app_id,
                                                  const std::string& role,
                                                  const std::string& session_id,
                                                  bool panel_open,
                                                  bool teleop_active,
                                                  bool task_active,
                                                  const std::string& task_kind,
                                                  uint64_t now_ms);
  LeaseRequestResult apply_heartbeat_request_locked(int client_fd,
                                                    const std::string& request_id,
                                                    const std::string& robot_name,
                                                    const std::string& app_id,
                                                    const std::string& role,
                                                    const std::string& session_id,
                                                    bool panel_open,
                                                    bool teleop_active,
                                                    bool task_active,
                                                    const std::string& task_kind,
                                                    uint64_t now_ms);
  LeaseRequestResult apply_release_request_locked(int client_fd,
                                                  const std::string& request_id,
                                                  const std::string& robot_name,
                                                  uint64_t now_ms);

  void publish_snapshot_locked(const std::string& event_name,
                               const std::string& request_id = std::string(),
                               const std::string& robot_name = std::string(),
                               const std::string& denied_reason = std::string(),
                               const std::string& holder_app_id = std::string());
  void publish_denied_locked(const std::string& request_id,
                             const std::string& robot_name,
                             const std::string& denied_reason,
                             const std::string& holder_app_id);

  void cleanup_expired_leases();
  void cleanup_expired_leases_locked(uint64_t now_ms, bool publish_if_changed);
  bool should_publish_denied_event_locked(int client_fd, const std::string& topic, uint64_t now_ms);

  static uint64_t now_ms();
  static bool try_decode_string_msg(const std::vector<uint8_t>& serialized_payload, std::string& out_data);
};

} // namespace horus_unity_bridge

