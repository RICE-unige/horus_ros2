// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#ifndef HORUS_BACKEND__NAV2_ACTION_ADAPTER_HPP_
#define HORUS_BACKEND__NAV2_ACTION_ADAPTER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <map>
#include <memory>
#include <string>
#include <mutex>

namespace horus_backend
{

class Nav2ActionAdapter
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav2 = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit Nav2ActionAdapter(rclcpp::Node* node);
  ~Nav2ActionAdapter();

  /**
   * @brief Call this when a new robot is registered to set up its action client adapter.
   */
  void register_robot(const std::string& robot_id, const std::string& robot_name);

  /**
   * @brief Call this when a robot is unregistered to clean up.
   */
  void unregister_robot(const std::string& robot_id);

private:
  struct RobotAdapter
  {
    std::string robot_id;
    std::string robot_name;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cancel_sub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub;
    
    // Track current goal to allow cancellation
    std::shared_future<GoalHandleNav2::SharedPtr> future_goal_handle;
    GoalHandleNav2::SharedPtr active_goal_handle;
    std::mutex goal_mutex;
  };

  rclcpp::Node* node_;
  std::map<std::string, std::shared_ptr<RobotAdapter>> adapters_;
  std::mutex adapters_mutex_;

  void setup_adapter(std::shared_ptr<RobotAdapter> adapter);
  void handle_goal(std::shared_ptr<RobotAdapter> adapter, const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void handle_cancel(std::shared_ptr<RobotAdapter> adapter, const std_msgs::msg::String::SharedPtr msg);
  
  void goal_response_callback(
    std::shared_ptr<RobotAdapter> adapter,
    const GoalHandleNav2::SharedPtr& goal_handle);
    
  void feedback_callback(
    std::shared_ptr<RobotAdapter> adapter,
    GoalHandleNav2::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    
  void result_callback(
    std::shared_ptr<RobotAdapter> adapter,
    const GoalHandleNav2::WrappedResult& result);

  void publish_status(std::shared_ptr<RobotAdapter> adapter, const std::string& status);
};

} // namespace horus_backend

#endif // HORUS_BACKEND__NAV2_ACTION_ADAPTER_HPP_
