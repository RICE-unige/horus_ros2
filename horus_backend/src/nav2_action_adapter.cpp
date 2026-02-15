// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#include "horus_backend/nav2_action_adapter.hpp"

namespace horus_backend
{

Nav2ActionAdapter::Nav2ActionAdapter(rclcpp::Node* node)
: node_(node)
{
}

Nav2ActionAdapter::~Nav2ActionAdapter()
{
  std::lock_guard<std::mutex> lock(adapters_mutex_);
  adapters_.clear();
}

void Nav2ActionAdapter::register_robot(const std::string& robot_id, const std::string& robot_name)
{
  std::lock_guard<std::mutex> lock(adapters_mutex_);
  
  if (adapters_.find(robot_id) != adapters_.end()) {
    // Already registered
    return;
  }
  
  auto adapter = std::make_shared<RobotAdapter>();
  adapter->robot_id = robot_id;
  adapter->robot_name = robot_name;
  
  // Setup the action client and subscriptions
  setup_adapter(adapter);
  
  adapters_[robot_id] = adapter;
  
  RCLCPP_INFO(node_->get_logger(), "Nav2 Action Adapter registered for robot: %s (%s)", robot_name.c_str(), robot_id.c_str());
}

void Nav2ActionAdapter::unregister_robot(const std::string& robot_id)
{
  std::lock_guard<std::mutex> lock(adapters_mutex_);
  
  auto it = adapters_.find(robot_id);
  if (it != adapters_.end()) {
    adapters_.erase(it);
    RCLCPP_INFO(node_->get_logger(), "Nav2 Action Adapter unregistered for robot: %s", robot_id.c_str());
  }
}

void Nav2ActionAdapter::setup_adapter(std::shared_ptr<RobotAdapter> adapter)
{
  // Create action client for navigate_to_pose
  // Topic: /<robot_name>/navigate_to_pose
  std::string action_topic = "/" + adapter->robot_name + "/navigate_to_pose";
  adapter->action_client = rclcpp_action::create_client<NavigateToPose>(
    node_,
    action_topic);
    
  // Create publisher for goal status
  // Topic: /<robot_name>/goal_status
  std::string status_topic = "/" + adapter->robot_name + "/goal_status";
  adapter->status_pub = node_->create_publisher<std_msgs::msg::String>(
    status_topic, 10);
    
  // Create subscriber for goal pose
  // Topic: /<robot_name>/goal_pose
  std::string goal_topic = "/" + adapter->robot_name + "/goal_pose";
  adapter->goal_sub = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_topic, 10,
    [this, adapter](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      handle_goal(adapter, msg);
    });
    
  // Create subscriber for goal cancel
  // Topic: /<robot_name>/goal_cancel
  std::string cancel_topic = "/" + adapter->robot_name + "/goal_cancel";
  adapter->cancel_sub = node_->create_subscription<std_msgs::msg::String>(
    cancel_topic, 10,
    [this, adapter](const std_msgs::msg::String::SharedPtr msg) {
      handle_cancel(adapter, msg);
    });
}

void Nav2ActionAdapter::handle_goal(
  std::shared_ptr<RobotAdapter> adapter,
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!adapter->action_client->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting: %s", adapter->robot_id.c_str());
    publish_status(adapter, "goal_failed");
    return;
  }
  
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose = *msg;
  // Use behavior_tree field if needed, currently empty
  
  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  
  send_goal_options.goal_response_callback =
    [this, adapter](const GoalHandleNav2::SharedPtr& goal_handle) {
      goal_response_callback(adapter, goal_handle);
    };
    
  send_goal_options.feedback_callback =
    [this, adapter](
      GoalHandleNav2::SharedPtr goal_handle,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
      feedback_callback(adapter, goal_handle, feedback);
    };
    
  send_goal_options.result_callback =
    [this, adapter](const GoalHandleNav2::WrappedResult& result) {
      result_callback(adapter, result);
    };
    
  // Send the goal
  RCLCPP_INFO(node_->get_logger(), "Sending goal to robot: %s", adapter->robot_id.c_str());
  
  // Clean up previous goal if any? 
  // Nav2 typically handles preemption, but we should track the handle.
  
  auto goal_handle_future = adapter->action_client->async_send_goal(goal_msg, send_goal_options);
  
  std::lock_guard<std::mutex> lock(adapter->goal_mutex);
  adapter->future_goal_handle = goal_handle_future;
  // Note: active_goal_handle will be set in response callback
}

void Nav2ActionAdapter::handle_cancel(
  std::shared_ptr<RobotAdapter> adapter,
  const std_msgs::msg::String::SharedPtr msg)
{
  if (msg && !msg->data.empty() &&
    msg->data != "cancel" && msg->data != "CANCEL" && msg->data != "Cancel")
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "Ignoring unsupported cancel payload for robot %s: '%s'",
      adapter->robot_id.c_str(),
      msg->data.c_str());
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "Received cancel request for robot: %s", adapter->robot_id.c_str());
  
  std::lock_guard<std::mutex> lock(adapter->goal_mutex);
  if (adapter->active_goal_handle) {
    auto future_cancel = adapter->action_client->async_cancel_goal(adapter->active_goal_handle);
    // We could attach a callback to future_cancel if needed
    RCLCPP_INFO(node_->get_logger(), "Sent cancel request to Nav2 for robot: %s", adapter->robot_id.c_str());
  } else {
    RCLCPP_WARN(node_->get_logger(), "No active goal to cancel for robot: %s", adapter->robot_id.c_str());
    // Publish cancelled anyway to acknowledge the request if the robot is idle
    publish_status(adapter, "goal_cancelled");
  }
}

void Nav2ActionAdapter::goal_response_callback(
  std::shared_ptr<RobotAdapter> adapter,
  const GoalHandleNav2::SharedPtr& goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server for robot: %s", adapter->robot_id.c_str());
    publish_status(adapter, "goal_failed");
    return;
  }
  
  std::lock_guard<std::mutex> lock(adapter->goal_mutex);
  adapter->active_goal_handle = goal_handle;
  RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result for robot: %s", adapter->robot_id.c_str());
}

void Nav2ActionAdapter::feedback_callback(
  std::shared_ptr<RobotAdapter> /*adapter*/,
  GoalHandleNav2::SharedPtr /*goal_handle*/,
  const std::shared_ptr<const NavigateToPose::Feedback> /*feedback*/)
{
  // Optional: forward feedback if Unity needs it
  // For now, we only care about terminal implementation for cancel
}

void Nav2ActionAdapter::result_callback(
  std::shared_ptr<RobotAdapter> adapter,
  const GoalHandleNav2::WrappedResult& result)
{
  std::string status_str;
  
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      status_str = "goal_reached"; // Matches Unity's expected string
      RCLCPP_INFO(node_->get_logger(), "Goal succeeded for robot: %s", adapter->robot_id.c_str());
      break;
    case rclcpp_action::ResultCode::ABORTED:
      status_str = "goal_failed";
      RCLCPP_ERROR(node_->get_logger(), "Goal was aborted for robot: %s", adapter->robot_id.c_str());
      break;
    case rclcpp_action::ResultCode::CANCELED:
      status_str = "goal_cancelled"; // Matches Unity's expected string
      RCLCPP_INFO(node_->get_logger(), "Goal was canceled for robot: %s", adapter->robot_id.c_str());
      break;
    default:
      status_str = "goal_unknown";
      RCLCPP_ERROR(node_->get_logger(), "Unknown result code for robot: %s", adapter->robot_id.c_str());
      break;
  }
  
  {
    std::lock_guard<std::mutex> lock(adapter->goal_mutex);
    adapter->active_goal_handle.reset();
  }

  publish_status(adapter, status_str);
}

void Nav2ActionAdapter::publish_status(std::shared_ptr<RobotAdapter> adapter, const std::string& status)
{
  if (adapter->status_pub) {
    std_msgs::msg::String msg;
    msg.data = status;
    adapter->status_pub->publish(msg);
  }
}

} // namespace horus_backend
