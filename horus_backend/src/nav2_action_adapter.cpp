// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#include "horus_backend/nav2_action_adapter.hpp"

namespace horus_backend
{

Nav2ActionAdapter::Nav2ActionAdapter(rclcpp::Node * node)
: node_(node)
{
}

Nav2ActionAdapter::~Nav2ActionAdapter()
{
  std::lock_guard<std::mutex> lock(adapters_mutex_);
  for (const auto & [id, adapter] : adapters_) {
    retire_adapter(adapter);
  }
  adapters_.clear();
}

void Nav2ActionAdapter::register_robot(
  const std::string & robot_id,
  const std::string & robot_name,
  const std::string & action_topic,
  const std::string & goal_topic,
  const std::string & cancel_topic,
  const std::string & status_topic)
{
  std::lock_guard<std::mutex> lock(adapters_mutex_);

  if (adapters_.find(robot_id) != adapters_.end()) {
    // Already registered
    return;
  }

  auto adapter = std::make_shared<RobotAdapter>();
  adapter->robot_id = robot_id;
  adapter->robot_name = robot_name;
  adapter->action_topic = action_topic;
  adapter->goal_topic = goal_topic;
  adapter->cancel_topic = cancel_topic;
  adapter->status_topic = status_topic;

  // Setup the action client and subscriptions
  setup_adapter(adapter);

  adapters_[robot_id] = adapter;

  RCLCPP_INFO(node_->get_logger(), "Nav2 Action Adapter registered for robot: %s (%s)",
      robot_name.c_str(), robot_id.c_str());
}

void Nav2ActionAdapter::unregister_robot(const std::string & robot_id)
{
  std::lock_guard<std::mutex> lock(adapters_mutex_);

  auto it = adapters_.find(robot_id);
  if (it != adapters_.end()) {
    retire_adapter(it->second);
    adapters_.erase(it);
    RCLCPP_INFO(node_->get_logger(), "Nav2 Action Adapter unregistered for robot: %s",
        robot_id.c_str());
  }
}

void Nav2ActionAdapter::setup_adapter(std::shared_ptr<RobotAdapter> adapter)
{
  // Create action client for navigate_to_pose
  // Topic: /<robot_name>/navigate_to_pose
  std::string action_topic = adapter->action_topic.empty() ?
    "/" + adapter->robot_name + "/navigate_to_pose" :
    adapter->action_topic;
  adapter->action_client = rclcpp_action::create_client<NavigateToPose>(
    node_,
    action_topic);

  // Create publisher for goal status
  // Topic: /<robot_name>/goal_status
  std::string status_topic = adapter->status_topic.empty() ?
    "/" + adapter->robot_name + "/goal_status" :
    adapter->status_topic;
  adapter->status_pub = node_->create_publisher<std_msgs::msg::String>(
    status_topic, 10);
  adapter->feedback_pub = node_->create_publisher<NavigateToPose::Feedback>(
    status_topic + "/feedback", rclcpp::QoS(1));
  std::weak_ptr<RobotAdapter> weak = adapter;

  // Create subscriber for goal pose
  // Topic: /<robot_name>/goal_pose
  std::string goal_topic = adapter->goal_topic.empty() ?
    "/" + adapter->robot_name + "/goal_pose" :
    adapter->goal_topic;
  adapter->goal_sub = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_topic, 10,
    [weak](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      if (auto adapter = weak.lock()) {
        handle_goal(adapter, msg);
      }
    });

  // Create subscriber for goal cancel
  // Topic: /<robot_name>/goal_cancel
  std::string cancel_topic = adapter->cancel_topic.empty() ?
    "/" + adapter->robot_name + "/goal_cancel" :
    adapter->cancel_topic;
  adapter->cancel_sub = node_->create_subscription<std_msgs::msg::String>(
    cancel_topic, 10,
    [weak](const std_msgs::msg::String::SharedPtr msg) {
      if (auto adapter = weak.lock()) {
        handle_cancel(adapter, msg);
      }
    });
}

void Nav2ActionAdapter::retire_adapter(const std::shared_ptr<RobotAdapter> & adapter)
{
  std::lock_guard<std::mutex> lock(adapter->goal_mutex);
  adapter->retired = true;
  adapter->cancel_requested = true;
  adapter->goal_sub.reset();
  adapter->cancel_sub.reset();
  cancel_active_goal(adapter);
}

void Nav2ActionAdapter::cancel_active_goal(const std::shared_ptr<RobotAdapter> & adapter)
{
  if (!adapter->active_goal_handle) {
    return;
  }
  try {
    adapter->action_client->async_cancel_goal(adapter->active_goal_handle);
  } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
    // A terminal result may already be queued; it supplies the final state.
  }
}

void Nav2ActionAdapter::handle_goal(
  std::shared_ptr<RobotAdapter> adapter,
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(adapter->goal_mutex);
  if (adapter->retired || !msg) {
    return;
  }
  if (!adapter->action_client->action_server_is_ready()) {
    publish_status(adapter, "goal_failed");
    return;
  }
  cancel_active_goal(adapter);
  const auto generation = ++adapter->generation;
  adapter->active_goal_handle.reset();
  adapter->awaiting_acceptance = true;
  adapter->cancel_requested = false;
  NavigateToPose::Goal goal;
  goal.pose = *msg;
  rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
  // Retain a pending adapter until acceptance, so unregister can still cancel it.
  options.goal_response_callback = [adapter, generation](const GoalHandleNav2::SharedPtr & handle) {
    goal_response_callback(adapter, generation, handle);
  };
  std::weak_ptr<RobotAdapter> weak = adapter;
  options.feedback_callback = [weak, generation](
    GoalHandleNav2::SharedPtr handle, const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
    if (auto adapter = weak.lock()) {
      feedback_callback(adapter, generation, handle, feedback);
    }
  };
  options.result_callback = [weak, generation](const GoalHandleNav2::WrappedResult & result) {
    if (auto adapter = weak.lock()) {
      result_callback(adapter, generation, result);
    }
  };
  try {
    adapter->action_client->async_send_goal(goal, options);
  } catch (const std::exception &) {
    adapter->awaiting_acceptance = false;
    publish_status(adapter, "goal_failed");
  }
}

void Nav2ActionAdapter::handle_cancel(
  std::shared_ptr<RobotAdapter> adapter,
  const std_msgs::msg::String::SharedPtr msg)
{
  if (msg && !msg->data.empty() &&
    msg->data != "cancel" && msg->data != "CANCEL" && msg->data != "Cancel")
  {
    return;
  }
  std::lock_guard<std::mutex> lock(adapter->goal_mutex);
  if (adapter->retired || (!adapter->awaiting_acceptance && !adapter->active_goal_handle)) {
    return;
  }
  // Cancellation before acceptance must wait for the goal handle. Do not report
  // success until Nav2 confirms a terminal CANCELED result.
  adapter->cancel_requested = true;
  cancel_active_goal(adapter);
}

void Nav2ActionAdapter::goal_response_callback(
  std::shared_ptr<RobotAdapter> adapter,
  uint64_t generation,
  const GoalHandleNav2::SharedPtr & goal_handle)
{
  std::lock_guard<std::mutex> lock(adapter->goal_mutex);
  if (adapter->retired || generation != adapter->generation) {
    if (goal_handle) {
      try {
        adapter->action_client->async_cancel_goal(goal_handle);
      } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
      }
    }
    return;
  }
  adapter->awaiting_acceptance = false;
  adapter->active_goal_handle = goal_handle;
  if (!goal_handle) {
    publish_status(adapter, "goal_failed");
  } else if (adapter->cancel_requested) {
    cancel_active_goal(adapter);
  }
}

void Nav2ActionAdapter::feedback_callback(
  std::shared_ptr<RobotAdapter> adapter,
  uint64_t generation,
  GoalHandleNav2::SharedPtr goal_handle,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  std::lock_guard<std::mutex> lock(adapter->goal_mutex);
  if (!adapter->retired && generation == adapter->generation && feedback &&
    adapter->active_goal_handle == goal_handle)
  {
    adapter->feedback_pub->publish(*feedback);
  }
}

void Nav2ActionAdapter::result_callback(
  std::shared_ptr<RobotAdapter> adapter,
  uint64_t generation,
  const GoalHandleNav2::WrappedResult & result)
{
  std::lock_guard<std::mutex> lock(adapter->goal_mutex);
  if (adapter->retired || generation != adapter->generation) {
    return;
  }
  adapter->active_goal_handle.reset();
  adapter->awaiting_acceptance = false;
  adapter->cancel_requested = false;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      publish_status(adapter, "goal_reached");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      publish_status(adapter, "goal_failed");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      publish_status(adapter, "goal_cancelled");
      break;
    default:
      publish_status(adapter, "goal_unknown");
      break;
  }
}

void Nav2ActionAdapter::publish_status(
  std::shared_ptr<RobotAdapter> adapter,
  const std::string & status)
{
  if (adapter->status_pub) {
    std_msgs::msg::String msg;
    msg.data = status;
    adapter->status_pub->publish(msg);
  }
}

}  // namespace horus_backend
