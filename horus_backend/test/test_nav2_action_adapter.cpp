// SPDX-FileCopyrightText: 2026 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#include "horus_backend/nav2_action_adapter.hpp"
#include <gtest/gtest.h>
#include <functional>
#include <thread>

namespace
{
using Action = nav2_msgs::action::NavigateToPose;
using ServerGoal = rclcpp_action::ServerGoalHandle<Action>;
using namespace std::chrono_literals;

class Nav2AdapterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    bridge = std::make_shared<rclcpp::Node>("nav2_adapter_test_bridge");
    source = std::make_shared<rclcpp::Node>("nav2_adapter_test_source");
    server_node = std::make_shared<rclcpp::Node>("nav2_adapter_test_server");
    create_server();
    adapter = std::make_unique<horus_backend::Nav2ActionAdapter>(bridge.get());
    adapter->register_robot("test_id", "test_bot");
    goal_pub = source->create_publisher<geometry_msgs::msg::PoseStamped>("/test_bot/goal_pose", 10);
    cancel_pub = source->create_publisher<std_msgs::msg::String>("/test_bot/goal_cancel", 10);
    status_sub = source->create_subscription<std_msgs::msg::String>(
      "/test_bot/goal_status", 10,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {statuses.push_back(msg->data);});
    feedback_sub = source->create_subscription<Action::Feedback>(
      "/test_bot/goal_status/feedback", 10,
      [this](Action::Feedback::ConstSharedPtr msg) {feedback.push_back(msg->distance_remaining);});
    ASSERT_TRUE(wait([&] {
      return goal_pub->get_subscription_count() > 0 &&
             cancel_pub->get_subscription_count() > 0 &&
             server_node->count_clients("/test_bot/navigate_to_pose/_action/send_goal") > 0;
    }));
    pump(false, 150ms);
  }

  void create_server()
  {
    server = rclcpp_action::create_server<Action>(
      server_node, "/test_bot/navigate_to_pose",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>) {
        return accept_goals ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
               rclcpp_action::GoalResponse::REJECT;
      },
      [](std::shared_ptr<ServerGoal>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<ServerGoal> goal) {goals.push_back(goal);});
  }

  void pump(bool include_server, std::chrono::milliseconds duration)
  {
    const auto end = std::chrono::steady_clock::now() + duration;
    do {
      if (include_server) {rclcpp::spin_some(server_node);}
      rclcpp::spin_some(bridge);
      rclcpp::spin_some(source);
      std::this_thread::sleep_for(2ms);
    } while (std::chrono::steady_clock::now() < end);
  }

  bool wait(const std::function<bool()> & condition)
  {
    const auto end = std::chrono::steady_clock::now() + 3s;
    while (!condition() && std::chrono::steady_clock::now() < end) {
      pump(true, 5ms);
    }
    return condition();
  }

  void send_goal(double x)
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "map";
    msg.pose.position.x = x;
    msg.pose.orientation.w = 1;
    goal_pub->publish(msg);
  }
  void cancel()
  {
    std_msgs::msg::String msg;
    msg.data = "cancel";
    cancel_pub->publish(msg);
  }
  std::shared_ptr<rclcpp::Node> bridge, source, server_node;
  rclcpp_action::Server<Action>::SharedPtr server;
  std::unique_ptr<horus_backend::Nav2ActionAdapter> adapter;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cancel_pub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub;
  rclcpp::Subscription<Action::Feedback>::SharedPtr feedback_sub;
  std::vector<std::shared_ptr<ServerGoal>> goals;
  std::vector<std::string> statuses;
  std::vector<float> feedback;
  bool accept_goals = true;
};

TEST_F(Nav2AdapterTest, CancelBeforeAcceptanceWaitsForConfirmedCancellation)
{
  send_goal(1);
  pump(false, 100ms);
  cancel();
  pump(false, 100ms);
  EXPECT_TRUE(goals.empty());
  EXPECT_TRUE(statuses.empty());
  ASSERT_TRUE(wait([&] {return goals.size() == 1 && goals[0]->is_canceling();}));
  EXPECT_TRUE(statuses.empty());
  goals[0]->canceled(std::make_shared<Action::Result>());
  ASSERT_TRUE(wait([&] {return !statuses.empty();}));
  EXPECT_EQ(statuses, (std::vector<std::string>{"goal_cancelled"}));
}

TEST_F(Nav2AdapterTest, OldResultsCannotClearNewGoalAndFeedbackIsForwarded)
{
  send_goal(1);
  ASSERT_TRUE(wait([&] {return goals.size() == 1;}));
  pump(true, 50ms);
  send_goal(2);
  ASSERT_TRUE(wait([&] {return goals.size() == 2 && goals[0]->is_canceling();}));
  pump(true, 50ms);
  goals[0]->canceled(std::make_shared<Action::Result>());
  pump(true, 50ms);
  EXPECT_TRUE(statuses.empty());
  auto sample = std::make_shared<Action::Feedback>();
  sample->distance_remaining = 12.5f;
  goals[1]->publish_feedback(sample);
  ASSERT_TRUE(wait([&] {return !feedback.empty();}));
  EXPECT_FLOAT_EQ(feedback.back(), 12.5f);
  cancel();
  ASSERT_TRUE(wait([&] {return goals[1]->is_canceling();}));
  goals[1]->canceled(std::make_shared<Action::Result>());
  ASSERT_TRUE(wait([&] {return !statuses.empty();}));
  EXPECT_EQ(statuses, (std::vector<std::string>{"goal_cancelled"}));
}

TEST_F(Nav2AdapterTest, UnregisterCancelsGoalAcceptedAfterTeardown)
{
  send_goal(1);
  pump(false, 100ms);
  adapter->unregister_robot("test_id");
  adapter.reset();
  ASSERT_TRUE(wait([&] {return goals.size() == 1 && goals[0]->is_canceling();}));
  goals[0]->canceled(std::make_shared<Action::Result>());
  pump(true, 50ms);
  EXPECT_TRUE(statuses.empty());
}

TEST_F(Nav2AdapterTest, RejectedGoalDoesNotPreventNextGoalFromCompleting)
{
  accept_goals = false;
  send_goal(1);
  ASSERT_TRUE(wait([&] {return !statuses.empty();}));
  EXPECT_EQ(statuses.back(), "goal_failed");
  EXPECT_TRUE(goals.empty());
  accept_goals = true;
  send_goal(2);
  ASSERT_TRUE(wait([&] {return goals.size() == 1;}));
  goals[0]->succeed(std::make_shared<Action::Result>());
  ASSERT_TRUE(wait([&] {return statuses.size() == 2;}));
  EXPECT_EQ(statuses.back(), "goal_reached");
}

TEST_F(Nav2AdapterTest, UnavailableServerDoesNotBlockAndCanReconnect)
{
  server.reset();
  ASSERT_TRUE(wait([&] {
    return bridge->count_services("/test_bot/navigate_to_pose/_action/send_goal") == 0;
  }));
  send_goal(1);
  ASSERT_TRUE(wait([&] {return !statuses.empty();}));
  EXPECT_EQ(statuses.back(), "goal_failed");
  create_server();
  ASSERT_TRUE(wait([&] {
    return bridge->count_services("/test_bot/navigate_to_pose/_action/send_goal") == 1;
  }));
  pump(true, 150ms);
  send_goal(2);
  ASSERT_TRUE(wait([&] {return goals.size() == 1;}));
  goals[0]->succeed(std::make_shared<Action::Result>());
  ASSERT_TRUE(wait([&] {return statuses.size() == 2;}));
  EXPECT_EQ(statuses.back(), "goal_reached");
}
}  // namespace
