// Copyright 2026 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0
#include "horus_unity_bridge/control_deadline.hpp"
#include "horus_unity_bridge/control_link.hpp"
#include "horus_unity_bridge/control_lease_manager.hpp"
#include "horus_unity_bridge/topic_manager.hpp"
#include "horus_unity_bridge/message_router.hpp"
#include <gtest/gtest.h>
#include <rclcpp/serialization.hpp>
#include <std_msgs/msg/string.hpp>
#include <cstring>
#include <thread>

namespace horus_unity_bridge
{
TEST(ControlClock, RejectsExpiredClockAndConservativelyBudgetsAge)
{
  control::ClockEstimate clock;
  EXPECT_FALSE(clock.valid(100));
  ASSERT_TRUE(clock.observe(100, 140, 1020));
  EXPECT_EQ(clock.offset_ms, 900);
  EXPECT_EQ(clock.uncertainty_ms, 20);
  EXPECT_EQ(clock.deadline(150, 200), 1230);
  EXPECT_FALSE(clock.valid(4000));
  EXPECT_FALSE(clock.observe(200, 100, 0));
}

TEST(ControlDeadline, RejectsDuplicatesReorderingAndExpiredMotion)
{
  control::SequenceDeadline gate;
  EXPECT_TRUE(gate.accept(1, 1200, 1000));
  EXPECT_FALSE(gate.accept(1, 1200, 1000));
  EXPECT_TRUE(gate.accept(4, 1200, 1000));
  EXPECT_FALSE(gate.accept(3, 1200, 1000));
  EXPECT_FALSE(gate.accept(5, 999, 1000));
  EXPECT_FALSE(gate.accept(4, 1200, 1000));
  EXPECT_FALSE(gate.accept(6, 9999, 1000));
}

TEST(ControlSnapshots, LocalLeaseDeliveryDoesNotCreateDdsRoutes)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  auto node = std::make_shared<rclcpp::Node>("control_snapshot_test");
  TopicManager topics(node.get());
  const std::string topic = ControlLeaseManager::lease_state_topic();
  const std::vector<uint8_t> body{8, 0, 0, 0, 's', 't', 'a', 't', 'e', 0, 0, 0};
  topics.publish_inprocess_snapshot(topic, body);
  EXPECT_EQ(topics.get_retained_payloads(topic), std::vector<std::vector<uint8_t>>{body});
  std::vector<uint8_t> received;
  ASSERT_TRUE(topics.register_inprocess_subscriber(topic, "std_msgs/msg/String", 42,
      [&](const auto &, const auto & data) {received = data;}));
  topics.publish_inprocess_snapshot(topic, body);
  EXPECT_EQ(received, body);
  EXPECT_EQ(node->count_publishers(topic), 0U);
  EXPECT_EQ(node->count_subscribers(topic), 0U);
  EXPECT_EQ(topics.unregister_all_client_subscribers(42), 1U);
}

namespace
{
std::vector<uint8_t> string_payload(const nlohmann::json & json)
{
  std_msgs::msg::String message; message.data = json.dump();
  rclcpp::SerializedMessage serialized;
  rclcpp::Serialization<std_msgs::msg::String>().serialize_message(&message, &serialized);
  const auto & raw = serialized.get_rcl_serialized_message();
  return std::vector<uint8_t>(raw.buffer + 4, raw.buffer + raw.buffer_length);
}
}

TEST(ControlCatalog, PrivateSnapshotReplacesOnlyItsOwnContribution)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  auto node = std::make_shared<rclcpp::Node>("private_catalog_test");
  ControlLeaseManager manager(node.get());
  auto catalog = [&](int client, const std::string & role, const std::string & robot, const std::string & topic) {
      return manager.handle_internal_message(client, ControlLeaseManager::control_topic_catalog_topic(),
        string_payload({{"role", role}, {"op", role == "host" ? "snapshot" : "upsert"},
          {"replace_client_catalog", true}, {"robots", {{{"robot_name", robot}, {"protected_topics", {topic}}}}}}));
    };
  auto acquire = [&](int client, const std::string & robot) {
      return manager.handle_internal_message(client, ControlLeaseManager::lease_request_topic(),
        string_payload({{"action", "acquire"}, {"robot_name", robot}, {"app_id", std::to_string(client)},
          {"role", "private"}, {"teleop_active", true}}));
    };
  ASSERT_TRUE(catalog(10, "host", "host_robot", "/host/goal"));
  ASSERT_TRUE(catalog(20, "private", "private_robot", "/private/goal"));
  ASSERT_TRUE(acquire(10, "host_robot")); ASSERT_TRUE(acquire(20, "private_robot"));
  EXPECT_TRUE(manager.owns_active_remote_lease(10, "/host/goal"));
  EXPECT_TRUE(manager.owns_active_remote_lease(20, "/private/goal"));
  ASSERT_TRUE(catalog(20, "private", "private_robot", "/private/new_goal"));
  EXPECT_FALSE(manager.owns_active_remote_lease(20, "/private/goal"));
  EXPECT_TRUE(manager.owns_active_remote_lease(20, "/private/new_goal"));
  EXPECT_TRUE(manager.owns_active_remote_lease(10, "/host/goal"));
  EXPECT_FALSE(catalog(20, "private", "private_robot", "/host/goal"));
  EXPECT_TRUE(manager.owns_active_remote_lease(20, "/private/new_goal"));
  manager.on_client_disconnected(20);
  EXPECT_TRUE(manager.owns_active_remote_lease(10, "/host/goal"));
}

#ifdef ENABLE_WEBRTC
TEST(ControlLink, ProxyDoesNotCreateCommandPublishersOnOperatorRos)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"control.mode", "proxy"}});
  auto router = std::make_shared<MessageRouter>(options, true);
  horuslink::ChannelDescriptor command;
  command.topic = "/control_proxy_only/cmd_vel"; command.type_name = "geometry_msgs/msg/Twist";
  command.delivery = horuslink::Delivery::ReplaceLatest;
  ASSERT_TRUE(router->register_horuslink_publisher(41, command));
  EXPECT_EQ(router->count_publishers(command.topic), 0U);
  command.topic = ControlLeaseManager::control_topic_catalog_topic(); command.type_name = "std_msgs/msg/String";
  ASSERT_TRUE(router->register_horuslink_publisher(41, command));
  EXPECT_EQ(router->count_publishers(command.topic), 0U);
}

TEST(ControlLink, SharesAuthorityAcrossPeersAndStopsExpiredMotionWithoutVideo)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  auto source = std::make_shared<rclcpp::Node>("control_link_test_source");
  auto first = std::make_shared<rclcpp::Node>("control_link_test_first");
  auto second = std::make_shared<rclcpp::Node>("control_link_test_second");
  ControlLeaseManager leases(source.get());
  std::vector<double> velocities;
  ControlLink authority(source.get(), "authority", [&](int client, const auto & channel,
      const std::vector<uint8_t> & data, uint8_t) {
      if (leases.is_internal_topic(channel.topic)) {return leases.handle_internal_message(client, channel.topic, data);}
      if (!leases.owns_active_remote_lease(client, channel.topic) || data.size() != 48) {return false;}
      double velocity = 0; std::memcpy(&velocity, data.data(), sizeof(double));
      velocities.push_back(velocity); return true;
    }, [&](int client) {leases.on_client_disconnected(client);}, {}, [&]() {return leases.protected_topics();});
  leases.set_snapshot_sink([&](const std::string & state) {authority.publish_lease_state(state);}, false);
  auto unused = [](int, const auto &, const auto &, uint8_t) {return false;};
  ControlLink one(first.get(), "proxy", unused, [](int) {});
  ControlLink two(second.get(), "proxy", unused, [](int) {});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(source); executor.add_node(first); executor.add_node(second);
  auto pump = [&](double seconds) {
      const auto until = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
      do {
        executor.spin_some(); one.tick(); two.tick(); authority.tick();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      } while (std::chrono::steady_clock::now() < until);
    };
  bool connected = false;
  for (int retry = 0; retry < 100; ++retry) {
    pump(0.1);
    const auto a = one.statistics()["links"], b = two.statistics()["links"];
    if (!a.empty() && !b.empty() && a[0]["clock_valid"] == true && b[0]["clock_valid"] == true) {
      connected = true; break;
    }
  }
  ASSERT_TRUE(connected) << one.statistics().dump() << two.statistics().dump();
  horuslink::ChannelDescriptor catalog;
  catalog.topic = ControlLeaseManager::control_topic_catalog_topic(); catalog.type_name = "std_msgs/msg/String";
  const auto body = string_payload({{"role", "single"}, {"robots", {{{"robot_name", "rover"},
      {"protected_topics", {"/control_link_test/cmd_vel"}}}}}});
  ASSERT_TRUE(one.forward(11, catalog, body, 0));
  ASSERT_TRUE(two.forward(11, catalog, body, 0));
  pump(0.2);
  auto request = [&](ControlLink & link, const std::string & app, const std::string & action) {
      auto channel = catalog; channel.topic = ControlLeaseManager::lease_request_topic();
      EXPECT_TRUE(link.forward(11, channel, string_payload({{"robot_name", "rover"},
          {"app_id", app}, {"role", "single"}, {"action", action}, {"teleop_active", true}}), 0));
    };
  request(one, "one", "acquire"); pump(0.1);
  request(two, "two", "acquire"); pump(0.1);
  horuslink::ChannelDescriptor motion;
  motion.topic = "/control_link_test/cmd_vel"; motion.type_name = "geometry_msgs/msg/Twist";
  motion.delivery = horuslink::Delivery::ReplaceLatest;
  std::vector<uint8_t> bytes(48, 0); const double speed = 0.5;
  std::memcpy(bytes.data(), &speed, sizeof(double));
  EXPECT_TRUE(two.forward(11, motion, bytes, 0)); pump(0.05);
  EXPECT_TRUE(velocities.empty());
  EXPECT_TRUE(one.forward(11, motion, bytes, 0)); pump(0.05);
  ASSERT_EQ(velocities.size(), 1U); EXPECT_DOUBLE_EQ(velocities[0], speed);
  pump(0.3);
  ASSERT_EQ(velocities.size(), 2U); EXPECT_DOUBLE_EQ(velocities[1], 0);
  request(one, "one", "release"); pump(0.1);
  request(two, "two", "acquire"); pump(0.1);
  EXPECT_TRUE(two.forward(11, motion, bytes, 0)); pump(0.05);
  ASSERT_EQ(velocities.size(), 3U); EXPECT_DOUBLE_EQ(velocities.back(), speed);
  two.disconnected(11); pump(0.1);
  ASSERT_EQ(velocities.size(), 4U); EXPECT_DOUBLE_EQ(velocities.back(), 0);
  one.disconnected(11); pump(0.1);
  EXPECT_FALSE(one.handles(motion.topic));
  EXPECT_FALSE(two.handles(motion.topic));
  EXPECT_TRUE(one.statistics()["acknowledged"].get<int>() >= 2);
}
#endif
}  // namespace horus_unity_bridge
