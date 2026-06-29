// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "horus_backend/backend_node.hpp"
#include "horus_backend/tcp_server.hpp"

namespace
{

class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void TearDown() override
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

}  // namespace

TEST(BackendNode, EscapesTcpResponsesAsValidJson)
{
  auto node = std::make_shared<horus_backend::BackendNode>();

  const std::string response =
    node->process_message_for_test("quoted \"value\"\\path\nnext");

  EXPECT_NE(response.find("\"status\": \"ok\""), std::string::npos);
  EXPECT_NE(response.find("quoted \\\"value\\\"\\\\path\\nnext"), std::string::npos);
}

TEST(TcpServer, StopIsSafeBeforeStart)
{
  horus_backend::TcpServer server(0);

  server.stop();

  EXPECT_FALSE(server.is_running());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ::testing::AddGlobalTestEnvironment(new RclcppEnvironment);
  return RUN_ALL_TESTS();
}
