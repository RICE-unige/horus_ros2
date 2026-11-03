// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

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

namespace
{
struct TestSocket
{
  int fd = -1;
  explicit TestSocket(int port)
  {
    fd = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    EXPECT_EQ(connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)), 0);
    timeval timeout{2, 0};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  }
  ~TestSocket() {if (fd >= 0) {close(fd);}}
  std::string read(size_t length)
  {
    std::string result;
    char buffer[4096];
    while (result.size() < length) {
      auto count = recv(fd, buffer, std::min(sizeof(buffer), length - result.size()), 0);
      if (count <= 0) {break;}
      result.append(buffer, static_cast<size_t>(count));
    }
    return result;
  }
};
}

TEST(BackendNode, LegacyCommandsNeverReportExecutionAndUninitializedConnectIsRejected)
{
  auto node = std::make_shared<horus_backend::BackendNode>();
  auto client_node = std::make_shared<rclcpp::Node>("backend_service_test");
  auto execute = client_node->create_client<horus_interfaces::srv::ExecuteCommand>(
    "/horus/execute_command");
  auto connect = client_node->create_client<horus_interfaces::srv::ConnectRobot>(
    "/horus/connect_robot");
  ASSERT_TRUE(execute->wait_for_service(std::chrono::seconds(2)));
  ASSERT_TRUE(connect->wait_for_service(std::chrono::seconds(2)));
  auto command = std::make_shared<horus_interfaces::srv::ExecuteCommand::Request>();
  command->robot_id = "test";
  command->command_type = "move";
  auto executing = execute->async_send_request(command);
  auto connecting = connect->async_send_request(
    std::make_shared<horus_interfaces::srv::ConnectRobot::Request>());
  for (int i = 0; i < 200 &&
    (executing.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready ||
    connecting.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready); ++i)
  {
    rclcpp::spin_some(node);
    rclcpp::spin_some(client_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_EQ(executing.wait_for(std::chrono::milliseconds(0)), std::future_status::ready);
  ASSERT_EQ(connecting.wait_for(std::chrono::milliseconds(0)), std::future_status::ready);
  EXPECT_FALSE(executing.get()->success);
  EXPECT_FALSE(connecting.get()->success);
}

TEST(TcpServer, FragmentsAndCoalescedFramesAreProcessedOnceAndIncompleteTailIsDiscarded)
{
  horus_backend::TcpServer server(0);
  std::atomic<int> calls{0};
  server.set_message_callback([&](const std::string & request) {
    ++calls;
    return request;
  });
  server.start();
  TestSocket socket(server.port());
  ASSERT_EQ(send(socket.fd, "fir", 3, MSG_NOSIGNAL), 3);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  EXPECT_EQ(calls.load(), 0);
  ASSERT_EQ(send(socket.fd, "st\nsecond\nunfinished", 20, MSG_NOSIGNAL), 20);
  EXPECT_EQ(socket.read(13), "first\nsecond\n");
  server.stop();
  EXPECT_EQ(calls.load(), 2);
}

TEST(TcpServer, PartialWritesDeliverLargeRepliesAndCallbackExceptionsDoNotTerminateServer)
{
  horus_backend::TcpServer server(0);
  const std::string expected(2 * 1024 * 1024, 'x');
  server.set_message_callback([&](const std::string & request) {
    if (request == "bad") {throw std::runtime_error("test");}
    return expected;
  });
  server.start();
  TestSocket socket(server.port());
  ASSERT_EQ(send(socket.fd, "ok\n", 3, MSG_NOSIGNAL), 3);
  EXPECT_EQ(socket.read(expected.size() + 1), expected + "\n");
  ASSERT_EQ(send(socket.fd, "bad\n", 4, MSG_NOSIGNAL), 4);
  const std::string error = "{\"status\":\"error\",\"message\":\"request processing failed\"}\n";
  EXPECT_EQ(socket.read(error.size()), error);
  server.stop();
}

TEST(TcpServer, StopInterruptsBlockedSendAndServerCanRestart)
{
  horus_backend::TcpServer server(0);
  std::atomic<bool> called{false};
  server.set_message_callback([&](const std::string &) {
    called = true;
    return std::string(16 * 1024 * 1024, 'x');
  });
  server.start();
  TestSocket socket(server.port());
  int receive_buffer = 1024;
  setsockopt(socket.fd, SOL_SOCKET, SO_RCVBUF, &receive_buffer, sizeof(receive_buffer));
  ASSERT_EQ(send(socket.fd, "ok\n", 3, MSG_NOSIGNAL), 3);
  for (int i = 0; i < 100 && !called; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  EXPECT_TRUE(called.load());
  const auto start = std::chrono::steady_clock::now();
  server.stop();
  EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::seconds(1));
  server.start();
  EXPECT_TRUE(server.is_running());
  server.stop();
}

TEST(BackendNode, EscapesTcpResponsesAsValidJson)
{
  auto node = std::make_shared<horus_backend::BackendNode>();

  const std::string response =
    node->process_message_for_test("quoted \"value\"\\path\nnext");

  EXPECT_NE(response.find("\"status\": \"unsupported\""), std::string::npos);
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
