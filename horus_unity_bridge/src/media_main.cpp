// Copyright 2026 RICE Lab, University of Genoa
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

#include "horus_unity_bridge/message_router.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    // Reuse the bridge's negotiated camera path, without a second TCP bridge.
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__node:=horus_media"});
    options.parameter_overrides({{"webrtc.enabled", true}});
    auto node = std::make_shared<horus_unity_bridge::MessageRouter>(options, true);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    RCLCPP_INFO(node->get_logger(), "Source-side WebRTC ready; waiting for camera offers");
    executor.spin();
    executor.remove_node(node);
    node.reset();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("horus_media"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
