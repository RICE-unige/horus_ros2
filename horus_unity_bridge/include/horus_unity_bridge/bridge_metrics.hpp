// SPDX-FileCopyrightText: 2026 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <chrono>
#include <cstddef>
#include <fstream>
#include <mutex>
#include <string>

namespace horus_unity_bridge
{

class BridgeMetrics
{
public:
  static BridgeMetrics& instance();

  bool enabled() const;

  void record(const std::string& category,
              const std::string& name,
              int client_fd,
              const std::string& destination,
              std::size_t payload_bytes,
              std::size_t serialized_bytes = 0,
              std::size_t queue_depth = 0,
              const std::string& extra_json = "");

private:
  BridgeMetrics();

  static long long unix_time_ns();
  static std::string csv(const std::string& value);

  mutable std::mutex mutex_;
  bool enabled_ = false;
  std::string run_id_;
  std::string experiment_;
  std::string condition_;
  std::ofstream csv_;
};

}  // namespace horus_unity_bridge
