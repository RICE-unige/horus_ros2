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

// SPDX-FileCopyrightText: 2026 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#include "horus_unity_bridge/bridge_metrics.hpp"

#include <cstdlib>
#include <filesystem>

namespace horus_unity_bridge
{
namespace
{
constexpr std::uint64_t kRowsPerFlush = 64;
}

BridgeMetrics & BridgeMetrics::instance()
{
  static BridgeMetrics metrics;
  return metrics;
}

BridgeMetrics::BridgeMetrics()
{
  const char * results_dir = std::getenv("HORUS_EXPERIMENT_RESULTS_DIR");
  if (results_dir == nullptr || std::string(results_dir).empty()) {
    return;
  }

  run_id_ = std::getenv("HORUS_EXPERIMENT_RUN_ID") ? std::getenv("HORUS_EXPERIMENT_RUN_ID") : "";
  experiment_ = std::getenv("HORUS_EXPERIMENT") ? std::getenv("HORUS_EXPERIMENT") : "";
  condition_ =
    std::getenv("HORUS_EXPERIMENT_CONDITION") ? std::getenv("HORUS_EXPERIMENT_CONDITION") : "";

  std::filesystem::path root(results_dir);
  std::filesystem::create_directories(root);
  csv_.open(root / "bridge_metrics.csv", std::ios::out | std::ios::trunc);
  if (!csv_.is_open()) {
    return;
  }

  csv_ <<
    "timestamp_ns,run_id,experiment,condition,source,category,name,client_fd,"
    "destination,payload_bytes,serialized_bytes,queue_depth,extra_json\n";
  csv_.flush();
  enabled_ = true;
}

BridgeMetrics::~BridgeMetrics()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (csv_.is_open()) {
    csv_.flush();
  }
}

bool BridgeMetrics::enabled() const
{
  return enabled_;
}

void BridgeMetrics::record(
  const std::string & category,
  const std::string & name,
  int client_fd,
  const std::string & destination,
  std::uint64_t payload_bytes,
  std::uint64_t serialized_bytes,
  std::uint64_t queue_depth,
  const std::string & extra_json)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  csv_ << unix_time_ns() << ','
       << csv(run_id_) << ','
       << csv(experiment_) << ','
       << csv(condition_) << ','
       << "horus_ros2_bridge" << ','
       << csv(category) << ','
       << csv(name) << ','
       << client_fd << ','
       << csv(destination) << ','
       << payload_bytes << ','
       << serialized_bytes << ','
       << queue_depth << ','
       << csv(extra_json) << '\n';
  flush_if_needed();
}

void BridgeMetrics::flush_if_needed()
{
  rows_since_flush_++;
  if (rows_since_flush_ < kRowsPerFlush) {
    return;
  }

  rows_since_flush_ = 0;
  csv_.flush();
}

std::int64_t BridgeMetrics::unix_time_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

std::string BridgeMetrics::csv(const std::string & value)
{
  if (value.find_first_of(",\"\n\r") == std::string::npos) {
    return value;
  }

  std::string escaped;
  escaped.reserve(value.size() + 2);
  escaped.push_back('"');
  for (char ch : value) {
    if (ch == '"') {
      escaped.push_back('"');
    }
    escaped.push_back(ch);
  }
  escaped.push_back('"');
  return escaped;
}

}  // namespace horus_unity_bridge
