// SPDX-FileCopyrightText: 2026 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#include "horus_unity_bridge/bridge_metrics.hpp"

#include <cstdlib>
#include <filesystem>

namespace horus_unity_bridge
{

BridgeMetrics& BridgeMetrics::instance()
{
  static BridgeMetrics metrics;
  return metrics;
}

BridgeMetrics::BridgeMetrics()
{
  const char* results_dir = std::getenv("HORUS_EXPERIMENT_RESULTS_DIR");
  if (results_dir == nullptr || std::string(results_dir).empty()) {
    return;
  }

  run_id_ = std::getenv("HORUS_EXPERIMENT_RUN_ID") ? std::getenv("HORUS_EXPERIMENT_RUN_ID") : "";
  experiment_ = std::getenv("HORUS_EXPERIMENT") ? std::getenv("HORUS_EXPERIMENT") : "";
  condition_ = std::getenv("HORUS_EXPERIMENT_CONDITION") ? std::getenv("HORUS_EXPERIMENT_CONDITION") : "";

  std::filesystem::path root(results_dir);
  std::filesystem::create_directories(root);
  csv_.open(root / "bridge_metrics.csv", std::ios::out | std::ios::trunc);
  if (!csv_.is_open()) {
    return;
  }

  csv_ << "timestamp_ns,run_id,experiment,condition,source,category,name,client_fd,"
          "destination,payload_bytes,serialized_bytes,queue_depth,extra_json\n";
  enabled_ = true;
}

bool BridgeMetrics::enabled() const
{
  return enabled_;
}

void BridgeMetrics::record(const std::string& category,
                           const std::string& name,
                           int client_fd,
                           const std::string& destination,
                           std::size_t payload_bytes,
                           std::size_t serialized_bytes,
                           std::size_t queue_depth,
                           const std::string& extra_json)
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
}

long long BridgeMetrics::unix_time_ns()
{
  using namespace std::chrono;
  return duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();
}

std::string BridgeMetrics::csv(const std::string& value)
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
