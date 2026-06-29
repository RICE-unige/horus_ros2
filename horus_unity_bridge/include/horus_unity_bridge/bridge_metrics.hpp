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

#pragma once

#include <chrono>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>

namespace horus_unity_bridge
{

class BridgeMetrics
{
public:
  static BridgeMetrics & instance();

  bool enabled() const;

  void record(
    const std::string & category,
    const std::string & name,
    int client_fd,
    const std::string & destination,
    std::uint64_t payload_bytes,
    std::uint64_t serialized_bytes = 0,
    std::uint64_t queue_depth = 0,
    const std::string & extra_json = "");

  // Wall-clock (system_clock) nanoseconds. Public so producers can stamp enqueue time on the same
  // clock as the recorded timestamp_ns, making cross-event durations (e.g. time-in-queue) exact.
  static std::int64_t unix_time_ns();

private:
  BridgeMetrics();
  ~BridgeMetrics();

  static std::string csv(const std::string & value);

  void flush_if_needed();

  mutable std::mutex mutex_;
  bool enabled_ = false;
  std::string run_id_;
  std::string experiment_;
  std::string condition_;
  std::ofstream csv_;
  std::uint64_t rows_since_flush_ = 0;
};

}  // namespace horus_unity_bridge
