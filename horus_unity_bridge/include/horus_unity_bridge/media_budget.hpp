// Copyright 2026 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace horus_unity_bridge::media
{
struct Demand
{
  std::string id;
  int requested_kbps;
  int capacity_kbps;
  int weight = 1;
};

struct Budget
{
  std::map<std::string, int> streams;
  int video_kbps = 0;
  int bulk_kbps = 0;
  bool admissible = true;
};

// A configured shared-link governor, not another congestion estimator. Each
// flow's capacity still comes from GCC. Unused video allowance returns to bulk.
inline Budget allocate_link_budget(const std::vector<Demand> & demands,
  int link_kbps, int reserved_kbps = 512, int bulk_floor_kbps = 256)
{
  if (link_kbps < 0 || reserved_kbps < 0 || bulk_floor_kbps < 0) {
    throw std::invalid_argument("Link budgets must be nonnegative");
  }
  Budget result;
  const int available = link_kbps ? int(std::max<int64_t>(0,
    int64_t(link_kbps) - reserved_kbps - bulk_floor_kbps)) : 0;
  if (link_kbps && int64_t(demands.size()) * 100 > available) {
    result.admissible = false;
    return result;
  }
  int64_t requested = 0;
  std::map<std::string, int> limits, weights;
  for (const auto & demand : demands) {
    if (demand.id.empty() || limits.count(demand.id) || demand.weight < 1 || demand.weight > 16) {
      throw std::invalid_argument("Invalid or duplicate stream demand");
    }
    const int limit = std::clamp(std::min(demand.requested_kbps, demand.capacity_kbps), 100, 100000);
    limits[demand.id] = limit; weights[demand.id] = demand.weight;
    result.streams[demand.id] = link_kbps ? 100 : limit;
    requested += limit;
  }
  int64_t remaining = link_kbps ? std::min<int64_t>(requested, available) - int64_t(demands.size()) * 100 : 0;
  while (remaining > 0) {
    int total_weight = 0;
    for (const auto & entry : limits) {
      if (result.streams.at(entry.first) < entry.second) {total_weight += weights.at(entry.first);}
    }
    if (!total_weight) {break;}
    const auto round = remaining;
    for (const auto & entry : limits) {
      auto & assigned = result.streams.at(entry.first);
      if (assigned >= entry.second) {continue;}
      const int64_t share = std::max<int64_t>(1, round * weights.at(entry.first) / total_weight);
      const int grant = int(std::min({share, remaining, int64_t(entry.second - assigned)}));
      assigned += grant; remaining -= grant;
    }
  }
  for (const auto & stream : result.streams) {result.video_kbps += stream.second;}
  result.bulk_kbps = link_kbps ? std::max(0, link_kbps - reserved_kbps - result.video_kbps) : 0;
  return result;
}
}  // namespace horus_unity_bridge::media
