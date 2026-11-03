// Copyright 2026 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0
#include "horus_unity_bridge/media_budget.hpp"
#include <gtest/gtest.h>
#include <random>

using namespace horus_unity_bridge::media;

TEST(MediaBudget, UnconfiguredBudgetPreservesIndependentGccLimits)
{
  const auto result = allocate_link_budget({{"a", 3000, 1500}, {"b", 3000, 2500}}, 0);
  EXPECT_EQ(result.streams.at("a"), 1500);
  EXPECT_EQ(result.streams.at("b"), 2500);
  EXPECT_EQ(result.bulk_kbps, 0);
}

TEST(MediaBudget, PrioritizesActiveViewAndKeepsTheAggregateBounded)
{
  const auto result = allocate_link_budget({{"overview", 3000, 3000, 1},
    {"teleop", 3000, 3000, 4}}, 4000);
  ASSERT_TRUE(result.admissible);
  EXPECT_GT(result.streams.at("teleop"), result.streams.at("overview"));
  EXPECT_EQ(result.video_kbps + result.bulk_kbps + 512, 4000);
  EXPECT_GE(result.bulk_kbps, 256);
  EXPECT_LE(result.streams.at("teleop"), 3000);
}

TEST(MediaBudget, ReclaimsUnusedVideoAndExplicitlyRejectsImpossibleAdmission)
{
  const auto result = allocate_link_budget({{"camera", 1000, 500}}, 10000);
  EXPECT_EQ(result.streams.at("camera"), 500);
  EXPECT_EQ(result.bulk_kbps, 8988);
  EXPECT_FALSE(allocate_link_budget({{"a", 1000, 1000}, {"b", 1000, 1000}}, 900).admissible);
  EXPECT_THROW(allocate_link_budget({{"a", 1000, 1000}, {"a", 1000, 1000}}, 4000), std::invalid_argument);
}

TEST(MediaBudget, AllocationsRespectEveryFlowAndTotalAcrossMixedDemands)
{
  std::mt19937 random(103);
  for (int iteration = 0; iteration < 500; ++iteration) {
    std::vector<Demand> demands;
    const auto count = 1 + random() % 64;
    for (unsigned index = 0; index < count; ++index) {
      demands.push_back({std::to_string(index), int(100 + random() % 20000),
        int(100 + random() % 20000), int(1 + random() % 16)});
    }
    const int link = 768 + int(count * 100) + int(random() % 100000);
    const auto result = allocate_link_budget(demands, link);
    ASSERT_TRUE(result.admissible);
    EXPECT_EQ(result.video_kbps + result.bulk_kbps + 512, link);
    EXPECT_GE(result.bulk_kbps, 256);
    for (const auto & demand : demands) {
      EXPECT_GE(result.streams.at(demand.id), 100);
      EXPECT_LE(result.streams.at(demand.id), std::min(demand.requested_kbps, demand.capacity_kbps));
    }
  }
}
