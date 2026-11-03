// Copyright 2026 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <cstdlib>
#include <limits>

namespace horus_unity_bridge::control
{
// Map process-monotonic clocks using the lowest observed round trip, never UTC.
struct ClockEstimate
{
  int64_t offset_ms = 0;
  int64_t uncertainty_ms = std::numeric_limits<int64_t>::max();
  int64_t sampled_at_ms = 0;

  bool observe(int64_t sent, int64_t received, int64_t remote)
  {
    if (received < sent || received - sent > 1000) {return false;}
    const auto uncertainty = (received - sent + 1) / 2;
    if (uncertainty <= uncertainty_ms || received - sampled_at_ms > 5000) {
      offset_ms = remote - (sent + (received - sent) / 2);
      uncertainty_ms = uncertainty;
    }
    sampled_at_ms = received;
    return true;
  }
  bool valid(int64_t now) const
  {
    return sampled_at_ms && now >= sampled_at_ms && now - sampled_at_ms <= 3000 &&
           uncertainty_ms < 100;
  }
  int64_t deadline(int64_t now, int64_t lifetime_ms) const
  {
    return valid(now) ? now + offset_ms + lifetime_ms - uncertainty_ms : 0;
  }
};

struct SequenceDeadline
{
  uint64_t last_sequence = 0;
  bool accept(uint64_t sequence, int64_t deadline, int64_t now, int64_t maximum_future = 300)
  {
    if (!sequence || sequence <= last_sequence) {return false;}
    // Even an expired stop/sample forms a barrier against older motion.
    last_sequence = sequence;
    return deadline >= now && deadline - now <= maximum_future;
  }
};
}  // namespace horus_unity_bridge::control
