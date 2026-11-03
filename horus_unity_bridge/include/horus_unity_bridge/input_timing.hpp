// Copyright 2026 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <chrono>
#include <cstdint>
#include <vector>

namespace horus_unity_bridge::horuslink
{
constexpr uint32_t kTimedInputFeature = 1;
constexpr uint16_t kFeaturesTlv = 22;
constexpr uint8_t kTimedInputFlag = 64;
inline int64_t input_clock_ms()
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
inline uint64_t read_timing_u64(const uint8_t * value)
{
  uint64_t result = 0;
  for (int index = 7; index >= 0; --index) {result = (result << 8) | value[index];}
  return result;
}
inline void write_timing_u64(uint8_t * value, uint64_t number)
{
  for (int index = 0; index < 8; ++index) {value[index] = number & 255; number >>= 8;}
}
inline bool is_clock_probe(const std::vector<uint8_t> & payload)
{
  return payload.size() == 20 && payload[0] == 'H' && payload[1] == 'L' &&
    payload[2] == 'T' && payload[3] == '1';
}
}  // namespace horus_unity_bridge::horuslink
