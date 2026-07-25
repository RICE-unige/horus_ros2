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

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <vector>

#include "horus_unity_bridge/remote_render_auxiliary.hpp"

namespace
{

using horus_unity_bridge::remote_render::chunk_auxiliary_payload;
using horus_unity_bridge::remote_render::kAuxiliaryChunkHeaderSize;
using horus_unity_bridge::remote_render::read_u32_le;

std::vector<uint8_t> make_remote_frame(std::size_t size, uint32_t sequence)
{
  std::vector<uint8_t> frame(std::max<std::size_t>(size, 12U));
  frame[0] = 'H';
  frame[1] = 'R';
  frame[2] = 'R';
  frame[3] = 'F';
  frame[8] = static_cast<uint8_t>(sequence & 0xffU);
  frame[9] = static_cast<uint8_t>((sequence >> 8U) & 0xffU);
  frame[10] = static_cast<uint8_t>((sequence >> 16U) & 0xffU);
  frame[11] = static_cast<uint8_t>((sequence >> 24U) & 0xffU);
  for (std::size_t index = 12; index < frame.size(); ++index) {
    frame[index] = static_cast<uint8_t>((index * 37U) & 0xffU);
  }
  return frame;
}

TEST(RemoteRenderAuxiliary, LeavesSmallPayloadUnwrapped)
{
  const auto frame = make_remote_frame(1024U, 42U);
  const auto chunks = chunk_auxiliary_payload(frame, 256U * 1024U);
  ASSERT_EQ(chunks.size(), 1U);
  EXPECT_EQ(chunks.front(), frame);
}

TEST(RemoteRenderAuxiliary, ChunksAndReassemblesLargePayload)
{
  const auto frame = make_remote_frame(410U * 1024U, 0x12345678U);
  auto chunks = chunk_auxiliary_payload(frame, 256U * 1024U);
  ASSERT_GT(chunks.size(), 1U);
  std::reverse(chunks.begin(), chunks.end());

  std::vector<uint8_t> restored(frame.size());
  std::vector<bool> received(chunks.size(), false);
  for (const auto & chunk : chunks) {
    ASSERT_GE(chunk.size(), kAuxiliaryChunkHeaderSize);
    EXPECT_EQ(chunk[0], 'H');
    EXPECT_EQ(chunk[1], 'R');
    EXPECT_EQ(chunk[2], 'A');
    EXPECT_EQ(chunk[3], '1');
    EXPECT_EQ(read_u32_le(chunk.data() + 4U), 0x12345678U);
    EXPECT_EQ(read_u32_le(chunk.data() + 8U), frame.size());
    const uint16_t index = static_cast<uint16_t>(
      chunk[12] | static_cast<uint16_t>(chunk[13]) << 8U);
    const uint16_t count = static_cast<uint16_t>(
      chunk[14] | static_cast<uint16_t>(chunk[15]) << 8U);
    const uint32_t offset = read_u32_le(chunk.data() + 16U);
    ASSERT_EQ(count, chunks.size());
    ASSERT_LT(index, received.size());
    ASSERT_LE(offset + chunk.size() - kAuxiliaryChunkHeaderSize, restored.size());
    std::copy(
      chunk.begin() + static_cast<std::ptrdiff_t>(kAuxiliaryChunkHeaderSize),
      chunk.end(),
      restored.begin() + static_cast<std::ptrdiff_t>(offset));
    received[index] = true;
  }

  EXPECT_TRUE(std::all_of(received.begin(), received.end(), [](bool value) {
      return value;
  }));
  EXPECT_EQ(restored, frame);
}

TEST(RemoteRenderAuxiliary, HonorsSmallNegotiatedMessageLimit)
{
  const auto frame = make_remote_frame(200U * 1024U, 9U);
  const auto chunks = chunk_auxiliary_payload(frame, 16U * 1024U);
  ASSERT_GT(chunks.size(), 1U);
  for (const auto & chunk : chunks) {
    EXPECT_LE(chunk.size(), 16U * 1024U);
  }
}

}  // namespace
