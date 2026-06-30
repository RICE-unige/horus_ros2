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

#include "horus_unity_bridge/horuslink_outbound_lane_queue.hpp"

#include <gtest/gtest.h>

namespace horus_unity_bridge::horuslink
{

namespace
{

Frame make_frame(uint32_t seq, bool replace_latest = false)
{
  Frame frame;
  frame.header.channel_id = 3;
  frame.header.msg_type = MessageType::Data;
  frame.header.flags = FrameFlags::RawOpaque;
  if (replace_latest) {
    frame.header.flags |= FrameFlags::ReplaceLatest;
  }
  frame.header.seq = seq;
  frame.payload = {static_cast<uint8_t>(seq & 0xFFu)};
  frame.header.length = static_cast<uint32_t>(frame.payload.size());
  return frame;
}

}  // namespace

TEST(HorusLinkOutboundLaneQueueTest, ReliableOverflowIsRejectedWithoutDroppingQueuedFrames)
{
  OutboundLaneQueue queue(1);

  auto first = queue.enqueue(make_frame(1));
  auto overflow = queue.enqueue(make_frame(2));

  EXPECT_TRUE(first.accepted);
  EXPECT_FALSE(first.overflow);
  EXPECT_FALSE(overflow.accepted);
  EXPECT_TRUE(overflow.overflow);
  EXPECT_FALSE(overflow.replaced);
  EXPECT_EQ(queue.size(), 1u);
  auto queued = queue.pop();
  ASSERT_TRUE(queued.has_value());
  EXPECT_EQ(queued->header.seq, 1u);
  EXPECT_FALSE(queue.pop().has_value());
}

TEST(HorusLinkOutboundLaneQueueTest, ReplaceLatestOverflowReplacesOldestReplaceableFrame)
{
  OutboundLaneQueue queue(2);
  ASSERT_TRUE(queue.enqueue(make_frame(1, true)).accepted);
  ASSERT_TRUE(queue.enqueue(make_frame(2, true)).accepted);

  auto replacement = queue.enqueue(make_frame(3, true));

  EXPECT_TRUE(replacement.accepted);
  EXPECT_TRUE(replacement.overflow);
  EXPECT_TRUE(replacement.replaced);
  ASSERT_TRUE(replacement.replaced_frame.has_value());
  EXPECT_EQ(replacement.replaced_frame->header.seq, 1u);
  auto first = queue.pop();
  auto second = queue.pop();
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(first->header.seq, 3u);
  EXPECT_EQ(second->header.seq, 2u);
}

TEST(HorusLinkOutboundLaneQueueTest, ReplaceLatestOverflowPreservesReliableFrames)
{
  OutboundLaneQueue queue(2);
  ASSERT_TRUE(queue.enqueue(make_frame(1, false)).accepted);
  ASSERT_TRUE(queue.enqueue(make_frame(2, true)).accepted);

  auto replacement = queue.enqueue(make_frame(3, true));

  EXPECT_TRUE(replacement.accepted);
  EXPECT_TRUE(replacement.replaced);
  ASSERT_TRUE(replacement.replaced_frame.has_value());
  EXPECT_EQ(replacement.replaced_frame->header.seq, 2u);
  auto first = queue.pop();
  auto second = queue.pop();
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(first->header.seq, 1u);
  EXPECT_EQ(second->header.seq, 3u);
}

TEST(HorusLinkOutboundLaneQueueTest, ReplaceLatestOverflowIsRejectedWhenQueueHasOnlyReliableFrames)
{
  OutboundLaneQueue queue(2);
  ASSERT_TRUE(queue.enqueue(make_frame(1, false)).accepted);
  ASSERT_TRUE(queue.enqueue(make_frame(2, false)).accepted);

  auto rejected = queue.enqueue(make_frame(3, true));

  EXPECT_FALSE(rejected.accepted);
  EXPECT_TRUE(rejected.overflow);
  EXPECT_FALSE(rejected.replaced);
  EXPECT_EQ(queue.size(), 2u);
  auto first = queue.pop();
  auto second = queue.pop();
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(first->header.seq, 1u);
  EXPECT_EQ(second->header.seq, 2u);
}

}  // namespace horus_unity_bridge::horuslink
