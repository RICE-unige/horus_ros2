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

#include <stdexcept>
#include <algorithm>
#include <utility>

namespace horus_unity_bridge::horuslink
{

OutboundLaneQueue::OutboundLaneQueue(size_t max_depth, size_t max_bytes, bool interleave_latest)
: max_depth_(max_depth), max_bytes_(max_bytes), interleave_latest_(interleave_latest)
{
  if (max_depth_ == 0 || max_bytes_ == 0) {
    throw std::out_of_range("HorusLink outbound lane queue limits must be positive.");
  }
}

OutboundLaneEnqueueResult OutboundLaneQueue::enqueue(Frame frame)
{
  OutboundLaneEnqueueResult result;
  const size_t frame_bytes = FrameHeader::kSize + frame.payload.size();
  const bool replace_latest = is_replace_latest(frame);
  if (replace_latest) {
    for (auto it = frames_.begin(); it != frames_.end(); ++it) {
      if (!is_replace_latest(*it) || it->header.channel_id != frame.header.channel_id) {
        continue;
      }

      const size_t other_bytes = bytes_ - FrameHeader::kSize - it->payload.size();
      if (other_bytes != 0 &&
        (other_bytes >= max_bytes_ || frame_bytes > max_bytes_ - other_bytes))
      {
        result.overflow = true;
        result.depth = frames_.size();
        return result;
      }
      bytes_ = other_bytes + frame_bytes;
      result.replaced_frame = std::move(*it);
      *it = std::move(frame);
      result.accepted = true;
      result.replaced = true;
      result.depth = frames_.size();
      return result;
    }
  }

  // A single legal oversized payload can occupy an empty lane. The connection
  // payload limit still bounds it; subsequent frames must wait for space.
  if (frames_.size() < max_depth_ && (frames_.empty() ||
    (bytes_ < max_bytes_ && frame_bytes <= max_bytes_ - bytes_)))
  {
    bytes_ += frame_bytes;
    frames_.push_back(std::move(frame));
    result.accepted = true;
    result.depth = frames_.size();
    return result;
  }

  result.overflow = true;
  result.depth = frames_.size();
  return result;
}

std::optional<Frame> OutboundLaneQueue::pop()
{
  if (frames_.empty()) {
    return std::nullopt;
  }

  auto next = frames_.begin();
  // Bound interference from FIFO map bursts without reordering reliable frames.
  // TCP frames remain atomic: a single large frame cannot be preempted here.
  constexpr size_t burst_bytes = 1024u * 1024u;
  if (interleave_latest_ && (reliable_burst_frames_ >= 4 || reliable_burst_bytes_ >= burst_bytes)) {
    const auto latest = std::find_if(frames_.begin(), frames_.end(), is_replace_latest);
    if (latest != frames_.end()) {next = latest;}
  }
  Frame frame = std::move(*next);
  bytes_ -= FrameHeader::kSize + frame.payload.size();
  frames_.erase(next);
  if (is_replace_latest(frame)) {
    reliable_burst_frames_ = 0;
    reliable_burst_bytes_ = 0;
  } else {
    reliable_burst_frames_ = std::min(size_t{4}, reliable_burst_frames_ + 1);
    reliable_burst_bytes_ = std::min(burst_bytes, reliable_burst_bytes_ + frame.payload.size());
  }
  return frame;
}

void OutboundLaneQueue::clear()
{
  frames_.clear();
  bytes_ = 0;
  reliable_burst_frames_ = 0;
  reliable_burst_bytes_ = 0;
}

void OutboundLaneQueue::remove_channel(uint16_t channel_id)
{
  for (auto it = frames_.begin(); it != frames_.end(); ) {
    if (it->header.channel_id == channel_id) {
      bytes_ -= FrameHeader::kSize + it->payload.size();
      it = frames_.erase(it);
    } else {
      ++it;
    }
  }
}

bool OutboundLaneQueue::is_replace_latest(const Frame & frame)
{
  return (frame.header.flags & FrameFlags::ReplaceLatest) != 0;
}

}  // namespace horus_unity_bridge::horuslink
