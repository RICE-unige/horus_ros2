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
#include <utility>

namespace horus_unity_bridge::horuslink
{

OutboundLaneQueue::OutboundLaneQueue(size_t max_depth)
: max_depth_(max_depth)
{
  if (max_depth_ == 0) {
    throw std::out_of_range("HorusLink outbound lane queue depth must be positive.");
  }
}

OutboundLaneEnqueueResult OutboundLaneQueue::enqueue(Frame frame)
{
  OutboundLaneEnqueueResult result;
  const bool replace_latest = is_replace_latest(frame);
  if (replace_latest) {
    for (auto it = frames_.begin(); it != frames_.end(); ++it) {
      if (!is_replace_latest(*it) || it->header.channel_id != frame.header.channel_id) {
        continue;
      }

      result.replaced_frame = std::move(*it);
      *it = std::move(frame);
      result.accepted = true;
      result.replaced = true;
      result.depth = frames_.size();
      return result;
    }
  }

  if (frames_.size() < max_depth_) {
    frames_.push_back(std::move(frame));
    result.accepted = true;
    result.depth = frames_.size();
    return result;
  }

  result.overflow = true;
  if (!replace_latest) {
    result.depth = frames_.size();
    return result;
  }

  for (auto it = frames_.begin(); it != frames_.end(); ++it) {
    if (!is_replace_latest(*it)) {
      continue;
    }

    result.replaced_frame = std::move(*it);
    *it = std::move(frame);
    result.accepted = true;
    result.replaced = true;
    result.depth = frames_.size();
    return result;
  }

  result.depth = frames_.size();
  return result;
}

std::optional<Frame> OutboundLaneQueue::pop()
{
  if (frames_.empty()) {
    return std::nullopt;
  }

  Frame frame = std::move(frames_.front());
  frames_.pop_front();
  return frame;
}

void OutboundLaneQueue::clear()
{
  frames_.clear();
}

bool OutboundLaneQueue::is_replace_latest(const Frame & frame)
{
  return (frame.header.flags & FrameFlags::ReplaceLatest) != 0;
}

}  // namespace horus_unity_bridge::horuslink
