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

#include "horus_unity_bridge/horuslink_protocol.hpp"

#include <cstddef>
#include <deque>
#include <optional>

namespace horus_unity_bridge::horuslink
{

struct OutboundLaneEnqueueResult
{
  bool accepted = false;
  bool replaced = false;
  bool overflow = false;
  size_t depth = 0;
  std::optional<Frame> replaced_frame;
};

class OutboundLaneQueue
{
public:
  explicit OutboundLaneQueue(size_t max_depth);

  OutboundLaneEnqueueResult enqueue(Frame frame);
  std::optional<Frame> pop();
  void clear();

  size_t size() const {return frames_.size();}
  size_t max_depth() const {return max_depth_;}
  bool empty() const {return frames_.empty();}

private:
  static bool is_replace_latest(const Frame & frame);

  size_t max_depth_ = 0;
  std::deque<Frame> frames_;
};

}  // namespace horus_unity_bridge::horuslink
