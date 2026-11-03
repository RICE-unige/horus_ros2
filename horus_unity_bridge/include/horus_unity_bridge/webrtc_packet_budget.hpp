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
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>

namespace horus_unity_bridge
{
// Leave room for RTP extensions, SRTP authentication, UDP and IPv6 on VPN paths.
// This changes packetization only, not image size, bitrate or encoder quality.
inline constexpr uint16_t kVideoRtpMaxFragmentBytes = 1100;
}  // namespace horus_unity_bridge
