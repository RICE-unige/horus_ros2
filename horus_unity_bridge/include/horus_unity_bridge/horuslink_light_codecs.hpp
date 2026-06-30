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

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace horus_unity_bridge::horuslink
{

struct Vector3
{
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;

  bool operator==(const Vector3 & other) const;
};

struct Quaternion
{
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  float w = 1.0F;

  bool operator==(const Quaternion & other) const;
};

struct Twist
{
  Vector3 linear;
  Vector3 angular;

  bool operator==(const Twist & other) const;
};

struct Pose
{
  Vector3 position;
  Quaternion orientation;

  bool operator==(const Pose & other) const;
};

struct Joy
{
  std::vector<float> axes;
  std::vector<int32_t> buttons;

  bool operator==(const Joy & other) const;
};

namespace light_codec
{
constexpr size_t kVector3Size = 12;
constexpr size_t kQuaternionSize = 16;
constexpr size_t kTwistSize = kVector3Size * 2;
constexpr size_t kPoseSize = kVector3Size + kQuaternionSize;
constexpr size_t kJoyHeaderSize = 4;
}  // namespace light_codec

std::vector<uint8_t> encode_vector3(const Vector3 & value);
std::optional<Vector3> decode_vector3(const uint8_t * data, size_t size);

std::vector<uint8_t> encode_quaternion(const Quaternion & value);
std::optional<Quaternion> decode_quaternion(const uint8_t * data, size_t size);

std::vector<uint8_t> encode_twist(const Twist & value);
std::optional<Twist> decode_twist(const uint8_t * data, size_t size);

std::vector<uint8_t> encode_pose(const Pose & value);
std::optional<Pose> decode_pose(const uint8_t * data, size_t size);

std::vector<uint8_t> encode_joy(const Joy & value);
std::optional<Joy> decode_joy(const uint8_t * data, size_t size);

}  // namespace horus_unity_bridge::horuslink
