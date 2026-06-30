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

#include "horus_unity_bridge/horuslink_light_codecs.hpp"

#include <gtest/gtest.h>

namespace horus_unity_bridge::horuslink
{

TEST(HorusLinkLightCodecsTest, TwistEncodesStableLittleEndianFloatVector)
{
  const Twist twist{
    Vector3{1.0F, -2.0F, 0.5F},
    Vector3{0.0F, 3.25F, -4.5F}
  };

  const auto encoded = encode_twist(twist);
  const std::vector<uint8_t> expected{
    0x00, 0x00, 0x80, 0x3F,
    0x00, 0x00, 0x00, 0xC0,
    0x00, 0x00, 0x00, 0x3F,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x50, 0x40,
    0x00, 0x00, 0x90, 0xC0
  };
  EXPECT_EQ(encoded, expected);

  const auto decoded = decode_twist(encoded.data(), encoded.size());
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(*decoded, twist);
}

TEST(HorusLinkLightCodecsTest, PoseRoundTripsPositionAndOrientation)
{
  const Pose pose{
    Vector3{1.25F, 2.5F, 5.0F},
    Quaternion{0.0F, 0.70710677F, 0.0F, 0.70710677F}
  };

  const auto encoded = encode_pose(pose);
  const auto decoded = decode_pose(encoded.data(), encoded.size());
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(*decoded, pose);
}

TEST(HorusLinkLightCodecsTest, DecodersRejectShortBuffers)
{
  std::vector<uint8_t> bytes(light_codec::kPoseSize, 0);

  EXPECT_FALSE(decode_vector3(bytes.data(), light_codec::kVector3Size - 1).has_value());
  EXPECT_FALSE(decode_quaternion(bytes.data(), light_codec::kQuaternionSize - 1).has_value());
  EXPECT_FALSE(decode_twist(bytes.data(), light_codec::kTwistSize - 1).has_value());
  EXPECT_FALSE(decode_pose(bytes.data(), light_codec::kPoseSize - 1).has_value());
}

TEST(HorusLinkLightCodecsTest, DecodersRejectNullPointers)
{
  EXPECT_FALSE(decode_vector3(nullptr, light_codec::kVector3Size).has_value());
  EXPECT_FALSE(decode_quaternion(nullptr, light_codec::kQuaternionSize).has_value());
  EXPECT_FALSE(decode_twist(nullptr, light_codec::kTwistSize).has_value());
  EXPECT_FALSE(decode_pose(nullptr, light_codec::kPoseSize).has_value());
}

}  // namespace horus_unity_bridge::horuslink
