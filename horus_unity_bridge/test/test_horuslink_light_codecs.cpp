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
#include "horuslink_golden_vectors.hpp"

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
  const auto expected = load_horuslink_golden_vector("twist");
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

TEST(HorusLinkLightCodecsTest, PoseStampedRoundTripsHeaderFrameAndPose)
{
  const PoseStamped pose{
    12,
    345,
    "drone/map",
    Pose{
      Vector3{1.25F, 2.5F, 5.0F},
      Quaternion{0.0F, 0.70710677F, 0.0F, 0.70710677F}
    }
  };

  const auto encoded = encode_pose_stamped(pose);
  const auto decoded = decode_pose_stamped(encoded.data(), encoded.size());
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(*decoded, pose);
}

TEST(HorusLinkLightCodecsTest, PathRoundTripsHeaderAndStampedPoses)
{
  const Path path{
    20,
    40,
    "map",
    {
      PoseStamped{
        21,
        41,
        "map",
        Pose{
          Vector3{1.0F, 2.0F, 3.0F},
          Quaternion{0.0F, 0.0F, 0.0F, 1.0F}
        }
      },
      PoseStamped{
        22,
        42,
        "odom",
        Pose{
          Vector3{4.0F, 5.0F, 6.0F},
          Quaternion{0.0F, 0.0F, 0.70710677F, 0.70710677F}
        }
      }
    }
  };

  const auto encoded = encode_path(path);
  const auto decoded = decode_path(encoded.data(), encoded.size());
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(*decoded, path);
}

TEST(HorusLinkLightCodecsTest, JoyEncodesCountsAxesAndButtons)
{
  const Joy joy{
    {1.0F, -0.5F, 0.25F},
    {1, 0, -1}
  };

  const auto encoded = encode_joy(joy);
  const auto expected = load_horuslink_golden_vector("joy");
  EXPECT_EQ(encoded, expected);

  const auto decoded = decode_joy(encoded.data(), encoded.size());
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(*decoded, joy);
}

TEST(HorusLinkLightCodecsTest, TfMessageEncodesStampedTransformsWithFrameIds)
{
  const TransformStamped transform{
    1,
    2,
    "map",
    "base_link",
    Vector3{1.0F, -2.0F, 0.5F},
    Quaternion{0.0F, 0.0F, 0.0F, 1.0F}
  };

  const auto encoded = encode_tf_message({transform});
  const auto expected = load_horuslink_golden_vector("tf_message");
  EXPECT_EQ(encoded, expected);

  const auto decoded = decode_tf_message(encoded.data(), encoded.size());
  ASSERT_TRUE(decoded.has_value());
  ASSERT_EQ(decoded->size(), 1u);
  EXPECT_EQ(decoded->front(), transform);
}

TEST(HorusLinkLightCodecsTest, DecodersRejectShortBuffers)
{
  std::vector<uint8_t> bytes(light_codec::kPoseSize, 0);

  EXPECT_FALSE(decode_vector3(bytes.data(), light_codec::kVector3Size - 1).has_value());
  EXPECT_FALSE(decode_quaternion(bytes.data(), light_codec::kQuaternionSize - 1).has_value());
  EXPECT_FALSE(decode_twist(bytes.data(), light_codec::kTwistSize - 1).has_value());
  EXPECT_FALSE(decode_pose(bytes.data(), light_codec::kPoseSize - 1).has_value());
  EXPECT_FALSE(decode_pose_stamped(
    bytes.data(),
    light_codec::kPoseStampedHeaderSize - 1).has_value());
  EXPECT_FALSE(decode_path(bytes.data(), light_codec::kPathHeaderSize - 1).has_value());
  const std::vector<uint8_t> truncated_path{0x01, 0x00, 0x00, 0x00};
  EXPECT_FALSE(decode_path(truncated_path.data(), truncated_path.size()).has_value());
  EXPECT_FALSE(decode_joy(bytes.data(), light_codec::kJoyHeaderSize - 1).has_value());
  const std::vector<uint8_t> truncated_joy{0x01, 0x00, 0x00, 0x00};
  EXPECT_FALSE(decode_joy(truncated_joy.data(), truncated_joy.size()).has_value());
  EXPECT_FALSE(decode_transform_stamped(
    bytes.data(),
    light_codec::kTransformStampedHeaderSize - 1).has_value());
  const std::vector<uint8_t> truncated_tf{0x01, 0x00};
  EXPECT_FALSE(decode_tf_message(truncated_tf.data(), truncated_tf.size()).has_value());
}

TEST(HorusLinkLightCodecsTest, DecodersRejectNullPointers)
{
  EXPECT_FALSE(decode_vector3(nullptr, light_codec::kVector3Size).has_value());
  EXPECT_FALSE(decode_quaternion(nullptr, light_codec::kQuaternionSize).has_value());
  EXPECT_FALSE(decode_twist(nullptr, light_codec::kTwistSize).has_value());
  EXPECT_FALSE(decode_pose(nullptr, light_codec::kPoseSize).has_value());
  EXPECT_FALSE(decode_pose_stamped(nullptr, light_codec::kPoseStampedHeaderSize).has_value());
  EXPECT_FALSE(decode_path(nullptr, light_codec::kPathHeaderSize).has_value());
  EXPECT_FALSE(decode_joy(nullptr, light_codec::kJoyHeaderSize).has_value());
  EXPECT_FALSE(decode_transform_stamped(nullptr,
      light_codec::kTransformStampedHeaderSize).has_value());
  EXPECT_FALSE(decode_tf_message(nullptr, light_codec::kTfMessageHeaderSize).has_value());
}

}  // namespace horus_unity_bridge::horuslink
