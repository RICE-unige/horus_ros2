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

#include <cstring>
#include <limits>
#include <stdexcept>

namespace horus_unity_bridge::horuslink
{

namespace
{

uint32_t float_to_bits(float value)
{
  uint32_t bits = 0;
  static_assert(sizeof(bits) == sizeof(value), "Unexpected float size.");
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

float bits_to_float(uint32_t bits)
{
  float value = 0.0F;
  static_assert(sizeof(bits) == sizeof(value), "Unexpected float size.");
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

void write_float(std::vector<uint8_t> & output, float value)
{
  const uint32_t bits = float_to_bits(value);
  output.push_back(static_cast<uint8_t>(bits & 0xFFu));
  output.push_back(static_cast<uint8_t>((bits >> 8) & 0xFFu));
  output.push_back(static_cast<uint8_t>((bits >> 16) & 0xFFu));
  output.push_back(static_cast<uint8_t>((bits >> 24) & 0xFFu));
}

void write_int32(std::vector<uint8_t> & output, int32_t value)
{
  uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  output.push_back(static_cast<uint8_t>(bits & 0xFFu));
  output.push_back(static_cast<uint8_t>((bits >> 8) & 0xFFu));
  output.push_back(static_cast<uint8_t>((bits >> 16) & 0xFFu));
  output.push_back(static_cast<uint8_t>((bits >> 24) & 0xFFu));
}

void write_uint16(std::vector<uint8_t> & output, uint16_t value)
{
  output.push_back(static_cast<uint8_t>(value & 0xFFu));
  output.push_back(static_cast<uint8_t>((value >> 8) & 0xFFu));
}

void write_uint32(std::vector<uint8_t> & output, uint32_t value)
{
  output.push_back(static_cast<uint8_t>(value & 0xFFu));
  output.push_back(static_cast<uint8_t>((value >> 8) & 0xFFu));
  output.push_back(static_cast<uint8_t>((value >> 16) & 0xFFu));
  output.push_back(static_cast<uint8_t>((value >> 24) & 0xFFu));
}

float read_float(const uint8_t * data)
{
  const uint32_t bits =
    static_cast<uint32_t>(data[0]) |
    (static_cast<uint32_t>(data[1]) << 8) |
    (static_cast<uint32_t>(data[2]) << 16) |
    (static_cast<uint32_t>(data[3]) << 24);
  return bits_to_float(bits);
}

uint32_t read_uint32(const uint8_t * data)
{
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) |
         (static_cast<uint32_t>(data[3]) << 24);
}

int32_t read_int32(const uint8_t * data)
{
  const uint32_t bits =
    static_cast<uint32_t>(data[0]) |
    (static_cast<uint32_t>(data[1]) << 8) |
    (static_cast<uint32_t>(data[2]) << 16) |
    (static_cast<uint32_t>(data[3]) << 24);
  int32_t value = 0;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

uint16_t read_uint16(const uint8_t * data)
{
  return static_cast<uint16_t>(
    static_cast<uint16_t>(data[0]) |
    static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8));
}

void append_vector3(std::vector<uint8_t> & output, const Vector3 & value)
{
  write_float(output, value.x);
  write_float(output, value.y);
  write_float(output, value.z);
}

void append_quaternion(std::vector<uint8_t> & output, const Quaternion & value)
{
  write_float(output, value.x);
  write_float(output, value.y);
  write_float(output, value.z);
  write_float(output, value.w);
}

std::optional<Vector3> decode_vector3_at(const uint8_t * data, size_t size, size_t offset)
{
  if (data == nullptr || size < offset || size - offset < light_codec::kVector3Size) {
    return std::nullopt;
  }

  return Vector3{
    read_float(data + offset),
    read_float(data + offset + 4),
    read_float(data + offset + 8)
  };
}

std::optional<Quaternion> decode_quaternion_at(const uint8_t * data, size_t size, size_t offset)
{
  if (data == nullptr || size < offset || size - offset < light_codec::kQuaternionSize) {
    return std::nullopt;
  }

  return Quaternion{
    read_float(data + offset),
    read_float(data + offset + 4),
    read_float(data + offset + 8),
    read_float(data + offset + 12)
  };
}

size_t joy_size(size_t axes_count, size_t buttons_count)
{
  if (axes_count > std::numeric_limits<uint16_t>::max()) {
    throw std::out_of_range("Joy axes count exceeds the wire limit.");
  }

  if (buttons_count > std::numeric_limits<uint16_t>::max()) {
    throw std::out_of_range("Joy button count exceeds the wire limit.");
  }

  return light_codec::kJoyHeaderSize +
         (axes_count * sizeof(float)) +
         (buttons_count * sizeof(int32_t));
}

size_t frame_id_size(const std::string & frame_id)
{
  if (frame_id.size() > std::numeric_limits<uint16_t>::max()) {
    throw std::out_of_range("TF frame ID exceeds the wire limit.");
  }

  return frame_id.size();
}

size_t transform_stamped_size(const TransformStamped & value)
{
  return light_codec::kTransformStampedHeaderSize +
         frame_id_size(value.parent_frame_id) +
         frame_id_size(value.child_frame_id) +
         light_codec::kTransformStampedFixedPayloadSize;
}

size_t pose_stamped_size(const PoseStamped & value)
{
  return light_codec::kPoseStampedHeaderSize +
         frame_id_size(value.frame_id) +
         light_codec::kPoseSize;
}

}  // namespace

bool Vector3::operator==(const Vector3 & other) const
{
  return x == other.x && y == other.y && z == other.z;
}

bool Quaternion::operator==(const Quaternion & other) const
{
  return x == other.x && y == other.y && z == other.z && w == other.w;
}

bool Twist::operator==(const Twist & other) const
{
  return linear == other.linear && angular == other.angular;
}

bool Pose::operator==(const Pose & other) const
{
  return position == other.position && orientation == other.orientation;
}

bool PoseStamped::operator==(const PoseStamped & other) const
{
  return seconds == other.seconds &&
         nanoseconds == other.nanoseconds &&
         frame_id == other.frame_id &&
         pose == other.pose;
}

bool Joy::operator==(const Joy & other) const
{
  return axes == other.axes && buttons == other.buttons;
}

bool TransformStamped::operator==(const TransformStamped & other) const
{
  return seconds == other.seconds &&
         nanoseconds == other.nanoseconds &&
         parent_frame_id == other.parent_frame_id &&
         child_frame_id == other.child_frame_id &&
         translation == other.translation &&
         rotation == other.rotation;
}

std::vector<uint8_t> encode_vector3(const Vector3 & value)
{
  std::vector<uint8_t> output;
  output.reserve(light_codec::kVector3Size);
  append_vector3(output, value);
  return output;
}

std::optional<Vector3> decode_vector3(const uint8_t * data, size_t size)
{
  return decode_vector3_at(data, size, 0);
}

std::vector<uint8_t> encode_quaternion(const Quaternion & value)
{
  std::vector<uint8_t> output;
  output.reserve(light_codec::kQuaternionSize);
  append_quaternion(output, value);
  return output;
}

std::optional<Quaternion> decode_quaternion(const uint8_t * data, size_t size)
{
  return decode_quaternion_at(data, size, 0);
}

std::vector<uint8_t> encode_twist(const Twist & value)
{
  std::vector<uint8_t> output;
  output.reserve(light_codec::kTwistSize);
  append_vector3(output, value.linear);
  append_vector3(output, value.angular);
  return output;
}

std::optional<Twist> decode_twist(const uint8_t * data, size_t size)
{
  const auto linear = decode_vector3_at(data, size, 0);
  const auto angular = decode_vector3_at(data, size, light_codec::kVector3Size);
  if (!linear.has_value() || !angular.has_value()) {
    return std::nullopt;
  }

  return Twist{*linear, *angular};
}

std::vector<uint8_t> encode_pose(const Pose & value)
{
  std::vector<uint8_t> output;
  output.reserve(light_codec::kPoseSize);
  append_vector3(output, value.position);
  append_quaternion(output, value.orientation);
  return output;
}

std::optional<Pose> decode_pose(const uint8_t * data, size_t size)
{
  const auto position = decode_vector3_at(data, size, 0);
  const auto orientation = decode_quaternion_at(data, size, light_codec::kVector3Size);
  if (!position.has_value() || !orientation.has_value()) {
    return std::nullopt;
  }

  return Pose{*position, *orientation};
}

std::vector<uint8_t> encode_pose_stamped(const PoseStamped & value)
{
  std::vector<uint8_t> output;
  output.reserve(pose_stamped_size(value));
  write_uint32(output, value.seconds);
  write_uint32(output, value.nanoseconds);
  write_uint16(output, static_cast<uint16_t>(value.frame_id.size()));
  output.insert(output.end(), value.frame_id.begin(), value.frame_id.end());
  const auto encoded_pose = encode_pose(value.pose);
  output.insert(output.end(), encoded_pose.begin(), encoded_pose.end());
  return output;
}

std::optional<PoseStamped> decode_pose_stamped(const uint8_t * data, size_t size)
{
  if (data == nullptr || size < light_codec::kPoseStampedHeaderSize) {
    return std::nullopt;
  }

  const uint32_t seconds = read_uint32(data);
  const uint32_t nanoseconds = read_uint32(data + 4);
  const uint16_t frame_id_length = read_uint16(data + 8);
  const size_t payload_size =
    light_codec::kPoseStampedHeaderSize +
    frame_id_length +
    light_codec::kPoseSize;
  if (size < payload_size) {
    return std::nullopt;
  }

  size_t offset = light_codec::kPoseStampedHeaderSize;
  const std::string frame_id(
    reinterpret_cast<const char *>(data + offset),
    frame_id_length);
  offset += frame_id_length;
  const auto pose = decode_pose(data + offset, light_codec::kPoseSize);
  if (!pose.has_value()) {
    return std::nullopt;
  }

  return PoseStamped{
    seconds,
    nanoseconds,
    frame_id,
    *pose
  };
}

std::vector<uint8_t> encode_joy(const Joy & value)
{
  std::vector<uint8_t> output;
  output.reserve(joy_size(value.axes.size(), value.buttons.size()));
  write_uint16(output, static_cast<uint16_t>(value.axes.size()));
  write_uint16(output, static_cast<uint16_t>(value.buttons.size()));
  for (float axis : value.axes) {
    write_float(output, axis);
  }

  for (int32_t button : value.buttons) {
    write_int32(output, button);
  }

  return output;
}

std::optional<Joy> decode_joy(const uint8_t * data, size_t size)
{
  if (data == nullptr || size < light_codec::kJoyHeaderSize) {
    return std::nullopt;
  }

  const uint16_t axes_count = read_uint16(data);
  const uint16_t buttons_count = read_uint16(data + 2);
  const size_t payload_size = joy_size(axes_count, buttons_count);
  if (size < payload_size) {
    return std::nullopt;
  }

  Joy joy;
  joy.axes.reserve(axes_count);
  joy.buttons.reserve(buttons_count);
  size_t offset = light_codec::kJoyHeaderSize;
  for (uint16_t i = 0; i < axes_count; ++i) {
    joy.axes.push_back(read_float(data + offset));
    offset += sizeof(float);
  }

  for (uint16_t i = 0; i < buttons_count; ++i) {
    joy.buttons.push_back(read_int32(data + offset));
    offset += sizeof(int32_t);
  }

  return joy;
}

std::vector<uint8_t> encode_transform_stamped(const TransformStamped & value)
{
  std::vector<uint8_t> output;
  output.reserve(transform_stamped_size(value));
  write_uint32(output, value.seconds);
  write_uint32(output, value.nanoseconds);
  write_uint16(output, static_cast<uint16_t>(value.parent_frame_id.size()));
  write_uint16(output, static_cast<uint16_t>(value.child_frame_id.size()));
  output.insert(output.end(), value.parent_frame_id.begin(), value.parent_frame_id.end());
  output.insert(output.end(), value.child_frame_id.begin(), value.child_frame_id.end());
  append_vector3(output, value.translation);
  append_quaternion(output, value.rotation);
  return output;
}

std::optional<TransformStamped> decode_transform_stamped(
  const uint8_t * data,
  size_t size,
  size_t * bytes_read)
{
  if (bytes_read != nullptr) {
    *bytes_read = 0;
  }

  if (data == nullptr || size < light_codec::kTransformStampedHeaderSize) {
    return std::nullopt;
  }

  const uint32_t seconds = read_uint32(data);
  const uint32_t nanoseconds = read_uint32(data + 4);
  const uint16_t parent_length = read_uint16(data + 8);
  const uint16_t child_length = read_uint16(data + 10);
  const size_t payload_size =
    light_codec::kTransformStampedHeaderSize +
    parent_length +
    child_length +
    light_codec::kTransformStampedFixedPayloadSize;
  if (size < payload_size) {
    return std::nullopt;
  }

  size_t offset = light_codec::kTransformStampedHeaderSize;
  const std::string parent_frame_id(
    reinterpret_cast<const char *>(data + offset),
    parent_length);
  offset += parent_length;
  const std::string child_frame_id(
    reinterpret_cast<const char *>(data + offset),
    child_length);
  offset += child_length;
  const auto translation = decode_vector3(data + offset, light_codec::kVector3Size);
  if (!translation.has_value()) {
    return std::nullopt;
  }

  offset += light_codec::kVector3Size;
  const auto rotation = decode_quaternion(data + offset, light_codec::kQuaternionSize);
  if (!rotation.has_value()) {
    return std::nullopt;
  }

  if (bytes_read != nullptr) {
    *bytes_read = payload_size;
  }

  return TransformStamped{
    seconds,
    nanoseconds,
    parent_frame_id,
    child_frame_id,
    *translation,
    *rotation
  };
}

std::vector<uint8_t> encode_tf_message(const std::vector<TransformStamped> & transforms)
{
  if (transforms.size() > std::numeric_limits<uint16_t>::max()) {
    throw std::out_of_range("TF transform count exceeds the wire limit.");
  }

  size_t payload_size = light_codec::kTfMessageHeaderSize;
  for (const auto & transform : transforms) {
    payload_size += transform_stamped_size(transform);
  }

  std::vector<uint8_t> output;
  output.reserve(payload_size);
  write_uint16(output, static_cast<uint16_t>(transforms.size()));
  for (const auto & transform : transforms) {
    const auto encoded = encode_transform_stamped(transform);
    output.insert(output.end(), encoded.begin(), encoded.end());
  }

  return output;
}

std::optional<std::vector<TransformStamped>> decode_tf_message(const uint8_t * data, size_t size)
{
  if (data == nullptr || size < light_codec::kTfMessageHeaderSize) {
    return std::nullopt;
  }

  const uint16_t transform_count = read_uint16(data);
  std::vector<TransformStamped> transforms;
  transforms.reserve(transform_count);
  size_t offset = light_codec::kTfMessageHeaderSize;
  for (uint16_t i = 0; i < transform_count; ++i) {
    size_t bytes_read = 0;
    const auto transform = decode_transform_stamped(data + offset, size - offset, &bytes_read);
    if (!transform.has_value()) {
      return std::nullopt;
    }

    transforms.push_back(*transform);
    offset += bytes_read;
  }

  return transforms;
}

}  // namespace horus_unity_bridge::horuslink
