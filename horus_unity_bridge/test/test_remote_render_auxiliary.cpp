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
#include "horus_unity_bridge/remote_render_media_metadata.hpp"
#ifdef ENABLE_WEBRTC
#include "horus_unity_bridge/webrtc_manager.hpp"
#endif

namespace
{

using horus_unity_bridge::remote_render::chunk_auxiliary_payload;
using horus_unity_bridge::remote_render::kAuxiliaryChunkHeaderSize;
using horus_unity_bridge::remote_render::read_u32_le;
using horus_unity_bridge::remote_render::append_media_metadata;
using horus_unity_bridge::remote_render::is_h264_compressed_format;
using horus_unity_bridge::remote_render::split_media_metadata;

#ifdef ENABLE_WEBRTC
TEST(RemoteRenderKeyframes, PreencodedRecoveryReachesSourceAndCoalescesBurst)
{
  horus_unity_bridge::WebRTCManager manager;
  horus_unity_bridge::WebRTCManager::Settings settings;
  settings.preencoded_h264 = true;
  int requests = 0;
  settings.request_preencoded_keyframe = [&requests]() {++requests;};
  ASSERT_TRUE(manager.initialize(settings));
  manager.request_keyframe();
  EXPECT_EQ(requests, 1);
  for (int i = 0; i < 100; ++i) {
    manager.request_keyframe();
  }
  EXPECT_EQ(requests, 1);
}
#endif

TEST(RemoteRenderMediaMetadata, RecoversPredictionChainOnlyAtIdr)
{
  horus_unity_bridge::remote_render::H264Continuity gate;
  const std::vector<uint8_t> pframe{0, 0, 1, 0x41, 0x88};
  const std::vector<uint8_t> idr{0, 0, 0, 1, 0x65, 0x88};
  EXPECT_FALSE(gate.accept(pframe, 10));
  EXPECT_TRUE(gate.accept(idr, 11));
  EXPECT_TRUE(gate.accept(pframe, 12));
  EXPECT_FALSE(gate.accept(pframe, 14));
  EXPECT_FALSE(gate.accept(pframe, 15));
  EXPECT_TRUE(gate.accept(idr, 16));
  gate.reset();
  EXPECT_FALSE(gate.accept(pframe, 17));
  EXPECT_TRUE(gate.accept(idr, 18));
  EXPECT_TRUE(gate.accept(pframe, 19));
}

TEST(RemoteRenderMediaMetadata, RoundTripsThroughH264SeiNal)
{
  std::vector<uint8_t> encoded = {0, 0, 0, 1, 0x65, 0x88, 0x84};
  const auto original = encoded;
  const std::string metadata = R"({"version":"horus.remote_render.frame.v3","frame_id":42})";

  append_media_metadata(encoded, metadata);

  std::size_t encoded_size = 0;
  std::string decoded;
  ASSERT_TRUE(split_media_metadata(encoded, encoded_size, decoded));
  EXPECT_EQ(decoded, metadata);
  EXPECT_EQ(encoded_size, encoded.size());
  ASSERT_GT(encoded.size(), original.size());
  EXPECT_EQ(encoded[4] & 0x1fU, 6U);
  EXPECT_TRUE(std::equal(original.begin(), original.end(), encoded.end() - original.size()));
}

TEST(RemoteRenderMediaMetadata, KeepsResolutionHeadersBeforeMetadata)
{
  const std::vector<uint8_t> headers{
    0, 0, 0, 1, 0x09, 0xf0,
    0, 0, 0, 1, 0x67, 0x64, 0, 0x33, 0xac,
    0, 0, 1, 0x68, 0xee, 0x3c,
    0, 0, 1, 0x06, 0x05, 0x01, 0x55, 0x80};
  const std::vector<uint8_t> slices{0, 0, 1, 0x65, 0x88, 0x84};
  auto encoded = headers;
  encoded.insert(encoded.end(), slices.begin(), slices.end());
  append_media_metadata(encoded, "{\"frame_id\":42}");

  ASSERT_GT(encoded.size(), headers.size() + slices.size());
  EXPECT_TRUE(std::equal(headers.begin(), headers.end(), encoded.begin()));
  EXPECT_EQ(encoded[headers.size() + 4U], 0x06U);
  EXPECT_TRUE(std::equal(slices.begin(), slices.end(), encoded.end() - slices.size()));
  std::size_t size = 0U;
  std::string metadata;
  ASSERT_TRUE(split_media_metadata(encoded, size, metadata));
  EXPECT_EQ(metadata, "{\"frame_id\":42}");
}

TEST(RemoteRenderMediaMetadata, EscapesAndRestoresRbspBytes)
{
  std::vector<uint8_t> encoded = {0, 0, 0, 1, 0x41, 0x9a};
  const std::string metadata("a\0\0\1b\0\0\3c", 10U);

  append_media_metadata(encoded, metadata);

  std::size_t encoded_size = 0;
  std::string decoded;
  ASSERT_TRUE(split_media_metadata(encoded, encoded_size, decoded));
  EXPECT_EQ(decoded, metadata);
  EXPECT_EQ(encoded_size, encoded.size());
}

TEST(RemoteRenderMediaMetadata, LeavesOrdinaryH264Untouched)
{
  const std::vector<uint8_t> encoded = {0, 0, 0, 1, 0x41, 0x9a};
  std::size_t encoded_size = 0;
  std::string metadata;

  EXPECT_FALSE(split_media_metadata(encoded, encoded_size, metadata));
  EXPECT_EQ(encoded_size, encoded.size());
  EXPECT_TRUE(metadata.empty());
}

TEST(RemoteRenderMediaMetadata, RecognizesPreencodedH264Formats)
{
  EXPECT_TRUE(is_h264_compressed_format("h264"));
  EXPECT_TRUE(is_h264_compressed_format(
      "H264; stream-format=byte-stream; alignment=au"));
  EXPECT_TRUE(is_h264_compressed_format(" video/h264 ; alignment=au"));
  EXPECT_FALSE(is_h264_compressed_format("jpeg"));
  EXPECT_FALSE(is_h264_compressed_format("png"));
}

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
