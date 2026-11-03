// Copyright 2025 RICE Lab, University of Genoa
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

// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <mutex>
#include <thread>
#include <atomic>
#include <cstdint>
#include <chrono>
#include <deque>
#include <unordered_map>

#include <rtc/rtc.hpp>
#include <nlohmann/json.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <rtc/h264rtppacketizer.hpp>
#include <rtc/rtcpsrreporter.hpp>
#include "horus_unity_bridge/gst_bandwidth_pacer.hpp"

namespace horus_unity_bridge
{

class WebRTCManager
{
public:
  using SignalingCallback = std::function<void(const std::string & type,
      const nlohmann::json & data)>;
  struct Settings
  {
    std::vector<std::string> stun_servers;
    std::string encoder = "x264enc";
    int bitrate_kbps = 2000;
    int framerate = 30;
    std::string pipeline;
    bool wait_for_complete_gathering = true;
    bool preencoded_h264 = false;
    bool packet_relay = false;
    std::function<void()> request_preencoded_keyframe;
    std::function<void(int)> bandwidth_estimate;
    bool encoder_only = false;
    std::function<bool()> has_consumers;
    std::function<void(const std::vector<uint8_t> &, uint64_t)> encoded_output;
  };

  WebRTCManager();
  ~WebRTCManager();

  // Initialize GStreamer and WebRTC configuration
  bool initialize(const Settings & settings);

  // Set callback for sending signaling messages back to the client via TCP
  void set_signaling_callback(SignalingCallback callback);

  // Handle incoming signaling messages from the client
  bool handle_offer(const std::string & sdp);
  void handle_answer(const std::string & sdp);
  void handle_candidate(
    const std::string & candidate, const std::string & sdp_mid,
    int sdp_mline_index);

  // Feed video frame to the streaming pipeline
  bool push_frame(
    const std::vector<uint8_t> & data, int width, int height,
    const std::string & format,
    const std::string & remote_render_metadata = "");

  // Submit one Annex-B H.264 access unit that was encoded by the render GPU.
  bool push_encoded_h264(
    const std::vector<uint8_t> & access_unit,
    const std::string & remote_render_metadata = "", uint64_t capture_time_ns = 0);

  bool push_rtp(const uint8_t * packet, size_t size, uint64_t source_generation);
  void update_profile(int bitrate_kbps, int framerate);
  void stop_encoding();

  // Send synchronized remote-render metadata/depth over the negotiated data channel.
  bool send_auxiliary_data(const std::vector<uint8_t> & data);

  // Request a keyframe from the encoder (useful for packet loss recovery)
  void request_keyframe();

  uint64_t get_encoded_units_sent() const;
  uint64_t get_encoded_bytes_sent() const;
  uint64_t get_metadata_inputs_received() const;
  uint64_t get_metadata_pts_matches() const;
  uint64_t get_metadata_fifo_fallbacks() const;
  uint64_t get_metadata_unmatched() const;
  uint64_t get_metadata_selfcheck_failures() const;
  bool is_peer_connected() const;
  bool is_video_track_open() const;
  nlohmann::json get_transport_statistics() const;

private:
  struct OfferVideoNegotiation
  {
    std::string mid;
    std::vector<int> h264_payload_types;
    int preferred_payload_type = -1;
    int twcc_extension = 0;
  };

  // Convert an encoded buffer's presentation time into 90 kHz RTP ticks
  // relative to the first frame of this session. Falls back to a nominal
  // per-frame advance only when the buffer carries no usable timestamp.
  uint32_t resolve_frame_timestamp_ticks(GstBuffer * buffer);
  uint32_t resolve_direct_frame_timestamp_ticks();

  Settings settings_;
  bool initialized_ = false;
  uint32_t video_ssrc_ = 0;
  int video_payload_type_ = 96;
  std::string negotiated_video_mid_ = "video";
  // WebRTC components
  std::shared_ptr<rtc::PeerConnection> peer_connection_;
  std::shared_ptr<rtc::Track> video_track_;
  std::shared_ptr<rtc::DataChannel> data_channel_;
  std::shared_ptr<rtc::RtpPacketizationConfig> rtp_config_;
  std::shared_ptr<rtc::RtcpSrReporter> sr_reporter_;
  std::shared_ptr<GstBandwidthPacer> bandwidth_pacer_;

  // GStreamer pipeline
  GstElement * pipeline_ = nullptr;
  GstElement * appsrc_ = nullptr;
  GstElement * appsink_ = nullptr;
  gulong appsink_signal_handler_id_ = 0;
  std::atomic<bool> pipeline_started_{false};
  std::atomic<bool> shutting_down_{false};
  std::atomic<int64_t> last_keyframe_request_ms_{0};
  std::atomic<bool> pending_keyframe_request_{false};
  std::mutex frame_mutex_;
  std::mutex frame_metadata_mutex_;
  std::unordered_map<uint64_t, std::string> frame_metadata_by_pts_;
  std::deque<uint64_t> frame_metadata_order_;
  std::chrono::steady_clock::time_point first_input_time_{};
  uint64_t last_input_pts_ns_ = 0;
  uint64_t next_input_deadline_ns_ = 0;
  bool has_first_input_time_ = false;
  std::mutex sample_callback_mutex_;
  std::atomic<bool> peer_connected_{false};
  std::atomic<bool> video_track_open_{false};
  std::atomic<bool> data_channel_open_{false};
  std::atomic<uint64_t> encoded_units_sent_{0};
  std::atomic<uint64_t> encoded_bytes_sent_{0};
  std::atomic<uint64_t> metadata_inputs_received_{0};
  std::atomic<uint64_t> metadata_pts_matches_{0};
  std::atomic<uint64_t> metadata_fifo_fallbacks_{0};
  std::atomic<uint64_t> metadata_unmatched_{0};
  std::atomic<uint64_t> metadata_selfcheck_failures_{0};
  std::atomic<uint64_t> auxiliary_units_sent_{0};
  std::atomic<uint64_t> auxiliary_bytes_sent_{0};
  std::atomic<uint64_t> auxiliary_units_dropped_{0};
  std::atomic<bool> first_encoded_unit_logged_{false};
  uint64_t first_frame_pts_ns_ = 0;
  bool has_first_frame_pts_ = false;
  uint32_t last_frame_timestamp_ticks_ = 0;
  std::chrono::steady_clock::time_point first_direct_frame_time_{};
  bool has_first_direct_frame_time_ = false;
  uint32_t last_direct_frame_timestamp_ticks_ = 0;
  uint64_t first_capture_time_ns_ = 0;
  uint64_t relay_generation_ = 0;
  uint16_t relay_sequence_offset_ = 0, relay_last_sequence_ = 0;
  uint32_t relay_timestamp_offset_ = 0, relay_last_timestamp_ = 0;
  bool relay_started_ = false;
  uint32_t relay_last_report_timestamp_ = 0;
  std::atomic<int> target_bitrate_kbps_{2000}, target_framerate_{30};
  std::atomic<int> congestion_bitrate_kbps_{100000};
  int applied_bitrate_kbps_ = 0;
  std::chrono::steady_clock::time_point last_track_closed_log_time_{};

  // Caps caching to avoid redundant GStreamer calls
  int last_width_ = 0;
  int last_height_ = 0;
  int last_framerate_ = 0;
  std::string last_format_;

  SignalingCallback signaling_callback_;
  std::mutex signaling_mutex_;
  std::mutex data_channel_mutex_;
  std::mutex pending_description_mutex_;
  bool has_pending_local_description_ = false;
  std::string pending_local_description_type_;
  nlohmann::json pending_local_description_data_;
  std::mutex pipeline_mutex_;

  // Configuration
  rtc::Configuration config_;

  // Internal helpers
  void create_peer_connection();
  bool parse_offer_video_negotiation(
    const std::string & sdp,
    OfferVideoNegotiation & negotiation) const;
  bool configure_video_track_for_offer(const OfferVideoNegotiation & negotiation);
  bool apply_video_payload_type(int payload_type);
  static bool parse_payload_type_from_rtpmap(
    const std::string & line,
    const std::string & codec_name,
    int & payload_type);
  static std::string trim_copy(const std::string & value);
  bool setup_gstreamer_pipeline();
  void start_pipeline();
  void stop_pipeline();
  static GstFlowReturn on_new_sample(GstElement * sink, gpointer user_data);
};

} // namespace horus_unity_bridge
