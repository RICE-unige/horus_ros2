// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#include "horus_unity_bridge/webrtc_manager.hpp"
#include <iostream>
#include <sstream>
#include <algorithm>
#include <cstddef>
#include <chrono>
#include <cctype>
#include <unordered_set>
#include <utility>

namespace
{

std::string to_lower_copy(const std::string& value)
{
  std::string out = value;
  std::transform(out.begin(), out.end(), out.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return out;
}

std::vector<std::string> split_space_tokens(const std::string& value)
{
  std::istringstream stream(value);
  std::vector<std::string> tokens;
  std::string token;
  while (stream >> token) {
    tokens.push_back(token);
  }
  return tokens;
}

}  // namespace

namespace horus_unity_bridge
{

WebRTCManager::WebRTCManager()
{
}

WebRTCManager::~WebRTCManager()
{
  stop_pipeline();

  if (appsrc_) {
    gst_object_unref(GST_OBJECT(appsrc_));
    appsrc_ = nullptr;
  }
  if (appsink_) {
    gst_object_unref(GST_OBJECT(appsink_));
    appsink_ = nullptr;
  }
  if (pipeline_) {
    gst_object_unref(GST_OBJECT(pipeline_));
    pipeline_ = nullptr;
  }

  if (peer_connection_) {
    peer_connection_->close();
    peer_connection_.reset();
  }
  initialized_ = false;
}

bool WebRTCManager::initialize(const Settings& settings)
{
  settings_ = settings;

  try {
    gst_init(nullptr, nullptr);
    create_peer_connection();
    if (!peer_connection_) {
      std::cerr << "Failed to create WebRTC peer connection" << std::endl;
      return false;
    }
    if (!setup_gstreamer_pipeline()) {
      std::cerr << "Failed to create GStreamer pipeline" << std::endl;
      return false;
    }
  } catch (const std::exception& e) {
    std::cerr << "WebRTC initialization failed: " << e.what() << std::endl;
    return false;
  }

  initialized_ = true;
  return true;
}

void WebRTCManager::set_signaling_callback(SignalingCallback callback)
{
  std::lock_guard<std::mutex> lock(signaling_mutex_);
  signaling_callback_ = callback;
}

void WebRTCManager::create_peer_connection()
{
  if (video_ssrc_ == 0) {
    const auto now_ticks = static_cast<uint64_t>(
      std::chrono::steady_clock::now().time_since_epoch().count());
    const auto derived_ssrc = static_cast<uint32_t>(now_ticks & 0xFFFFFFFFu);
    video_ssrc_ = (derived_ssrc == 0u) ? 1u : derived_ssrc;
  }

  config_.iceServers.clear();
  for (const auto& server : settings_.stun_servers) {
    if (server.empty()) {
      continue;
    }

    try {
      config_.iceServers.emplace_back(server);
    } catch (const std::exception& e) {
      std::cerr << "Invalid ICE server URL '" << server << "': " << e.what() << std::endl;
    }
  }

  config_.iceTransportPolicy = rtc::TransportPolicy::All;
  config_.enableIceTcp = true;
  config_.enableIceUdpMux = true;
  config_.forceMediaTransport = true;

  peer_connection_ = std::make_shared<rtc::PeerConnection>(config_);

  peer_connection_->onLocalDescription([this](rtc::Description description) {
    nlohmann::json data;
    data["sdp"] = std::string(description);
    data["type"] = description.typeString();

    const bool should_delay_answer =
      settings_.wait_for_complete_gathering &&
      description.type() == rtc::Description::Type::Answer &&
      peer_connection_ &&
      peer_connection_->gatheringState() != rtc::PeerConnection::GatheringState::Complete;

    if (should_delay_answer) {
      {
        std::lock_guard<std::mutex> lock(pending_description_mutex_);
        has_pending_local_description_ = true;
        pending_local_description_type_ = description.typeString();
        pending_local_description_data_ = data;
      }
      std::cout << "Delaying local WebRTC answer until ICE gathering completes" << std::endl;
      return;
    }

    std::lock_guard<std::mutex> signaling_lock(signaling_mutex_);
    if (signaling_callback_) {
      signaling_callback_(description.typeString(), data);
    }
  });

  peer_connection_->onLocalCandidate([this](rtc::Candidate candidate) {
    std::string candidate_sdp = std::string(candidate);
    if (candidate_sdp.rfind("a=", 0) == 0) {
      candidate_sdp = candidate_sdp.substr(2);
    }
    std::string candidate_type = "unknown";
    const std::string type_marker = " typ ";
    const auto type_pos = candidate_sdp.find(type_marker);
    if (type_pos != std::string::npos) {
      const auto start = type_pos + type_marker.size();
      auto end = candidate_sdp.find(' ', start);
      if (end == std::string::npos) {
        end = candidate_sdp.size();
      }
      if (end > start) {
        candidate_type = candidate_sdp.substr(start, end - start);
      }
    }
    std::cout << "WebRTC local candidate mid=" << candidate.mid()
              << " type=" << candidate_type
              << " sdp='" << candidate_sdp << "'" << std::endl;

    // When we are delaying the answer until ICE gathering completes, suppress
    // individual candidate signals.  All gathered candidates will be embedded
    // in the answer SDP that is published once gathering finishes.
    if (settings_.wait_for_complete_gathering) {
      std::lock_guard<std::mutex> lock(pending_description_mutex_);
      if (has_pending_local_description_) {
        return;
      }
    }

    nlohmann::json data;
    data["candidate"] = candidate_sdp;
    data["sdpMid"] = candidate.mid();
    data["sdpMLineIndex"] = 0;

    std::lock_guard<std::mutex> lock(signaling_mutex_);
    if (signaling_callback_) {
      signaling_callback_("candidate", data);
    }
  });

  peer_connection_->onStateChange([this](rtc::PeerConnection::State state) {
    std::cout << "WebRTC State: " << state << std::endl;
    switch (state) {
      case rtc::PeerConnection::State::Connected:
        peer_connected_.store(true);
        start_pipeline();
        request_keyframe();
        break;
      case rtc::PeerConnection::State::Disconnected:
      case rtc::PeerConnection::State::Failed:
      case rtc::PeerConnection::State::Closed:
        peer_connected_.store(false);
        video_track_open_.store(false);
        stop_pipeline();
        break;
      default:
        break;
    }
  });
  peer_connection_->onIceStateChange([this](rtc::PeerConnection::IceState state) {
    std::cout << "WebRTC ICE State: " << state << std::endl;
    switch (state) {
      case rtc::PeerConnection::IceState::Connected:
      case rtc::PeerConnection::IceState::Completed:
        peer_connected_.store(true);
        start_pipeline();
        request_keyframe();
        break;
      case rtc::PeerConnection::IceState::Failed:
      case rtc::PeerConnection::IceState::Disconnected:
      case rtc::PeerConnection::IceState::Closed:
        peer_connected_.store(false);
        video_track_open_.store(false);
        stop_pipeline();
        break;
      default:
        break;
    }
  });
  peer_connection_->onGatheringStateChange([this](rtc::PeerConnection::GatheringState state) {
    std::cout << "WebRTC Gathering State: " << state << std::endl;
    if (state != rtc::PeerConnection::GatheringState::Complete ||
        !settings_.wait_for_complete_gathering) {
      return;
    }

    bool emit_pending = false;
    std::string pending_type;
    nlohmann::json pending_data;
    {
      std::lock_guard<std::mutex> lock(pending_description_mutex_);
      if (has_pending_local_description_) {
        emit_pending = true;
        pending_type = pending_local_description_type_;
        pending_data = pending_local_description_data_;
        has_pending_local_description_ = false;
        pending_local_description_type_.clear();
        pending_local_description_data_.clear();
      }
    }

    if (!emit_pending) {
      return;
    }

    // Re-fetch the local description so it contains all gathered ICE
    // candidates.  The original SDP captured in onLocalDescription was
    // generated before any candidates existed.
    if (peer_connection_) {
      auto current_desc = peer_connection_->localDescription();
      if (current_desc.has_value()) {
        pending_data["sdp"] = std::string(*current_desc);
        pending_data["type"] = current_desc->typeString();
      }
    }

    std::lock_guard<std::mutex> signaling_lock(signaling_mutex_);
    if (signaling_callback_) {
      signaling_callback_(pending_type, pending_data);
      std::cout << "Published delayed WebRTC answer after ICE gathering completed" << std::endl;
    }
  });
  peer_connection_->onSignalingStateChange([](rtc::PeerConnection::SignalingState state) {
    std::cout << "WebRTC Signaling State: " << state << std::endl;
  });

  video_track_.reset();
  video_track_open_.store(false);
}

std::string WebRTCManager::trim_copy(const std::string& value)
{
  const auto first = std::find_if(value.begin(), value.end(), [](unsigned char ch) {
    return !std::isspace(ch);
  });
  if (first == value.end()) {
    return std::string();
  }
  const auto last = std::find_if(value.rbegin(), value.rend(), [](unsigned char ch) {
    return !std::isspace(ch);
  }).base();
  return std::string(first, last);
}

bool WebRTCManager::parse_payload_type_from_rtpmap(
  const std::string& line,
  const std::string& codec_name,
  int& payload_type)
{
  const std::string prefix = "a=rtpmap:";
  if (line.rfind(prefix, 0) != 0) {
    return false;
  }

  const auto space_pos = line.find(' ', prefix.size());
  if (space_pos == std::string::npos) {
    return false;
  }

  try {
    payload_type = std::stoi(line.substr(prefix.size(), space_pos - prefix.size()));
  } catch (...) {
    return false;
  }

  const std::string codec_value = to_lower_copy(trim_copy(line.substr(space_pos + 1)));
  const std::string expected_codec = to_lower_copy(codec_name);
  return codec_value.rfind(expected_codec, 0) == 0;
}

bool WebRTCManager::parse_offer_video_negotiation(
  const std::string& sdp,
  OfferVideoNegotiation& negotiation) const
{
  struct VideoSection
  {
    bool rejected = false;
    std::string mid;
    std::vector<int> mline_payload_types;
    std::vector<int> h264_payload_types;
    std::unordered_set<int> packetization_mode_1_payload_types;
  };

  std::vector<VideoSection> sections;
  VideoSection* current_section = nullptr;

  std::istringstream stream(sdp);
  std::string line;
  while (std::getline(stream, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    const std::string trimmed_line = trim_copy(line);
    if (trimmed_line.empty()) {
      continue;
    }

    if (trimmed_line.rfind("m=", 0) == 0) {
      current_section = nullptr;
      const auto tokens = split_space_tokens(trimmed_line);
      if (tokens.empty() || tokens[0] != "m=video") {
        continue;
      }

      VideoSection section;
      if (tokens.size() > 1) {
        section.rejected = tokens[1] == "0";
      }
      for (size_t idx = 3; idx < tokens.size(); ++idx) {
        try {
          section.mline_payload_types.push_back(std::stoi(tokens[idx]));
        } catch (...) {
        }
      }
      sections.push_back(std::move(section));
      current_section = &sections.back();
      continue;
    }

    if (!current_section) {
      continue;
    }

    if (trimmed_line.rfind("a=mid:", 0) == 0) {
      current_section->mid = trim_copy(trimmed_line.substr(6));
      continue;
    }

    int rtpmap_pt = -1;
    if (parse_payload_type_from_rtpmap(trimmed_line, "h264/90000", rtpmap_pt)) {
      current_section->h264_payload_types.push_back(rtpmap_pt);
      continue;
    }

    const std::string fmtp_prefix = "a=fmtp:";
    if (trimmed_line.rfind(fmtp_prefix, 0) == 0) {
      const auto space_pos = trimmed_line.find(' ', fmtp_prefix.size());
      if (space_pos == std::string::npos) {
        continue;
      }

      int fmtp_pt = -1;
      try {
        fmtp_pt = std::stoi(trimmed_line.substr(fmtp_prefix.size(), space_pos - fmtp_prefix.size()));
      } catch (...) {
        continue;
      }
      const std::string fmtp_params = to_lower_copy(trim_copy(trimmed_line.substr(space_pos + 1)));
      if (fmtp_params.find("packetization-mode=1") != std::string::npos) {
        current_section->packetization_mode_1_payload_types.insert(fmtp_pt);
      }
    }
  }

  const VideoSection* selected_section = nullptr;
  for (const auto& section : sections) {
    if (!section.rejected && !section.h264_payload_types.empty()) {
      selected_section = &section;
      break;
    }
  }
  if (!selected_section) {
    for (const auto& section : sections) {
      if (!section.rejected) {
        selected_section = &section;
        break;
      }
    }
  }

  if (!selected_section || selected_section->h264_payload_types.empty()) {
    return false;
  }

  std::unordered_set<int> h264_set(
    selected_section->h264_payload_types.begin(),
    selected_section->h264_payload_types.end());
  std::unordered_set<int> unique_payloads;
  std::vector<int> ordered_h264_payloads;

  for (const int payload_type : selected_section->mline_payload_types) {
    if (h264_set.count(payload_type) == 0) {
      continue;
    }
    if (unique_payloads.insert(payload_type).second) {
      ordered_h264_payloads.push_back(payload_type);
    }
  }
  for (const int payload_type : selected_section->h264_payload_types) {
    if (unique_payloads.insert(payload_type).second) {
      ordered_h264_payloads.push_back(payload_type);
    }
  }

  if (ordered_h264_payloads.empty()) {
    return false;
  }

  int selected_payload_type = -1;
  for (const int payload_type : ordered_h264_payloads) {
    if (selected_section->packetization_mode_1_payload_types.count(payload_type) > 0) {
      selected_payload_type = payload_type;
      break;
    }
  }
  if (selected_payload_type == -1) {
    selected_payload_type = ordered_h264_payloads.front();
  }

  negotiation.mid = selected_section->mid.empty() ? "0" : selected_section->mid;
  negotiation.h264_payload_types = std::move(ordered_h264_payloads);
  negotiation.preferred_payload_type = selected_payload_type;
  return true;
}

bool WebRTCManager::apply_video_payload_type(int payload_type)
{
  if (payload_type < 0 || payload_type > 127) {
    std::cerr << "Invalid video payload type requested: " << payload_type << std::endl;
    return false;
  }

  if (video_payload_type_ != payload_type) {
    std::cout << "Updating GStreamer/WebRTC payload type from "
              << video_payload_type_ << " to " << payload_type << std::endl;
  }
  video_payload_type_ = payload_type;

  if (!pipeline_) {
    return true;
  }

  GstElement* payloader = gst_bin_get_by_name(GST_BIN(pipeline_), "mypayloader");
  if (!payloader) {
    std::cerr << "Failed to locate rtph264pay element 'mypayloader' while applying payload type "
              << video_payload_type_ << std::endl;
    return false;
  }

  g_object_set(payloader, "pt", video_payload_type_, nullptr);
  gst_object_unref(payloader);
  return true;
}

bool WebRTCManager::configure_video_track_for_offer(const OfferVideoNegotiation& negotiation)
{
  if (!peer_connection_) {
    return false;
  }
  if (negotiation.mid.empty() || negotiation.preferred_payload_type < 0) {
    return false;
  }

  if (!apply_video_payload_type(negotiation.preferred_payload_type)) {
    return false;
  }

  if (video_track_) {
    video_track_.reset();
  }
  video_track_open_.store(false);

  rtc::Description::Video media(negotiation.mid, rtc::Description::Direction::SendOnly);
  media.addH264Codec(negotiation.preferred_payload_type);
  media.addSSRC(video_ssrc_, "horus-video");
  video_track_ = peer_connection_->addTrack(media);
  if (!video_track_) {
    std::cerr << "Failed to create WebRTC video track for mid='" << negotiation.mid << "'" << std::endl;
    return false;
  }

  negotiated_video_mid_ = negotiation.mid;
  rtp_packets_sent_.store(0);
  rtp_bytes_sent_.store(0);
  first_rtp_logged_.store(false);
  {
    std::lock_guard<std::mutex> lock(rtp_metrics_mutex_);
    first_rtp_sent_timestamp_ = std::chrono::steady_clock::time_point{};
  }

  video_track_->onOpen([this]() {
    video_track_open_.store(true);
    std::cout << "WebRTC video track open (mid='" << negotiated_video_mid_
              << "', pt=" << video_payload_type_ << ")" << std::endl;
    request_keyframe();
  });
  video_track_->onClosed([this]() {
    video_track_open_.store(false);
    std::cout << "WebRTC video track closed" << std::endl;
  });
  return true;
}

bool WebRTCManager::handle_offer(const std::string& sdp)
{
  if (!initialized_ || !peer_connection_) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(pending_description_mutex_);
    has_pending_local_description_ = false;
    pending_local_description_type_.clear();
    pending_local_description_data_.clear();
  }

  std::cout << "Received WebRTC Offer" << std::endl;

  OfferVideoNegotiation negotiation;
  if (!parse_offer_video_negotiation(sdp, negotiation)) {
    std::cerr << "Failed to parse offer video section with H264 payload negotiation" << std::endl;
    return false;
  }

  std::ostringstream payloads;
  for (size_t idx = 0; idx < negotiation.h264_payload_types.size(); ++idx) {
    if (idx > 0) {
      payloads << ",";
    }
    payloads << negotiation.h264_payload_types[idx];
  }

  std::cout << "Negotiated WebRTC video from offer: mid='" << negotiation.mid
            << "' h264_pts=[" << payloads.str()
            << "] selected_pt=" << negotiation.preferred_payload_type << std::endl;

  if (!configure_video_track_for_offer(negotiation)) {
    std::cerr << "Failed to configure WebRTC video track for offer mid='"
              << negotiation.mid << "'" << std::endl;
    return false;
  }

  try {
    peer_connection_->setRemoteDescription(rtc::Description(sdp, "offer"));
  } catch (const std::exception& e) {
    std::cerr << "Failed to apply WebRTC offer remote description: " << e.what() << std::endl;
    return false;
  }
  return true;
}

void WebRTCManager::handle_answer(const std::string& sdp)
{
  if (!initialized_) {
    return;
  }
  std::cout << "Received WebRTC Answer" << std::endl;
  peer_connection_->setRemoteDescription(rtc::Description(sdp, "answer"));
}

void WebRTCManager::handle_candidate(const std::string& candidate, const std::string& sdp_mid, int sdp_mline_index)
{
  if (!initialized_) {
    return;
  }
  std::string normalized_candidate = candidate;
  if (normalized_candidate.rfind("a=", 0) == 0) {
    normalized_candidate = normalized_candidate.substr(2);
  }

  std::cout << "Received WebRTC Candidate (mid=" << sdp_mid
            << ", mline=" << sdp_mline_index << ")" << std::endl;
  std::vector<std::string> mids_to_try;
  if (!sdp_mid.empty()) {
    mids_to_try.push_back(sdp_mid);
  }
  if (!negotiated_video_mid_.empty()) {
    mids_to_try.push_back(negotiated_video_mid_);
  }
  if (video_track_) {
    try {
      const auto track_mid = video_track_->mid();
      if (!track_mid.empty()) {
        mids_to_try.push_back(track_mid);
      }
    } catch (...) {
    }
  }
  mids_to_try.push_back("0");
  mids_to_try.push_back("video");

  auto dedupe = [](std::vector<std::string>& mids) {
    std::vector<std::string> unique;
    for (const auto& mid : mids) {
      if (mid.empty()) {
        continue;
      }
      if (std::find(unique.begin(), unique.end(), mid) == unique.end()) {
        unique.push_back(mid);
      }
    }
    mids.swap(unique);
  };
  dedupe(mids_to_try);

  std::string first_error;
  for (const auto& mid : mids_to_try) {
    try {
      peer_connection_->addRemoteCandidate(rtc::Candidate(normalized_candidate, mid));
      if (mid != sdp_mid) {
        std::cout << "Accepted WebRTC Candidate using fallback mid='"
                  << mid << "'" << std::endl;
      }
      return;
    } catch (const std::exception& e) {
      if (first_error.empty()) {
        first_error = e.what();
      }
    }
  }

  if (!first_error.empty()) {
    std::cerr << "Failed to add remote candidate: " << first_error << std::endl;
  }
}

bool WebRTCManager::setup_gstreamer_pipeline()
{
  auto has_factory = [](const char * factory_name) -> bool {
      if (!factory_name || !*factory_name) {
        return false;
      }
      GstElementFactory * factory = gst_element_factory_find(factory_name);
      if (!factory) {
        return false;
      }
      gst_object_unref(factory);
      return true;
    };

  auto build_pipeline = [this](const std::string & encoder_stage) {
      std::ostringstream builder;
      builder << "appsrc name=mysource format=time is-live=true do-timestamp=true ! ";
      builder << "videoconvert ! ";
      builder << encoder_stage << " ! ";
      // Force constrained-baseline profile via caps to ensure Unity/mobile compatibility
      // This works for any H264 encoder (x264enc, nvh264enc, etc.)
      builder << "video/x-h264,profile=constrained-baseline ! ";
      builder << "h264parse ! ";
      builder << "rtph264pay name=mypayloader config-interval=1 pt=" << video_payload_type_;
      builder << " ssrc=" << video_ssrc_ << " mtu=1200 ! ";
      builder << "appsink name=mysink emit-signals=true sync=false";
      return builder.str();
    };

  std::string pipeline_str;
  if (!settings_.pipeline.empty()) {
    pipeline_str = settings_.pipeline;
    GError * error = nullptr;
    pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
    if (error || !pipeline_) {
      std::cerr << "Failed to create pipeline: "
                << (error ? error->message : "unknown error") << std::endl;
      if (error) {
        g_error_free(error);
      }
      if (pipeline_) {
        gst_object_unref(GST_OBJECT(pipeline_));
      }
      pipeline_ = nullptr;
      return false;
    }
  } else {
    const int bitrate_kbps = std::max(1, settings_.bitrate_kbps);
    const int framerate = std::max(1, settings_.framerate);
    const int bitrate_bps = bitrate_kbps * 1000;

    std::vector<std::pair<std::string, std::string>> encoder_candidates;
    auto add_candidate = [&encoder_candidates](
      const std::string & label, const std::string & stage)
      {
        for (const auto & existing : encoder_candidates) {
          if (existing.second == stage) {
            return;
          }
        }
        encoder_candidates.emplace_back(label, stage);
      };

    // If a specific encoder was requested, try it first.
    const std::string requested_encoder = settings_.encoder;
    if (!requested_encoder.empty() && requested_encoder != "auto") {
      const bool simple_factory_name =
        requested_encoder.find(' ') == std::string::npos &&
        requested_encoder.find('!') == std::string::npos;
      if (!simple_factory_name) {
        // Complex pipeline stage (e.g. "x264enc bitrate=500") – use as-is.
        add_candidate("requested", requested_encoder);
      } else if (has_factory(requested_encoder.c_str())) {
        // Simple factory name – build a proper stage with configured
        // bitrate / framerate / low-latency tuning so we don't fall back
        // to high-latency x264 defaults (B-frames, slow preset, etc.).
        std::string requested_stage;
        if (requested_encoder == "x264enc") {
          std::ostringstream s;
          // tune=zerolatency is essential for real-time streaming (disables B-frames).
          // We don't set profile here because x264enc might not have the property;
          // we enforce it via caps in build_pipeline instead.
          s << "x264enc tune=zerolatency speed-preset=ultrafast bitrate="
            << bitrate_kbps << " key-int-max=" << framerate;
          requested_stage = s.str();
        } else if (requested_encoder == "openh264enc") {
          std::ostringstream s;
          s << "openh264enc bitrate=" << bitrate_bps;
          requested_stage = s.str();
        } else if (requested_encoder == "nvh264enc") {
          requested_stage = "nvh264enc preset=low-latency-hp";
        } else {
          requested_stage = requested_encoder;
        }
        add_candidate("requested", requested_stage);
      } else {
        std::cout << "Requested encoder '" << requested_encoder
                  << "' is not available. Trying fallbacks." << std::endl;
      }
    }

    // Hardware + software fallback chain.
    if (has_factory("nvh264enc")) {
      add_candidate("nvh264enc", "nvh264enc preset=low-latency-hp");
    }
    if (has_factory("vaapih264enc")) {
      add_candidate("vaapih264enc", "vaapih264enc");
    }
    if (has_factory("v4l2h264enc")) {
      add_candidate("v4l2h264enc", "v4l2h264enc");
    }
    if (has_factory("openh264enc")) {
      std::ostringstream stage;
      stage << "openh264enc bitrate=" << bitrate_bps;
      add_candidate("openh264enc", stage.str());
    }
    if (has_factory("x264enc")) {
      std::ostringstream stage;
      stage << "x264enc tune=zerolatency speed-preset=ultrafast bitrate=" << bitrate_kbps
            << " key-int-max=" << framerate;
      add_candidate("x264enc", stage.str());
    }
    if (has_factory("avenc_h264")) {
      add_candidate("avenc_h264", "avenc_h264");
    }

    if (encoder_candidates.empty()) {
      std::cerr << "No supported H264 encoder found (tried nvh264enc/vaapih264enc/v4l2h264enc/openh264enc/x264enc/avenc_h264)."
                << std::endl;
      return false;
    }

    for (const auto & candidate : encoder_candidates) {
      pipeline_str = build_pipeline(candidate.second);
      std::cout << "Trying WebRTC encoder: " << candidate.first
                << " [" << candidate.second << "]" << std::endl;

      GError * error = nullptr;
      GstElement * trial_pipeline = gst_parse_launch(pipeline_str.c_str(), &error);
      if (error || !trial_pipeline) {
        std::cerr << "Encoder '" << candidate.first << "' failed: "
                  << (error ? error->message : "unknown error") << std::endl;
        if (error) {
          g_error_free(error);
        }
        if (trial_pipeline) {
          gst_object_unref(GST_OBJECT(trial_pipeline));
        }
        continue;
      }

      pipeline_ = trial_pipeline;
      std::cout << "Selected WebRTC encoder: " << candidate.first << std::endl;
      break;
    }

    if (!pipeline_) {
      std::cerr << "Failed to create GStreamer pipeline with all encoder candidates." << std::endl;
      return false;
    }
  }

  appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "mysource");
  appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "mysink");
  if (!appsrc_ || !appsink_) {
    std::cerr << "Failed to acquire appsrc/appsink from pipeline" << std::endl;
    return false;
  }

  if (!apply_video_payload_type(video_payload_type_)) {
    std::cerr << "Failed to apply negotiated RTP payload type to GStreamer payloader" << std::endl;
    return false;
  }

  // Configure appsink callback
  g_signal_connect(appsink_, "new-sample", G_CALLBACK(on_new_sample), this);
  return true;
}

void WebRTCManager::start_pipeline()
{
  std::lock_guard<std::mutex> lock(pipeline_mutex_);
  if (pipeline_ && !pipeline_started_) {
    gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    pipeline_started_ = true;
  }
}

void WebRTCManager::stop_pipeline()
{
  std::lock_guard<std::mutex> lock(pipeline_mutex_);
  if (pipeline_) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    pipeline_started_ = false;
  }
}

void WebRTCManager::push_frame(const std::vector<uint8_t>& data, int width, int height, const std::string& format)
{
  std::lock_guard<std::mutex> lock(frame_mutex_);

  if (!appsrc_ || !pipeline_ || !pipeline_started_) {
    return;
  }

  guint size = data.size();
  GstBuffer* buffer = gst_buffer_new_allocate(nullptr, size, nullptr);
  if (!buffer) {
    std::cerr << "Failed to allocate GStreamer buffer" << std::endl;
    return;
  }

  gst_buffer_fill(buffer, 0, data.data(), size);

  // Set caps only if they changed
  if (width != last_width_ || height != last_height_ || format != last_format_) {
    GstCaps* caps = gst_caps_new_simple("video/x-raw",
      "format", G_TYPE_STRING, format.c_str(),
      "width", G_TYPE_INT, width,
      "height", G_TYPE_INT, height,
      "framerate", GST_TYPE_FRACTION, std::max(1, settings_.framerate), 1,
      nullptr);

    gst_app_src_set_caps(GST_APP_SRC(appsrc_), caps);
    gst_caps_unref(caps);

    last_width_ = width;
    last_height_ = height;
    last_format_ = format;
  }

  GstFlowReturn ret;
  g_signal_emit_by_name(appsrc_, "push-buffer", buffer, &ret);
  gst_buffer_unref(buffer);

  if (ret != GST_FLOW_OK) {
    std::cerr << "Failed to push frame to GStreamer pipeline: " << ret << std::endl;
  }

  static int frame_count = 0;
  if (++frame_count % 30 == 0) {
      std::cout << "Pushed " << frame_count << " frames to GStreamer pipeline" << std::endl;
  }
}

GstFlowReturn WebRTCManager::on_new_sample(GstElement* sink, gpointer user_data)
{
  WebRTCManager* manager = static_cast<WebRTCManager*>(user_data);

  static int sample_count = 0;
  if (++sample_count % 30 == 0) {
      std::cout << "Pulled " << sample_count << " samples from GStreamer pipeline" << std::endl;
  }

  GstSample* sample;
  g_signal_emit_by_name(sink, "pull-sample", &sample);

  if (sample) {
    GstBuffer* buffer = gst_sample_get_buffer(sample);
    if (buffer && manager->video_track_) {
      if (!manager->peer_connected_.load() || !manager->video_track_open_.load() ||
          manager->peer_connection_->state() != rtc::PeerConnection::State::Connected ||
          !manager->video_track_->isOpen()) {
        gst_sample_unref(sample);
        return GST_FLOW_OK;
      }
      GstMapInfo info;
      if (gst_buffer_map(buffer, &info, GST_MAP_READ)) {
        bool sent_packet = false;
        try {
          // libdatachannel expects RTP packets for media tracks.
          auto* data_ptr = reinterpret_cast<const std::byte*>(info.data);
          manager->video_track_->send(data_ptr, info.size);
          sent_packet = true;
        } catch (const std::exception& e) {
          const auto now = std::chrono::steady_clock::now();
          if (now - manager->last_track_closed_log_time_ > std::chrono::seconds(2)) {
            std::cerr << "Failed to send RTP packet: " << e.what() << std::endl;
            manager->last_track_closed_log_time_ = now;
          }
          if (std::string(e.what()).find("Track is closed") != std::string::npos) {
            manager->video_track_open_.store(false);
          }
        }

        if (sent_packet) {
          const uint64_t packets_sent = manager->rtp_packets_sent_.fetch_add(1) + 1;
          const uint64_t bytes_sent = manager->rtp_bytes_sent_.fetch_add(info.size) + info.size;

          if (!manager->first_rtp_logged_.exchange(true)) {
            const auto now = std::chrono::steady_clock::now();
            {
              std::lock_guard<std::mutex> lock(manager->rtp_metrics_mutex_);
              manager->first_rtp_sent_timestamp_ = now;
            }

            if (info.size >= 12) {
              const auto payload_type = static_cast<int>(info.data[1] & 0x7F);
              const auto sequence = static_cast<uint16_t>(
                (static_cast<uint16_t>(info.data[2]) << 8) |
                static_cast<uint16_t>(info.data[3]));
              const auto ssrc = static_cast<uint32_t>(
                (static_cast<uint32_t>(info.data[8]) << 24) |
                (static_cast<uint32_t>(info.data[9]) << 16) |
                (static_cast<uint32_t>(info.data[10]) << 8) |
                static_cast<uint32_t>(info.data[11]));
              std::cout << "First RTP packet sent: mid='" << manager->negotiated_video_mid_
                        << "' pt=" << payload_type
                        << " ssrc=" << ssrc
                        << " seq=" << sequence
                        << " expected_pt=" << manager->video_payload_type_ << std::endl;
            } else {
              std::cout << "First RTP packet sent: packet too short for header decode (size="
                        << info.size << ")" << std::endl;
            }
          }

          if (packets_sent % 30 == 0) {
            std::cout << "Sent " << packets_sent << " RTP packets ("
                      << bytes_sent << " bytes total)" << std::endl;
          }
        }
        gst_buffer_unmap(buffer, &info);
      }
    }
    gst_sample_unref(sample);
    return GST_FLOW_OK;
  }
  return GST_FLOW_ERROR;
}

void WebRTCManager::request_keyframe()
{
  // Send the keyframe request to the sink so it travels UPSTREAM to the encoder.
  // Sending it to appsrc (source) would try to send it upstream into the void.
  if (!pipeline_ || !appsink_) return;

  std::cout << "Requesting IDR keyframe from encoder..." << std::endl;

  GstStructure* s = gst_structure_new_empty("GstForceKeyUnit");
  gst_structure_set(s, "all-headers", G_TYPE_BOOLEAN, TRUE, nullptr);

  GstEvent* event = gst_event_new_custom(GST_EVENT_CUSTOM_UPSTREAM, s);
  if (!gst_element_send_event(appsink_, event)) {
      std::cerr << "Failed to send keyframe request event" << std::endl;
  }
}

uint64_t WebRTCManager::get_rtp_packets_sent() const
{
  return rtp_packets_sent_.load();
}

uint64_t WebRTCManager::get_rtp_bytes_sent() const
{
  return rtp_bytes_sent_.load();
}

bool WebRTCManager::is_peer_connected() const
{
  return peer_connected_.load();
}

bool WebRTCManager::is_video_track_open() const
{
  return video_track_open_.load();
}

} // namespace horus_unity_bridge
