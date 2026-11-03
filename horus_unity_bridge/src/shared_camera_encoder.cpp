// Copyright 2026 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0
#include "horus_unity_bridge/message_router.hpp"
#include "horus_unity_bridge/media_budget.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <cstdio>

namespace horus_unity_bridge
{
void MessageRouter::stop_shared_encoder(const std::shared_ptr<SharedEncoderGroup> & group)
{
  auto worker = group->worker;
  worker->running.store(false);
  worker->frame_queue_cv.notify_all();
  if (worker->worker_thread.joinable()) {worker->worker_thread.join();}
  worker->raw_sub.reset(); worker->compressed_sub.reset();
  worker->manager->stop_encoding();
}

bool MessageRouter::attach_shared_encoder(const std::shared_ptr<WebRtcSession> & session)
{
  if (!session->running.load()) {return false;}
  const int bitrate = std::max(100,
    std::min({session->requested_bitrate.load(), session->capacity_bitrate.load(),
      session->budget_bitrate.load()}) / 100 * 100);
  const int fps = session->requested_framerate.load();
  auto settings = session->source_settings;
  settings.bitrate_kbps = bitrate; settings.framerate = fps;
  const std::string key = nlohmann::json({session->stream_topic, session->image_type,
    settings.encoder, settings.pipeline, bitrate, fps}).dump();
  auto prior = std::atomic_load(&session->encoder_group);
  if (prior && prior->key == key) {return true;}
  std::shared_ptr<SharedEncoderGroup> group;
  {
    std::lock_guard<std::mutex> lock(shared_encoder_mutex_);
    if (!session->running.load()) {return false;}
    auto found = shared_encoder_groups_.find(key);
    if (found != shared_encoder_groups_.end()) {group = found->second;}
    else if (prior) {
      std::lock_guard<std::mutex> clients(prior->mutex);
      if (prior->consumers.size() == 1 && prior->consumers.count(session->session_id)) {
        shared_encoder_groups_.erase(prior->key);
        prior->key = key;
        prior->worker->manager->update_profile(bitrate, fps);
        shared_encoder_groups_[key] = prior;
        session->last_encoder_rebind_ms.store(std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count());
        group = prior;
      }
    }
    if (!group) {
      if (shared_encoder_groups_.size() >= max_shared_encoder_groups_) {return false;}
      const size_t per_camera = std::count_if(shared_encoder_groups_.begin(), shared_encoder_groups_.end(),
        [&](const auto & entry) {return entry.second->topic == session->stream_topic;});
      if (per_camera >= max_shared_encoder_groups_per_camera_) {return false;}
      group = std::make_shared<SharedEncoderGroup>();
      group->key = key; group->topic = session->stream_topic;
      auto worker = std::make_shared<WebRtcSession>();
      group->worker = worker;
      worker->session_id = "encoder:" + session->stream_topic;
      worker->stream_topic = session->stream_topic;
      worker->image_type = session->image_type;
      worker->session_start_time = std::chrono::steady_clock::now();
      worker->last_incoming_frame_time = worker->last_push_time = worker->last_encoded_progress_time =
        worker->last_keyframe_request_time = worker->last_telemetry_log_time = worker->session_start_time;
      worker->manager = std::make_shared<WebRTCManager>();
      const auto weak = std::weak_ptr<SharedEncoderGroup>(group);
      settings.encoder_only = true;
      settings.has_consumers = [weak]() {
        auto group = weak.lock();
        if (!group) {return false;}
        std::lock_guard<std::mutex> lock(group->mutex);
        for (const auto & entry : group->consumers) {
          auto peer = entry.second.lock();
          if (peer && peer->running.load() && peer->manager->is_peer_connected() &&
            peer->manager->is_video_track_open()) {return true;}
        }
        return false;
      };
      settings.encoded_output = [weak](const std::vector<uint8_t> & bytes, uint64_t capture_ns) {
        auto group = weak.lock();
        if (!group) {return;}
        std::vector<std::shared_ptr<WebRtcSession>> peers;
        {
          std::lock_guard<std::mutex> lock(group->mutex);
          for (const auto & entry : group->consumers) {
            if (auto peer = entry.second.lock(); peer && peer->running.load()) {peers.push_back(peer);}
          }
        }
        for (const auto & peer : peers) {
          std::lock_guard<std::mutex> delivery(peer->encoder_delivery_mutex);
          if (std::atomic_load(&peer->encoder_group) == group && peer->running.load()) {
            peer->manager->push_encoded_h264(bytes, "", capture_ns);
          }
        }
      };
      if (!worker->manager->initialize(settings)) {return false;}
      worker->running.store(true);
      const bool compressed = worker->image_type == "compressed" ||
        worker->stream_topic.find("/compressed") != std::string::npos;
      auto qos = rclcpp::QoS(1).best_effort().durability_volatile();
      if (compressed) {
        worker->compressed_sub = create_subscription<sensor_msgs::msg::CompressedImage>(worker->stream_topic, qos,
          [this, worker](sensor_msgs::msg::CompressedImage::SharedPtr frame) {enqueue_webrtc_compressed_frame(worker, frame);});
      } else {
        worker->raw_sub = create_subscription<sensor_msgs::msg::Image>(worker->stream_topic, qos,
          [this, worker](sensor_msgs::msg::Image::SharedPtr frame) {enqueue_webrtc_raw_frame(worker, frame);});
      }
      worker->worker_thread = std::thread(&MessageRouter::process_webrtc_frames, this, worker);
      shared_encoder_groups_[key] = group;
      RCLCPP_INFO(get_logger(), "Created shared camera encoder topic='%s' bitrate=%d fps=%d groups=%zu",
        worker->stream_topic.c_str(), bitrate, fps, shared_encoder_groups_.size());
    }
    {
      std::lock_guard<std::mutex> clients(group->mutex);
      group->consumers[session->session_id] = session;
    }
  }
  if (prior && prior != group) {release_shared_encoder(session);}
  {
    std::lock_guard<std::mutex> delivery(session->encoder_delivery_mutex);
    std::atomic_store(&session->encoder_group, group);
  }
  group->worker->manager->request_keyframe();
  session->last_encoder_rebind_ms.store(std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
  return true;
}

void MessageRouter::release_shared_encoder(const std::shared_ptr<WebRtcSession> & session)
{
  std::shared_ptr<SharedEncoderGroup> group;
  {
    std::lock_guard<std::mutex> delivery(session->encoder_delivery_mutex);
    group = std::atomic_exchange(&session->encoder_group, std::shared_ptr<SharedEncoderGroup>{});
  }
  if (!group) {return;}
  bool retire = false;
  {
    std::lock_guard<std::mutex> lock(shared_encoder_mutex_);
    std::lock_guard<std::mutex> clients(group->mutex);
    group->consumers.erase(session->session_id);
    retire = group->consumers.empty();
    if (retire) {shared_encoder_groups_.erase(group->key);}
  }
  // Keep an owner here until the worker has joined: never destroy a group on
  // its own encoder callback thread when the last viewer disconnects.
  if (retire) {stop_shared_encoder(group);}
}

void MessageRouter::refresh_shared_encoders()
{
  std::vector<std::shared_ptr<WebRtcSession>> sessions;
  {
    std::lock_guard<std::mutex> lock(webrtc_sessions_mutex_);
    for (const auto & entry : webrtc_sessions_) {
      if (entry.second->running.load()) {sessions.push_back(entry.second);}
    }
  }
  const auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  if (media_link_budget_kbps_) {
    std::vector<media::Demand> demands;
    for (const auto & session : sessions) {
      if (session->running.load()) {demands.push_back({session->session_id,
        session->requested_bitrate.load(), session->capacity_bitrate.load(), session->media_priority.load()});}
    }
    const auto budget = media::allocate_link_budget(demands, media_link_budget_kbps_,
      media_reserved_kbps_, media_bulk_floor_kbps_);
    if (budget.admissible) {
      for (const auto & session : sessions) {
        const auto found = budget.streams.find(session->session_id);
        if (found != budget.streams.end()) {session->budget_bitrate.store(found->second);}
      }
      std_msgs::msg::String message;
      nlohmann::json profiles = nlohmann::json::object();
      for (const auto & demand : demands) {
        profiles[demand.id] = {{"requested_kbps", demand.requested_kbps},
          {"capacity_kbps", demand.capacity_kbps}, {"priority", demand.weight}};
      }
      message.data = nlohmann::json({{"link_kbps", media_link_budget_kbps_},
        {"reserved_kbps", media_reserved_kbps_}, {"video_kbps", budget.video_kbps},
        {"bulk_kbps", budget.bulk_kbps}, {"streams", budget.streams}, {"profiles", profiles}}).dump();
      media_budget_status_->publish(message);
      if (!media_bulk_budget_file_.empty()) {
        std::ofstream file(media_bulk_budget_file_ + ".new", std::ios::trunc);
        if (file) {
          file << int64_t(budget.bulk_kbps) * 1000 / 8 << '\n';
          file.close();
          if (file) {std::rename((media_bulk_budget_file_ + ".new").c_str(), media_bulk_budget_file_.c_str());}
        }
      }
    }
  }
  for (const auto & session : sessions) {
    if (!std::atomic_load(&session->encoder_group)) {
      if (media_link_budget_kbps_) {
        const int target = std::min(session->requested_bitrate.load(), session->budget_bitrate.load());
        session->manager->update_profile(target, session->requested_framerate.load());
        if (session->packet_relay && native_media_relay_) {
          native_media_relay_->update_profile(session->stream_topic, session->session_id,
            target, session->requested_framerate.load(), session->media_priority.load());
        }
      }
      continue;
    }
    const bool explicit_profile = session->last_encoder_rebind_ms.load() == 0;
    if (explicit_profile || now - session->last_encoder_rebind_ms.load() >= 2000) {
      if (!attach_shared_encoder(session)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Camera profile retained: bounded source encoder capacity reached");
      }
    }
  }
}
}  // namespace horus_unity_bridge
