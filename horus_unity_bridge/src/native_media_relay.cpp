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

#include "horus_unity_bridge/native_media_relay.hpp"
#include "horus_unity_bridge/rtp_relay.hpp"
#include <std_msgs/msg/string.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/webrtc/webrtc.h>
#include <gst/sdp/sdp.h>
#include <gst/video/video-event.h>
#include <atomic>
#include <fstream>
#include <map>
#include <mutex>
#include <chrono>
#include <vector>
#include <utility>
#include <stdexcept>
#include <set>
#include <regex>

namespace horus_unity_bridge
{
namespace
{
using Clock = std::chrono::steady_clock;
int64_t monotonic_ms()
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    Clock::now().time_since_epoch()).count();
}

struct RelaySource : std::enable_shared_from_this<RelaySource>
{
  nlohmann::json config;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr signal;
  mutable std::mutex mutex;
  std::mutex lifecycle_mutex, handlers_mutex;
  std::map<std::string, NativeMediaRelay::PacketCallback> consumers;
  std::map<std::string, std::pair<int, int>> profiles;
  std::map<std::string, int> priorities;
  std::map<std::string, int> capacities;
  std::vector<nlohmann::json> candidates;
  uint64_t local_relay_candidates = 0, local_other_candidates = 0;
  media::RtpSequenceGaps sequence_gaps;
  std::atomic<uint64_t> upstream_gap_events{0}, upstream_missing_packets{0}, keyframe_requests{0};
  GstElement * pipeline = nullptr;
  GstElement * peer = nullptr;
  GstElement * sink = nullptr;
  std::vector<std::pair<GObject *, gulong>> handlers;
  std::string session, offer, answer;
  std::atomic<uint64_t> generation{0}, packets{0}, frames{0}, bytes{0}, failures{0};
  std::atomic<int64_t> last_packet_ms{0}, last_keyframe_ms{0};
  int64_t started = 0, retry_at = 0, idle_since = 0, last_offer = 0;
  unsigned retries = 0;
  std::atomic<bool> connected{false};
  std::atomic<bool> needs_keyframe{true};
  bool profile_supported = false;
  int last_profile_bitrate = 0, last_profile_fps = 0, last_profile_priority = 0;
  std::atomic<bool> profile_dirty{true};

  struct Context
  {
    std::weak_ptr<RelaySource> source;
    uint64_t generation;
  };
  static std::shared_ptr<RelaySource> current(gpointer data)
  {
    auto * context = static_cast<Context *>(data);
    auto source = context->source.lock();
    return source && source->generation.load() == context->generation ? source : nullptr;
  }
  void connect(GObject * object, const char * name, GCallback callback, uint64_t expected = 0)
  {
    std::lock_guard<std::mutex> lock(handlers_mutex);
    if (expected && expected != generation.load()) {return;}
    auto * context = new Context{weak_from_this(), expected ? expected : generation.load()};
    auto id = g_signal_connect_data(object, name, callback, context,
      [](gpointer p, GClosure *) {delete static_cast<Context *>(p);}, GConnectFlags(0));
    handlers.emplace_back(object, id);
  }
  void send(nlohmann::json payload)
  {
    payload["session_id"] = session;
    std_msgs::msg::String message;
    message.data = payload.dump();
    try {signal->publish(message);}
    catch (const std::exception &) {failures.fetch_add(1);}
  }
  static GstFlowReturn sample(GstElement * element, gpointer data)
  {
    GstSample * value = gst_app_sink_pull_sample(GST_APP_SINK(element));
    if (!value) {return GST_FLOW_OK;}
    auto source = current(data);
    GstBuffer * buffer = gst_sample_get_buffer(value);
    GstMapInfo mapped;
    if (source && buffer && gst_buffer_map(buffer, &mapped, GST_MAP_READ)) {
      auto packet = media::parse_rtp(mapped.data, mapped.size);
      if (packet) {
        source->packets.fetch_add(1);
        source->bytes.fetch_add(mapped.size);
        source->last_packet_ms.store(monotonic_ms());
        if (packet->marker) {source->frames.fetch_add(1);}
        std::vector<NativeMediaRelay::PacketCallback> callbacks;
        {
          std::lock_guard<std::mutex> lock(source->mutex);
          const auto missing = source->sequence_gaps.observe(media::read_u32(mapped.data + 8), packet->sequence);
          if (missing) {
            source->upstream_gap_events.fetch_add(1);
            source->upstream_missing_packets.fetch_add(missing);
            if (source->config.value("recover_on_packet_gap", true)) {
              source->needs_keyframe.store(true);
            }
          }
          for (const auto & entry : source->consumers) {callbacks.push_back(entry.second);}
        }
        for (const auto & callback : callbacks) {
          try {callback(mapped.data, mapped.size, static_cast<Context *>(data)->generation);}
          catch (...) {source->failures.fetch_add(1);}
        }
      }
      gst_buffer_unmap(buffer, &mapped);
    }
    gst_sample_unref(value);
    return GST_FLOW_OK;
  }
  static void incoming(GstElement *, GstPad * pad, gpointer data)
  {
    auto source = current(data);
    if (!source || GST_PAD_DIRECTION(pad) != GST_PAD_SRC) {return;}
    const auto epoch = static_cast<Context *>(data)->generation;
    std::lock_guard<std::mutex> life(source->lifecycle_mutex);
    if (source->generation.load() != epoch || !source->pipeline || source->sink) {return;}
    GstCaps * caps = gst_pad_get_current_caps(pad);
    if (!caps) {caps = gst_pad_query_caps(pad, nullptr);}
    const bool rtp = caps && gst_caps_get_size(caps) > 0 &&
      gst_structure_has_name(gst_caps_get_structure(caps, 0), "application/x-rtp");
    if (caps) {gst_caps_unref(caps);}
    if (!rtp) {return;}
    GstElement * element = gst_element_factory_make("appsink", nullptr);
    if (!element) {source->failures.fetch_add(1); return;}
    g_object_set(element, "emit-signals", TRUE, "sync", FALSE,
      "max-buffers", 32U, "drop", FALSE, "enable-last-sample", FALSE, nullptr);
    source->sink = element;
    source->connect(G_OBJECT(element), "new-sample", G_CALLBACK(sample), epoch);
    gst_bin_add(GST_BIN(source->pipeline), element);
    GstPad * target = gst_element_get_static_pad(element, "sink");
    if (gst_pad_link(pad, target) != GST_PAD_LINK_OK) {source->failures.fetch_add(1);}
    gst_object_unref(target);
    gst_element_sync_state_with_parent(element);
  }
  static void candidate(GstElement *, guint line, gchar * value, gpointer data)
  {
    auto source = current(data);
    if (!source || !value) {return;}
    std::lock_guard<std::mutex> lock(source->mutex);
    if (source->generation.load() != static_cast<Context *>(data)->generation) {return;}
    if (std::string(value).find(" typ relay") != std::string::npos) {++source->local_relay_candidates;}
    else {++source->local_other_candidates;}
    source->candidates.push_back({{"type", "candidate"}, {"candidate", value},
      {"sdpMid", "0"}, {"sdpMLineIndex", line}});
  }
  static void state(GObject * object, GParamSpec *, gpointer data)
  {
    auto source = current(data);
    if (!source) {return;}
    GstWebRTCPeerConnectionState state;
    g_object_get(object, "connection-state", &state, nullptr);
    source->connected.store(state == GST_WEBRTC_PEER_CONNECTION_STATE_CONNECTED);
    if (state == GST_WEBRTC_PEER_CONNECTION_STATE_FAILED) {
      source->failures.fetch_add(1);
    }
  }
  static void jitterbuffer(GstElement *, GstElement * buffer, guint, guint, gpointer data)
  {
    if (!current(data)) {return;}
    // Variable camera cadence must not turn an invented next-packet deadline
    // into loss of the first RTP packet of every lower-FPS frame.
    g_object_set(buffer, "rtx-next-seqnum", FALSE, nullptr);
  }
  static void offered(GstPromise * promise, gpointer data)
  {
    auto source = current(data);
    const GstStructure * reply = gst_promise_wait(promise) == GST_PROMISE_RESULT_REPLIED ?
      gst_promise_get_reply(promise) : nullptr;
    GstWebRTCSessionDescription * description = nullptr;
    if (source && reply && gst_structure_get(reply, "offer",
        GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &description, nullptr) && description)
    {
      gchar * text = gst_sdp_message_as_text(description->sdp);
      GstElement * peer = nullptr;
      {
        std::lock_guard<std::mutex> life(source->lifecycle_mutex);
        if (source->generation.load() == static_cast<Context *>(data)->generation && source->peer) {
          peer = GST_ELEMENT(gst_object_ref(source->peer));
          std::lock_guard<std::mutex> lock(source->mutex);
          source->offer = text ? text : "";
        }
      }
      g_free(text);
      if (peer) {
        g_signal_emit_by_name(peer, "set-local-description", description, nullptr);
        gst_object_unref(peer);
      }
      gst_webrtc_session_description_free(description);
    } else if (source) {source->failures.fetch_add(1);}
    gst_promise_unref(promise);
  }
  void start()
  {
    ++generation;
    started = monotonic_ms();
    last_packet_ms.store(0);
    gchar * nonce = g_uuid_string_random();
    session = "native_relay_" + config.at("id").get<std::string>() + "_" + nonce;
    g_free(nonce);
    {
      std::lock_guard<std::mutex> lock(mutex);
      offer.clear(); answer.clear(); candidates.clear();
      local_relay_candidates = local_other_candidates = 0;
      sequence_gaps = {};
    }
    connected.store(false);
    profile_supported = false;
    last_profile_bitrate = last_profile_fps = last_profile_priority = 0;
    profile_dirty.store(true);
    needs_keyframe.store(true);
    pipeline = gst_pipeline_new(nullptr);
    peer = gst_element_factory_make("webrtcbin", nullptr);
    if (!pipeline || !peer) {
      if (peer) {gst_object_unref(peer); peer = nullptr;}
      if (pipeline) {gst_object_unref(pipeline); pipeline = nullptr;}
      throw std::runtime_error("Native relay requires webrtcbin");
    }
    g_object_set(peer, "bundle-policy", GST_WEBRTC_BUNDLE_POLICY_MAX_BUNDLE,
      "latency", config.value("jitter_buffer_ms", 30U), nullptr);
    gst_bin_add(GST_BIN(pipeline), peer);
    g_object_set(peer, "ice-transport-policy",
      config.value("ice_transport_policy", "all") == "relay" ?
      GST_WEBRTC_ICE_TRANSPORT_POLICY_RELAY : GST_WEBRTC_ICE_TRANSPORT_POLICY_ALL, nullptr);
    for (const auto & value : config.value("ice_servers", nlohmann::json::array())) {
      const auto url = value.get<std::string>();
      if (url.rfind("stun:", 0) == 0) {
        const auto normalized = url.rfind("stun://", 0) == 0 ? url : "stun://" + url.substr(5);
        g_object_set(peer, "stun-server", normalized.c_str(), nullptr);
      } else {
        gboolean accepted = FALSE;
        g_signal_emit_by_name(peer, "add-turn-server", url.c_str(), &accepted);
        if (!accepted) {throw std::invalid_argument("Native relay rejected a configured TURN server");}
      }
    }
    if (auto * rtpbin = gst_bin_get_by_name(GST_BIN(peer), "rtpbin")) {
      connect(G_OBJECT(rtpbin), "new-jitterbuffer", G_CALLBACK(jitterbuffer));
      gst_object_unref(rtpbin);
    }
    connect(G_OBJECT(peer), "pad-added", G_CALLBACK(incoming));
    connect(G_OBJECT(peer), "on-ice-candidate", G_CALLBACK(candidate));
    connect(G_OBJECT(peer), "notify::connection-state", G_CALLBACK(state));
    GstCaps * caps = gst_caps_from_string(
      "application/x-rtp,media=video,encoding-name=H264,clock-rate=90000,payload=96,"
      "rtcp-fb-nack=(boolean)true,rtcp-fb-nack-pli=(boolean)true,"
      "rtcp-fb-transport-cc=(boolean)true,"
      "extmap-1=(string)http://www.ietf.org/id/draft-holmer-rmcat-transport-wide-cc-extensions-01");
    GstWebRTCRTPTransceiver * transceiver = nullptr;
    g_signal_emit_by_name(peer, "add-transceiver", GST_WEBRTC_RTP_TRANSCEIVER_DIRECTION_RECVONLY,
      caps, &transceiver);
    gst_caps_unref(caps);
    if (transceiver) {
      if (g_object_class_find_property(G_OBJECT_GET_CLASS(transceiver), "do-nack")) {
        g_object_set(transceiver, "do-nack", TRUE, nullptr);
      }
      gst_object_unref(transceiver);
    }
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    auto * context = new Context{weak_from_this(), generation.load()};
    GstPromise * promise = gst_promise_new_with_change_func(offered, context,
      [](gpointer p) {delete static_cast<Context *>(p);});
    g_signal_emit_by_name(peer, "create-offer", nullptr, promise);
  }
  void stop()
  {
    GstElement * old_pipeline;
    {
      std::lock_guard<std::mutex> life(lifecycle_mutex);
      ++generation;
      old_pipeline = pipeline;
      pipeline = peer = sink = nullptr;
    }
    if (!session.empty()) {send({{"type", "stop"}});}
    std::vector<std::pair<GObject *, gulong>> detached;
    {
      std::lock_guard<std::mutex> lock(handlers_mutex);
      detached.swap(handlers);
    }
    for (const auto & handler : detached) {g_signal_handler_disconnect(handler.first, handler.second);}
    if (old_pipeline) {
      gst_element_set_state(old_pipeline, GST_STATE_NULL);
      gst_element_get_state(old_pipeline, nullptr, nullptr, 2 * GST_SECOND);
      gst_object_unref(old_pipeline);
    }
    std::lock_guard<std::mutex> lock(mutex);
    session.clear(); offer.clear(); answer.clear(); candidates.clear();
    connected.store(false);
  }
  void keyframe()
  {
    const auto now = monotonic_ms();
    if (session.empty() || !connected.load() || now - last_keyframe_ms.load() < 500) {return;}
    last_keyframe_ms.store(now);
    needs_keyframe.store(false);
    keyframe_requests.fetch_add(1);
    send({{"type", "keyframe"}});
    GstElement * target = nullptr;
    {
      std::lock_guard<std::mutex> life(lifecycle_mutex);
      if (sink) {target = GST_ELEMENT(gst_object_ref(sink));}
    }
    if (target) {
      gst_element_send_event(target, gst_video_event_new_upstream_force_key_unit(
          GST_CLOCK_TIME_NONE, TRUE, 0));
      gst_object_unref(target);
    }
  }
  void receive(const nlohmann::json & message)
  {
    if (!peer || message.value("session_id", "") != session) {return;}
    const auto type = message.value("type", "");
    if (type == "candidate") {
      const auto candidate = message.value("candidate", "");
      g_signal_emit_by_name(peer, "add-ice-candidate", message.value("sdpMLineIndex", 0),
        candidate.c_str());
    } else if (type == "answer") {
      const auto text = message.value("sdp", "");
      if (text.empty() || answer == text) {return;}
      GstSDPMessage * sdp = nullptr;
      gst_sdp_message_new(&sdp);
      if (gst_sdp_message_parse_buffer(reinterpret_cast<const guint8 *>(text.data()),
          text.size(), sdp) != GST_SDP_OK)
      {
        gst_sdp_message_free(sdp); failures.fetch_add(1); return;
      }
      // Expand inherited transport attributes; retain the actual fingerprint.
      for (guint i = 0; i < gst_sdp_message_medias_len(sdp); ++i) {
        auto * media = const_cast<GstSDPMedia *>(gst_sdp_message_get_media(sdp, i));
        for (const char * name : {"setup", "ice-ufrag", "ice-pwd"}) {
          const char * inherited = gst_sdp_message_get_attribute_val(sdp, name);
          if (inherited && !gst_sdp_media_get_attribute_val(media, name)) {
            gst_sdp_media_add_attribute(media, name, inherited);
          }
        }
      }
      auto * description = gst_webrtc_session_description_new(GST_WEBRTC_SDP_TYPE_ANSWER, sdp);
      g_signal_emit_by_name(peer, "set-remote-description", description, nullptr);
      gst_webrtc_session_description_free(description);
      answer = text;
    } else if (type == "ready") {
      profile_supported = message.value("profile_updates", false);
      profile_dirty.store(true);
    } else if (type == "error") {failures.fetch_add(1);}
  }
  void tick()
  {
    const int64_t now = monotonic_ms();
    bool demanded;
    {std::lock_guard<std::mutex> lock(mutex); demanded = !consumers.empty();}
    if (!demanded) {
      if (!idle_since) {idle_since = now;}
      if (pipeline && now - idle_since > 3000) {stop();}
      return;
    }
    idle_since = 0;
    if (!pipeline) {if (now >= retry_at) {start();} return;}
    GstBus * bus = gst_element_get_bus(pipeline);
    GstMessage * error = gst_bus_pop_filtered(bus, GST_MESSAGE_ERROR);
    gst_object_unref(bus);
    if (error) {gst_message_unref(error); failures.fetch_add(1);}
    GstWebRTCPeerConnectionState state;
    g_object_get(peer, "connection-state", &state, nullptr);
    const auto age = last_packet_ms.load();
    if (error || state == GST_WEBRTC_PEER_CONNECTION_STATE_FAILED ||
      (!connected.load() && now - started > 15000) ||
      (connected.load() && now - (age ? age : started) > 10000))
    {
      stop();
      retry_at = now + std::min<int64_t>(10000, 500LL << std::min(retries++, 4U));
      return;
    }
    if (connected.load()) {
      retries = 0;
      if (needs_keyframe.load()) {keyframe();}
      if (profile_supported && profile_dirty.exchange(false)) {
        int bitrate = 100, fps = 1, priority = 1;
        {
          std::lock_guard<std::mutex> lock(mutex);
          for (const auto & profile : profiles) {
            const auto capacity = capacities.find(profile.first);
            bitrate = std::max(bitrate, capacity == capacities.end() ? profile.second.first :
              std::min(profile.second.first, capacity->second));
            fps = std::max(fps, profile.second.second);
          }
          for (const auto & entry : priorities) {priority = std::max(priority, entry.second);}
        }
        bitrate = std::max(100, bitrate / 100 * 100);
        if (bitrate != last_profile_bitrate || fps != last_profile_fps || priority != last_profile_priority) {
          send({{"type", "profile"}, {"bitrate_kbps", bitrate}, {"framerate", fps}, {"priority", priority}});
          last_profile_bitrate = bitrate; last_profile_fps = fps; last_profile_priority = priority;
        }
      }
    }
    if (connected.load() && now - (age ? age : started) > 2000) {keyframe();}
    std::string local_offer;
    std::vector<nlohmann::json> pending;
    {
      std::lock_guard<std::mutex> lock(mutex);
      local_offer = offer;
      pending.swap(candidates);
    }
    if (!local_offer.empty() && answer.empty() && now - last_offer > 1000) {
      last_offer = now;
      send({{"type", "offer"}, {"sdp", local_offer},
        {"stream_topic", config.at("input_topic")}, {"image_type", config.value("image_type", "raw")},
        {"bitrate_kbps", config.value("bitrate_kbps", 1500)},
        {"framerate", config.value("framerate", 30)},
        {"stun_servers", config.value("ice_servers", nlohmann::json::array())},
        {"wait_for_complete_gathering", true}});
    }
    if (!local_offer.empty() && last_offer >= started) {
      for (auto & value : pending) {send(std::move(value));}
    } else {
      std::lock_guard<std::mutex> lock(mutex);
      candidates.insert(candidates.end(), pending.begin(), pending.end());
    }
  }
};
}  // namespace

struct NativeMediaRelay::Impl
{
  std::map<std::string, std::shared_ptr<RelaySource>> sources;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr signals;
};

nlohmann::json NativeMediaRelay::validate_config(const nlohmann::json & input)
{
  auto config = input;
  if (!config.is_object() || !config.contains("streams") || !config["streams"].is_array() ||
    config["streams"].empty() || config["streams"].size() > 32)
  {throw std::invalid_argument("Native media relay requires 1..32 streams");}
  auto topic = [](const nlohmann::json & value, const char * name) {
      if (!value.is_string() || value.get_ref<const std::string &>().size() < 2 ||
        value.get_ref<const std::string &>().front() != '/' ||
        value.get_ref<const std::string &>().find_first_of(" \t\r\n") != std::string::npos)
      {throw std::invalid_argument(std::string(name) + " must be an absolute ROS topic");}
    };
  for (const char * name : {"client_signal_topic", "server_signal_topic"}) {
    if (config.contains(name)) {topic(config[name], name);}
  }
  std::set<std::string> inputs, outputs, ids;
  for (auto & stream : config["streams"]) {
    if (!stream.is_object() || !stream.contains("input_topic") || !stream.contains("output_topic"))
    {throw std::invalid_argument("Relay stream requires input_topic and output_topic");}
    topic(stream["input_topic"], "input_topic");
    topic(stream["output_topic"], "output_topic");
    for (const char * key : {"ice_servers", "ice_transport_policy", "recover_on_packet_gap"}) {
      if (!stream.contains(key) && config.contains(key)) {stream[key] = config[key];}
    }
    if (stream.contains("recover_on_packet_gap") && !stream["recover_on_packet_gap"].is_boolean())
    {throw std::invalid_argument("Relay recover_on_packet_gap must be boolean");}
    const auto policy = stream.value("ice_transport_policy", nlohmann::json("all"));
    if (!policy.is_string() || (policy != "all" && policy != "relay"))
    {throw std::invalid_argument("Relay ice_transport_policy must be all or relay");}
    bool has_turn = false;
    if (stream.contains("ice_servers")) {
      if (!stream["ice_servers"].is_array() || stream["ice_servers"].size() > 16)
      {throw std::invalid_argument("Relay ice_servers must be an array of at most 16 URLs");}
      for (const auto & server : stream["ice_servers"]) {
        if (!server.is_string()) {throw std::invalid_argument("Relay ICE server must be a URL string");}
        const auto & url = server.get_ref<const std::string &>();
        const bool turn = url.rfind("turn://", 0) == 0 || url.rfind("turns://", 0) == 0;
        if (url.size() > 4096 || url.find_first_of(" \t\r\n") != std::string::npos ||
          (!turn && url.rfind("stun:", 0) != 0))
        {throw std::invalid_argument("Relay ICE server requires a STUN or TURN URL");}
        has_turn = has_turn || turn;
      }
    }
    if (policy == "relay" && !has_turn)
    {throw std::invalid_argument("Relay-only ICE requires at least one TURN server");}
    inputs.insert(stream["input_topic"].get<std::string>());
    if (!stream.contains("id")) {stream["id"] = "camera_" + std::to_string(ids.size());}
    if (!stream["id"].is_string() || !std::regex_match(stream["id"].get_ref<const std::string &>(),
        std::regex("[A-Za-z][A-Za-z0-9_]*")) ||
      !ids.insert(stream["id"].get<std::string>()).second ||
      !outputs.insert(stream["output_topic"].get<std::string>()).second)
    {throw std::invalid_argument("Relay stream IDs and output topics must be unique");}
    for (const auto & limit : {std::make_tuple("bitrate_kbps", 100, 100000),
        std::make_tuple("framerate", 1, 60), std::make_tuple("jitter_buffer_ms", 0, 1000)})
    {
      const char * name = std::get<0>(limit);
      if (stream.contains(name) && (!stream[name].is_number_integer() ||
        stream[name] < std::get<1>(limit) || stream[name] > std::get<2>(limit)))
      {throw std::invalid_argument(std::string("Invalid relay ") + name);}
    }
    if (stream.contains("image_type") && stream["image_type"] != "raw" &&
      stream["image_type"] != "compressed")
    {throw std::invalid_argument("Relay image_type must be raw or compressed");}
  }
  for (const auto & output : outputs) {
    if (inputs.count(output)) {throw std::invalid_argument("Relay output cannot overlap a source input");}
  }
  return config;
}

NativeMediaRelay::NativeMediaRelay(rclcpp::Node * node, const std::string & path)
: impl_(std::make_unique<Impl>())
{
  gst_init(nullptr, nullptr);
  std::ifstream file(path);
  nlohmann::json config;
  if (!file || !(file >> config))
  {throw std::invalid_argument("Native media relay requires a streams configuration");}
  config = validate_config(config);
  for (const char * name : {"webrtcbin", "nicesrc", "nicesink", "appsink"}) {
    auto * factory = gst_element_factory_find(name);
    if (!factory) {throw std::runtime_error(std::string("Native relay requires GStreamer ") + name);}
    gst_object_unref(factory);
  }
  auto publisher = node->create_publisher<std_msgs::msg::String>(
    config.value("client_signal_topic", "/horus/remote/webrtc/client_signal"), 10);
  for (const auto & stream : config["streams"]) {
    auto source = std::make_shared<RelaySource>();
    source->config = stream;
    source->signal = publisher;
    const auto output = stream.at("output_topic").get<std::string>();
    if (output.empty() || impl_->sources.count(output) ||
      stream.at("input_topic").get<std::string>().empty())
    {throw std::invalid_argument("Native relay stream aliases must be unique");}
    impl_->sources.emplace(output, source);
  }
  impl_->signals = node->create_subscription<std_msgs::msg::String>(
    config.value("server_signal_topic", "/horus/remote/webrtc/server_signal"),
    rclcpp::QoS(100).reliable().transient_local(),
    [this](const std_msgs::msg::String::SharedPtr message) {
      try {
        auto signal = nlohmann::json::parse(message->data);
        for (auto & source : impl_->sources) {source.second->receive(signal);}
      } catch (const std::exception &) { /* Invalid or obsolete signaling is ignored. */ }
    });
}

NativeMediaRelay::~NativeMediaRelay()
{
  impl_->signals.reset();
  for (auto & source : impl_->sources) {source.second->stop();}
}

bool NativeMediaRelay::contains(const std::string & topic) const
{
  return impl_->sources.count(topic) != 0;
}

bool NativeMediaRelay::subscribe(
  const std::string & topic, const std::string & consumer, PacketCallback callback)
{
  auto found = impl_->sources.find(topic);
  if (found == impl_->sources.end() || !callback) {return false;}
  auto source = found->second;
  std::lock_guard<std::mutex> lock(source->mutex);
  source->consumers[consumer] = std::move(callback);
  source->needs_keyframe.store(true);
  return true;
}

void NativeMediaRelay::unsubscribe(const std::string & topic, const std::string & consumer)
{
  auto found = impl_->sources.find(topic);
  if (found != impl_->sources.end()) {
    std::lock_guard<std::mutex> lock(found->second->mutex);
    found->second->consumers.erase(consumer);
    found->second->profiles.erase(consumer);
    found->second->priorities.erase(consumer);
    found->second->capacities.erase(consumer);
    found->second->profile_dirty.store(true);
  }
}

void NativeMediaRelay::request_keyframe(const std::string & topic)
{
  auto found = impl_->sources.find(topic);
  if (found != impl_->sources.end()) {found->second->needs_keyframe.store(true);}
}

void NativeMediaRelay::update_profile(
  const std::string & topic, const std::string & consumer, int bitrate_kbps, int framerate, int priority)
{
  const auto found = impl_->sources.find(topic);
  if (found == impl_->sources.end()) {return;}
  std::lock_guard<std::mutex> lock(found->second->mutex);
  if (!found->second->consumers.count(consumer)) {return;}
  found->second->profiles[consumer] = {std::clamp(bitrate_kbps, 100, 100000),
    std::clamp(framerate, 1, 60)};
  found->second->priorities[consumer] = std::clamp(priority, 1, 16);
  found->second->profile_dirty.store(true);
}

void NativeMediaRelay::update_capacity(
  const std::string & topic, const std::string & consumer, int bitrate_kbps)
{
  const auto found = impl_->sources.find(topic);
  if (found == impl_->sources.end()) {return;}
  std::lock_guard<std::mutex> lock(found->second->mutex);
  if (!found->second->consumers.count(consumer)) {return;}
  found->second->capacities[consumer] = std::clamp(bitrate_kbps, 100, 100000);
  found->second->profile_dirty.store(true);
}

void NativeMediaRelay::tick()
{
  for (auto & source : impl_->sources) {
    try {source.second->tick();}
    catch (const std::exception &) {
      source.second->failures.fetch_add(1);
      source.second->stop();
      source.second->retry_at = monotonic_ms() + 10000;
    }
  }
}

nlohmann::json NativeMediaRelay::statistics() const
{
  nlohmann::json result = nlohmann::json::object();
  for (const auto & entry : impl_->sources) {
    const auto & source = entry.second;
    std::lock_guard<std::mutex> lock(source->mutex);
    result[entry.first] = {{"consumers", source->consumers.size()},
      {"packets", source->packets.load()}, {"frames", source->frames.load()},
      {"bytes", source->bytes.load()}, {"connected", source->connected.load()},
      {"last_packet_age_ms", source->last_packet_ms.load() ?
        monotonic_ms() - source->last_packet_ms.load() : -1},
      {"failures", source->failures.load()}, {"transport", "native_rtp_no_dds"},
      {"ice_transport_policy", source->config.value("ice_transport_policy", "all")}};
    result[entry.first]["local_relay_candidates"] = source->local_relay_candidates;
    result[entry.first]["local_other_candidates"] = source->local_other_candidates;
    result[entry.first]["upstream_sequence_gap_events"] = source->upstream_gap_events.load();
    result[entry.first]["upstream_missing_sequence_numbers"] = source->upstream_missing_packets.load();
    result[entry.first]["keyframe_requests"] = source->keyframe_requests.load();
  }
  return result;
}
}  // namespace horus_unity_bridge
