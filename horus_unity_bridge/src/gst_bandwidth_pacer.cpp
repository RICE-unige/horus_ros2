// Copyright 2026 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0
#include "horus_unity_bridge/gst_bandwidth_pacer.hpp"
#include "horus_unity_bridge/rtp_relay.hpp"
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <gst/rtp/gstrtpbuffer.h>
#include <atomic>
#include <chrono>
#include <mutex>
#include <map>
#include <stdexcept>

namespace horus_unity_bridge
{
namespace
{
int64_t steady_us()
{
  return std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
bool begins_idr(const media::RtpPacket & packet)
{
  if (!packet.size) {return false;}
  const auto type = packet.payload[0] & 31;
  if (type == 5) {return true;}
  if (type == 28 && packet.size >= 2) {return (packet.payload[1] & 0x9f) == 0x85;}
  if (type == 24) {
    for (size_t p = 1; p + 2 < packet.size; ) {
      const size_t size = (size_t(packet.payload[p]) << 8) | packet.payload[p + 1];
      p += 2;
      if (!size || size > packet.size - p) {break;}
      if ((packet.payload[p] & 31) == 5) {return true;}
      p += size;
    }
  }
  return false;
}

std::optional<uint16_t> transport_id(GstBuffer * buffer, int extension)
{
  GstRTPBuffer rtp = GST_RTP_BUFFER_INIT;
  if (!gst_rtp_buffer_map(buffer, GST_MAP_READ, &rtp)) {return std::nullopt;}
  gpointer data = nullptr;
  guint length = 0;
  std::optional<uint16_t> result;
  if (gst_rtp_buffer_get_extension_onebyte_header(&rtp, extension, 0, &data, &length) && length == 2) {
    const auto * bytes = static_cast<const uint8_t *>(data);
    result = uint16_t((uint16_t(bytes[0]) << 8) | bytes[1]);
  }
  gst_rtp_buffer_unmap(&rtp);
  return result;
}
}
struct GstBandwidthPacer::Impl
{
  GstElement * pipeline = nullptr, * rtp = nullptr, * rtcp = nullptr, * output = nullptr;
  gulong sample_handler = 0, estimate_handler = 0;
  GstElement * estimator = nullptr;
  std::function<void(int)> estimate;
  std::function<void()> keyframe;
  rtc::message_callback send;
  mutable std::mutex mutex;
  std::mutex callback_mutex;
  std::atomic<bool> closed{false}, waiting_idr{false};
  std::atomic<uint64_t> emitted{0}, frames{0}, bytes{0}, dropped{0}, feedback{0}, estimates{0};
  std::atomic<unsigned> bitrate{0};
  uint16_t transport_sequence = 0;
  int extension;
  uint32_t ssrc;
  uint8_t payload_type;
  int64_t last_keyframe = 0;
  struct Pending {int64_t time; size_t bytes;};
  std::map<uint16_t, Pending> pending;
  size_t pending_bytes = 0;

  void request_keyframe()
  {
    std::function<void()> callback;
    {
      std::lock_guard<std::mutex> lock(mutex);
      const auto now = steady_us();
      if (now - last_keyframe < 500000 || closed.load()) {return;}
      last_keyframe = now; callback = keyframe;
    }
    if (callback) {callback();}
  }
  static GstFlowReturn sample(GstElement * sink, gpointer data)
  {
    auto * self = static_cast<Impl *>(data);
    std::lock_guard<std::mutex> life(self->callback_mutex);
    GstSample * sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
    if (!sample) {return GST_FLOW_OK;}
    GstBuffer * buffer = gst_sample_get_buffer(sample);
    GstMapInfo map;
    bool recover = false;
    if (!self->closed.load() && buffer && gst_buffer_map(buffer, &map, GST_MAP_READ)) {
      const auto packet = media::parse_rtp(map.data, map.size);
      const auto identity = transport_id(buffer, self->extension);
      rtc::message_callback send;
      if (packet) {
        std::lock_guard<std::mutex> lock(self->mutex);
        const auto pending = identity ? self->pending.find(*identity) : self->pending.end();
        const bool stale = pending == self->pending.end() || steady_us() - pending->second.time > 250000;
        if (pending != self->pending.end()) {
          self->pending_bytes -= pending->second.bytes; self->pending.erase(pending);
        }
        if (stale) {self->waiting_idr.store(true); recover = true;}
        if (!stale && self->waiting_idr.load() && begins_idr(*packet)) {self->waiting_idr.store(false);}
        if (!stale && !self->waiting_idr.load()) {send = self->send;}
      }
      if (send) {
        try {
          auto message = rtc::make_message(reinterpret_cast<const std::byte *>(map.data),
            reinterpret_cast<const std::byte *>(map.data + map.size), rtc::Message::Binary, self->ssrc);
          send(std::move(message)); ++self->emitted;
          self->bytes.fetch_add(map.size);
          if (packet->marker) {++self->frames;}
        } catch (const std::exception &) {++self->dropped;}
      } else {++self->dropped;}
      gst_buffer_unmap(buffer, &map);
    }
    gst_sample_unref(sample);
    if (recover) {self->request_keyframe();}
    return GST_FLOW_OK;
  }
  static void changed(GObject * object, GParamSpec *, gpointer data)
  {
    auto * self = static_cast<Impl *>(data);
    std::lock_guard<std::mutex> life(self->callback_mutex);
    if (self->closed.load()) {return;}
    guint bitrate;
    g_object_get(object, "estimated-bitrate", &bitrate, nullptr);
    self->bitrate.store(bitrate); ++self->estimates;
    std::function<void(int)> callback;
    {std::lock_guard<std::mutex> lock(self->mutex); callback = self->estimate;}
    if (callback) {callback(std::max(100, int(bitrate / 1000)));}
  }
  void shutdown()
  {
    if (closed.exchange(true)) {return;}
    if (output && sample_handler) {g_signal_handler_disconnect(output, sample_handler);}
    if (estimator && estimate_handler) {g_signal_handler_disconnect(estimator, estimate_handler);}
    if (pipeline) {
      gst_element_set_state(pipeline, GST_STATE_NULL);
      gst_element_get_state(pipeline, nullptr, nullptr, 2 * GST_SECOND);
    }
    std::lock_guard<std::mutex> life(callback_mutex);
    std::lock_guard<std::mutex> lock(mutex);
    send = {}; estimate = {}; keyframe = {}; pending.clear(); pending_bytes = 0;
  }
  ~Impl()
  {
    shutdown();
    for (auto * element : {rtp, rtcp, output, estimator, pipeline}) {
      if (element) {gst_object_unref(element);}
    }
  }
};

bool GstBandwidthPacer::available()
{
  gst_init(nullptr, nullptr);
  auto * factory = gst_element_factory_find("rtpgccbwe");
  if (!factory) {return false;}
  gst_object_unref(factory); return true;
}
GstBandwidthPacer::GstBandwidthPacer(uint32_t ssrc, uint8_t pt, int extension, int kbps,
  std::function<void(int)> estimate, std::function<void()> keyframe)
: impl_(std::make_unique<Impl>())
{
  if (extension < 1 || extension > 14 || !available()) {
    throw std::invalid_argument("GCC pacing requires rtpgccbwe and a negotiated TWCC extension 1..14");
  }
  impl_->ssrc = ssrc; impl_->payload_type = pt; impl_->extension = extension;
  impl_->estimate = std::move(estimate); impl_->keyframe = std::move(keyframe);
  impl_->bitrate.store(kbps * 1000);
  const std::string description =
    "rtpbin name=session rtp-profile=avpf "
    "appsrc name=rtp_in is-live=true do-timestamp=true format=time block=false max-bytes=2097152 "
    "! rtpgccbwe name=bwe estimated-bitrate=" + std::to_string(kbps * 1000) +
    " max-bitrate=100000000 min-bitrate=100000 ! session.send_rtp_sink_0 "
    "session.send_rtp_src_0 ! appsink name=out emit-signals=true sync=false max-buffers=32 drop=false "
    "appsrc name=rtcp_in is-live=true do-timestamp=true format=time block=false max-bytes=65536 "
    "caps=application/x-rtcp ! session.recv_rtcp_sink_0 "
    "session.send_rtcp_src_0 ! fakesink sync=false async=false";
  GError * error = nullptr;
  impl_->pipeline = gst_parse_launch(description.c_str(), &error);
  if (error) {
    const std::string message = error->message; g_error_free(error);
    throw std::runtime_error("GCC pipeline: " + message);
  }
  for (const auto & entry : {std::make_pair("rtp_in", &impl_->rtp), {"rtcp_in", &impl_->rtcp},
      {"out", &impl_->output}, {"bwe", &impl_->estimator}})
  {*entry.second = gst_bin_get_by_name(GST_BIN(impl_->pipeline), entry.first);}
  if (!impl_->rtp || !impl_->rtcp || !impl_->output || !impl_->estimator) {
    throw std::runtime_error("GCC pipeline elements unavailable");
  }
  auto * caps = gst_caps_new_simple("application/x-rtp", "media", G_TYPE_STRING, "video",
    "encoding-name", G_TYPE_STRING, "H264", "clock-rate", G_TYPE_INT, 90000,
    "payload", G_TYPE_INT, int(pt), "ssrc", G_TYPE_UINT, ssrc,
    "rtcp-fb-transport-cc", G_TYPE_BOOLEAN, TRUE, nullptr);
  gst_structure_set(gst_caps_get_structure(caps, 0), ("extmap-" + std::to_string(extension)).c_str(),
    G_TYPE_STRING, "http://www.ietf.org/id/draft-holmer-rmcat-transport-wide-cc-extensions-01", nullptr);
  gst_app_src_set_caps(GST_APP_SRC(impl_->rtp), caps); gst_caps_unref(caps);
  impl_->sample_handler = g_signal_connect(impl_->output, "new-sample", G_CALLBACK(Impl::sample), impl_.get());
  impl_->estimate_handler = g_signal_connect(impl_->estimator, "notify::estimated-bitrate", G_CALLBACK(Impl::changed), impl_.get());
  if (gst_element_set_state(impl_->pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    throw std::runtime_error("GCC pipeline cannot start");
  }
}
GstBandwidthPacer::~GstBandwidthPacer() = default;
void GstBandwidthPacer::shutdown() {impl_->shutdown();}
void GstBandwidthPacer::incoming(rtc::message_vector & messages, const rtc::message_callback &)
{
  if (impl_->closed.load()) {return;}
  for (const auto & message : messages) {
    if (!message || message->type != rtc::Message::Control || message->size() > 65536) {continue;}
    if (gst_app_src_get_current_level_bytes(GST_APP_SRC(impl_->rtcp)) + message->size() > 65536) {continue;}
    auto * buffer = gst_buffer_new_allocate(nullptr, message->size(), nullptr);
    gst_buffer_fill(buffer, 0, message->data(), message->size());
    gst_app_src_push_buffer(GST_APP_SRC(impl_->rtcp), buffer); ++impl_->feedback;
  }
}
void GstBandwidthPacer::outgoing(rtc::message_vector & messages, const rtc::message_callback & send)
{
  if (impl_->closed.load()) {messages.clear(); return;}
  rtc::message_vector control;
  bool recover = false;
  for (const auto & message : messages) {
    if (!message) {continue;}
    if (message->type == rtc::Message::Control) {control.push_back(message); continue;}
    const auto * bytes = reinterpret_cast<const uint8_t *>(message->data());
    auto source = media::parse_rtp(bytes, message->size());
    if (!source) {continue;}
    auto packet = media::translate_rtp(*source, impl_->payload_type, impl_->ssrc, 0, 0);
    {
      std::lock_guard<std::mutex> lock(impl_->mutex);
      const auto now = steady_us();
      for (auto it = impl_->pending.begin(); it != impl_->pending.end(); ) {
        if (now - it->second.time <= 250000) {++it; continue;}
        impl_->pending_bytes -= it->second.bytes;
        it = impl_->pending.erase(it);
        impl_->waiting_idr.store(true); ++impl_->dropped; recover = true;
      }
      if (impl_->pending_bytes + packet.size() + 8 > 2 * 1024 * 1024) {
        ++impl_->dropped; impl_->waiting_idr.store(true); recover = true; continue;
      }
      impl_->send = send;
      packet[0] |= 0x10;
      const uint16_t transport = impl_->transport_sequence++;
      packet.insert(packet.begin() + 12, {0xbe, 0xde, 0, 1, uint8_t((impl_->extension << 4) | 1),
        uint8_t(transport >> 8), uint8_t(transport), 0});
      // A repair reuses the media RTP sequence. TWCC is unique per hop send,
      // so an original and its queued repair must own different entries.
      const auto prior = impl_->pending.find(transport);
      if (prior != impl_->pending.end()) {impl_->pending_bytes -= prior->second.bytes;}
      impl_->pending[transport] = {steady_us(), packet.size()};
      impl_->pending_bytes += packet.size();
    }
    auto * buffer = gst_buffer_new_allocate(nullptr, packet.size(), nullptr);
    gst_buffer_fill(buffer, 0, packet.data(), packet.size());
    if (gst_app_src_push_buffer(GST_APP_SRC(impl_->rtp), buffer) != GST_FLOW_OK) {
      ++impl_->dropped; impl_->waiting_idr.store(true); recover = true;
    }
  }
  messages = std::move(control);
  if (recover) {impl_->request_keyframe();}
}
nlohmann::json GstBandwidthPacer::statistics() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return {{"engine", "gstreamer_gcc_twcc"}, {"estimated_bitrate_bps", impl_->bitrate.load()},
    {"estimate_updates", impl_->estimates.load()}, {"rtcp_packets", impl_->feedback.load()},
    {"paced_packets", impl_->emitted.load()}, {"dropped_packets", impl_->dropped.load()},
    {"paced_frames", impl_->frames.load()},
    {"pending_bytes", impl_->pending_bytes}, {"waiting_idr", impl_->waiting_idr.load()}};
}
uint64_t GstBandwidthPacer::frames_sent() const {return impl_->frames.load();}
uint64_t GstBandwidthPacer::bytes_sent() const {return impl_->bytes.load();}
}  // namespace horus_unity_bridge
