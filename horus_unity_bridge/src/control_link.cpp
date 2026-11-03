// Copyright 2026 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0
#include "horus_unity_bridge/control_link.hpp"
#include "horus_unity_bridge/control_deadline.hpp"
#include "horus_unity_bridge/control_lease_manager.hpp"
#include "horus_unity_bridge/serialized_payload.hpp"
#include <rtc/rtc.hpp>
#include <rclcpp/serialization.hpp>
#include <std_msgs/msg/string.hpp>
#include <algorithm>
#include <atomic>
#include <deque>
#include <map>
#include <mutex>
#include <set>
#include <chrono>
#include <limits>

namespace horus_unity_bridge
{
namespace
{
using Json = nlohmann::json;
int64_t now_ms()
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
std::string decode_string(const std::vector<uint8_t> & bytes)
{
  rclcpp::SerializedMessage serialized;
  detail::fill_serialized_message(detail::add_cdr_header(bytes), serialized);
  std_msgs::msg::String message;
  rclcpp::Serialization<std_msgs::msg::String>().deserialize_message(&serialized, &message);
  return message.data;
}
bool is_zero_twist(const std::string & type, const std::vector<uint8_t> & payload)
{
  return type == "geometry_msgs/msg/Twist" && payload.size() == 48 &&
         std::all_of(payload.begin(), payload.end(), [](uint8_t b) {return b == 0;});
}
}

struct ControlLink::Impl : std::enable_shared_from_this<Impl>
{
  struct Peer
  {
    std::string id, local_sdp;
    std::shared_ptr<rtc::PeerConnection> pc;
    std::shared_ptr<rtc::DataChannel> reliable, motion;
    int64_t started = now_ms(), last_seen = 0, last_signal = 0, last_ping = 0;
    uint64_t sequence = 0;
    control::ClockEstimate clock;
    std::set<int> clients;
    std::set<int> retired_clients;
    bool retire = false;
    std::map<std::string, control::SequenceDeadline> ordering;
    struct Motion {int client; horuslink::ChannelDescriptor channel; int64_t deadline;};
    std::map<std::string, Motion> active;
  };
  struct Event {std::string peer; Json data; bool reliable;};
  rclcpp::Node * node;
  bool proxy;
  Apply apply;
  Disconnect disconnect;
  Snapshot snapshot;
  Catalog catalog_snapshot;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr signaling, lease_output, status;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr signals;
  mutable std::recursive_mutex mutex;
  std::mutex inbox_mutex;
  std::deque<Event> inbox;
  std::map<std::string, std::shared_ptr<Peer>> peers;
  std::set<std::string> protected_topics;
  std::set<std::string> fleet_topics;
  Json source_catalog;
  std::string latest_lease_state;
  std::map<int, std::pair<horuslink::ChannelDescriptor, std::vector<uint8_t>>> catalogs;
  std::map<std::pair<std::string, int>, int> client_ids;
  std::map<int, int> wire_clients;
  int next_wire_client = 1;
  std::set<int> blocked_clients;
  int next_client = 1000000000;
  int64_t retry_at = 0, last_status = 0;
  uint64_t accepted = 0, rejected = 0, acknowledged = 0;
  std::atomic<uint64_t> inbox_dropped{0};
  std::string mode;

  Impl(rclcpp::Node * n, std::string m, Apply a, Disconnect d, Snapshot s, Catalog c)
  : node(n), proxy(m == "proxy"), apply(std::move(a)), disconnect(std::move(d)),
    snapshot(std::move(s)), catalog_snapshot(std::move(c)), mode(std::move(m)) {}

  void refresh_catalog()
  {
    std::set<std::string> current = fleet_topics;
    if (!proxy && catalog_snapshot) {
      const auto topics = catalog_snapshot();
      current = {topics.begin(), topics.end()};
    } else if (proxy) {
      for (const auto & entry : catalogs) {
        auto root = Json::parse(decode_string(entry.second.second));
        for (const auto & robot : root.value("robots", Json::array())) {
          for (const auto & topic : robot.value("protected_topics", Json::array())) {
            if (topic.is_string()) {current.insert(topic.get<std::string>());}
          }
          for (const char * field : {"teleop_command_topic", "teleop_raw_input_topic", "teleop_head_pose_topic",
              "go_to_goal_topic", "go_to_cancel_topic", "waypoint_path_topic"}) {
            if (robot.contains(field) && robot[field].is_string()) {
              const auto topic = robot[field].get<std::string>();
              if (!topic.empty()) {current.insert(topic);}
            }
          }
        }
      }
    }
    if (current == protected_topics) {return;}
    protected_topics = std::move(current);
    if (!proxy) {
      source_catalog = {{"kind", "catalog_state"}, {"topics", protected_topics}};
      for (const auto & peer : peers) {send(peer.second, source_catalog, true);}
    }
  }

  void initialize()
  {
    const auto input = node->declare_parameter<std::string>("control.client_signal_topic",
      "/horus/remote/control/client_signal");
    const auto output = node->declare_parameter<std::string>("control.server_signal_topic",
      "/horus/remote/control/server_signal");
    signaling = node->create_publisher<std_msgs::msg::String>(proxy ? input : output, 50);
    auto weak = weak_from_this();
    signals = node->create_subscription<std_msgs::msg::String>(proxy ? output : input, 50,
      [weak](const std_msgs::msg::String::SharedPtr value) {
        if (auto self = weak.lock()) {
          auto message = Json::parse(value->data, nullptr, false);
          if (!message.is_discarded()) {self->signal(message);}
        }
      });
    const auto qos = rclcpp::QoS(100).reliable().transient_local();
    if (proxy && !snapshot) {
      lease_output = node->create_publisher<std_msgs::msg::String>(ControlLeaseManager::lease_state_topic(), qos);
    }
    status = node->create_publisher<std_msgs::msg::String>("/horus/remote/control/" + mode + "/status", 10);
  }
  bool send(const std::shared_ptr<Peer> & peer, const Json & value, bool reliable)
  {
    const auto & channel = reliable ? peer->reliable : peer->motion;
    if (!channel || !channel->isOpen() || channel->bufferedAmount() > (reliable ? 65536U : 4096U)) {
      return false;
    }
    try {return channel->sendBuffer(Json::to_cbor(value));}
    catch (const std::exception &) {return false;}
  }
  void enqueue(const std::string & id, Json value, bool reliable)
  {
    std::lock_guard<std::mutex> lock(inbox_mutex);
    if (!reliable && value.value("kind", "") == "command") {
      for (auto it = inbox.rbegin(); it != inbox.rend(); ++it) {
        if (!it->reliable && it->peer == id && it->data.value("kind", "") == "command" &&
          it->data.value("topic", "") == value.value("topic", "") &&
          it->data.value("client", -1) == value.value("client", -1))
        {
          if (value.value("sequence", uint64_t(0)) > it->data.value("sequence", uint64_t(0))) {
            it->data = std::move(value);
          }
          return;
        }
      }
    }
    if (inbox.size() >= 128) {++inbox_dropped; return;}
    inbox.push_back({id, std::move(value), reliable});
  }
  void attach(const std::shared_ptr<Peer> & peer, std::shared_ptr<rtc::DataChannel> channel)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    const auto label = channel->label();
    if (label != "horus-control-v1" && label != "horus-motion-v1") {channel->close(); return;}
    const bool reliable = label == "horus-control-v1";
    (reliable ? peer->reliable : peer->motion) = channel;
    auto weak = weak_from_this();
    const auto id = peer->id;
    channel->onMessage([weak, id, reliable](rtc::message_variant value) {
      if (auto self = weak.lock()) {
        if (auto bytes = std::get_if<rtc::binary>(&value); bytes && bytes->size() <= 65536) {
          const auto * begin = reinterpret_cast<const uint8_t *>(bytes->data());
          auto decoded = Json::from_cbor(begin, begin + bytes->size(), true, false);
          if (!decoded.is_discarded() && decoded.is_object()) {self->enqueue(id, std::move(decoded), reliable);}
        }
      }
    });
  }
  std::shared_ptr<Peer> make_peer(const std::string & id)
  {
    auto peer = std::make_shared<Peer>();
    peer->id = id;
    rtc::Configuration config;
    config.disableAutoNegotiation = true;
    peer->pc = std::make_shared<rtc::PeerConnection>(config);
    peers[id] = peer;
    auto weak = weak_from_this();
    std::weak_ptr<Peer> target = peer;
    peer->pc->onDataChannel([weak, target](std::shared_ptr<rtc::DataChannel> channel) {
      if (auto self = weak.lock()) {if (auto peer = target.lock()) {self->attach(peer, channel);}}
    });
    peer->pc->onGatheringStateChange([weak, target](rtc::PeerConnection::GatheringState state) {
      if (state != rtc::PeerConnection::GatheringState::Complete) {return;}
      if (auto self = weak.lock()) {
        if (auto peer = target.lock()) {
          // Network callbacks only enqueue; negotiation runs on the node executor.
          auto description = peer->pc->localDescription();
          if (description) {self->enqueue(peer->id, {{"kind", "sdp"}, {"sdp", std::string(*description)}}, true);}
        }
      }
    });
    if (proxy) {
      attach(peer, peer->pc->createDataChannel("horus-control-v1"));
      rtc::DataChannelInit motion;
      motion.reliability.unordered = true;
      motion.reliability.maxRetransmits = 0;
      attach(peer, peer->pc->createDataChannel("horus-motion-v1", motion));
      peer->pc->setLocalDescription(rtc::Description::Type::Offer);
    }
    return peer;
  }
  void signal(const Json & value)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    try {
      if (value.value("version", 0) != 1) {return;}
      const auto id = value.value("session", ""), type = value.value("type", "");
      if (id.empty() || id.size() > 128) {return;}
      auto found = peers.find(id);
      if (type == "offer" && !proxy && found == peers.end() && peers.size() < 16) {
        auto peer = make_peer(id);
        peer->pc->setRemoteDescription(rtc::Description(value.at("sdp").get<std::string>(), "offer"));
        peer->pc->setLocalDescription(rtc::Description::Type::Answer);
      } else if (found != peers.end() && type == "answer" && proxy &&
        !found->second->pc->remoteDescription())
      {
        found->second->pc->setRemoteDescription(rtc::Description(value.at("sdp").get<std::string>(), "answer"));
      } else if (found != peers.end() && type == "stop") {close(found->second); peers.erase(found);}
    } catch (const std::exception &) {++rejected;}
  }
  int client_id(const std::shared_ptr<Peer> & peer, int client)
  {
    if (client < 1 || peer->retired_clients.count(client)) {return -1;}
    const auto key = std::make_pair(peer->id, client);
    auto found = client_ids.find(key);
    if (found != client_ids.end()) {return found->second;}
    if (peer->clients.size() >= 64 || next_client == std::numeric_limits<int>::max()) {return -1;}
    const int id = next_client++;
    client_ids[key] = id;
    peer->clients.insert(id);
    return id;
  }
  void stop_motion(const std::shared_ptr<Peer> & peer, int client = -1)
  {
    for (auto it = peer->active.begin(); it != peer->active.end(); ) {
      if (client == -1 || it->second.client == client) {
        apply(it->second.client, it->second.channel, std::vector<uint8_t>(48, 0), 0);
        it = peer->active.erase(it);
      } else {++it;}
    }
  }
  void close(const std::shared_ptr<Peer> & peer)
  {
    if (!proxy) {
      stop_motion(peer);
      for (int client : peer->clients) {disconnect(client);}
      refresh_catalog();
    } else {
      // A dropped link needs a fresh release/lease action before motion resumes.
      for (const auto & catalog : catalogs) {blocked_clients.insert(catalog.first);}
      fleet_topics.clear(); refresh_catalog();
      const std::string state = Json({{"event", "control_link_lost"}, {"leases", Json::array()}}).dump();
      if (snapshot) {snapshot(state);}
      else if (lease_output) {std_msgs::msg::String message; message.data = state; lease_output->publish(message);}
    }
    for (auto it = client_ids.begin(); it != client_ids.end(); ) {
      if (it->first.first == peer->id) {it = client_ids.erase(it);} else {++it;}
    }
    if (peer->reliable) {peer->reliable->resetCallbacks(); peer->reliable->close();}
    if (peer->motion) {peer->motion->resetCallbacks(); peer->motion->close();}
    peer->pc->resetCallbacks(); peer->pc->close();
  }
  bool forward(int client, const horuslink::ChannelDescriptor & channel,
    const std::vector<uint8_t> & payload, uint8_t flags, int64_t remaining_input_ms = 200)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    const bool catalog = channel.topic == ControlLeaseManager::control_topic_catalog_topic();
    const bool lease = channel.topic == ControlLeaseManager::lease_request_topic();
    if (catalog) {
      auto root = Json::parse(decode_string(payload));
      const auto role = root.value("role", "");
      if (role == "host" || role == "single" || role == "private" || role == "joiner") {
        if (!catalogs.count(client) && catalogs.size() >= 64) {++rejected; return false;}
        catalogs[client] = {channel, payload};
        refresh_catalog();
      }
    }
    const bool zero = is_zero_twist(channel.type_name, payload);
    if (lease) {
      const auto request = Json::parse(decode_string(payload));
      if (request.value("action", "") == "release") {blocked_clients.erase(client);}
    }
    const bool continuous = !catalog && !lease && channel.delivery == horuslink::Delivery::ReplaceLatest;
    if (continuous && remaining_input_ms <= 0) {++rejected; return false;}
    if (payload.size() > 60000 || peers.empty() || (continuous && !zero && blocked_clients.count(client))) {
      ++rejected; return catalog;
    }
    auto peer = peers.begin()->second;
    const auto deadline = peer->clock.deadline(now_ms(), continuous ? std::clamp<int64_t>(remaining_input_ms, 1, 200) : 2000);
    if (!deadline && !catalog) {++rejected; return false;}
    if (!wire_clients.count(client)) {
      if (wire_clients.size() >= 64 || next_wire_client == std::numeric_limits<int>::max()) {++rejected; return false;}
      wire_clients[client] = next_wire_client++;
    }
    Json message = {{"kind", "command"}, {"client", wire_clients.at(client)}, {"sequence", ++peer->sequence},
      {"topic", channel.topic}, {"type", channel.type_name}, {"flags", flags},
      {"latest", continuous}, {"deadline", deadline}, {"data", Json::binary(payload)}};
    const bool sent = send(peer, message, !continuous || zero);
    if (!sent) {++rejected;}
    return sent || catalog;
  }
  void receive(const std::shared_ptr<Peer> & peer, const Json & value, bool reliable)
  {
    const auto kind = value.value("kind", "");
    const auto now = now_ms();
    if (kind == "sdp") {peer->local_sdp = value.at("sdp").get<std::string>(); return;}
    const bool first_message = !peer->last_seen;
    peer->last_seen = now;
    if (kind == "ping" && reliable) {
      send(peer, {{"kind", "pong"}, {"sent", value.at("sent")}, {"remote", now}}, true);
      if (first_message && !proxy) {
        if (!source_catalog.is_null()) {send(peer, source_catalog, true);}
        if (!latest_lease_state.empty()) {
          send(peer, {{"kind", "lease_state"}, {"data", latest_lease_state}}, true);
        }
      }
    } else if (kind == "pong" && proxy && reliable) {
      const bool was_valid = peer->clock.valid(now);
      peer->clock.observe(value.at("sent"), now, value.at("remote"));
      if (!was_valid && peer->clock.valid(now)) {
        for (const auto & catalog : catalogs) {forward(catalog.first, catalog.second.first, catalog.second.second, 0);}
      }
    } else if (kind == "lease_state" && proxy && reliable) {
      const auto data = value.at("data").get<std::string>();
      if (snapshot) {snapshot(data);}
      else {std_msgs::msg::String message; message.data = data; lease_output->publish(message);}
    } else if (kind == "catalog_state" && proxy && reliable) {
      fleet_topics.clear();
      for (const auto & topic : value.at("topics")) {
        if (topic.is_string()) {fleet_topics.insert(topic.get<std::string>());}
      }
      refresh_catalog();
    } else if (kind == "ack" && proxy && reliable) {++acknowledged;}
    else if (kind == "disconnect" && !proxy && reliable) {
      const int wire = value.at("client").get<int>();
      const auto found = client_ids.find({peer->id, wire});
      if (found == client_ids.end()) {return;}
      const int client = found->second;
      stop_motion(peer, client); disconnect(client); peer->clients.erase(client);
      client_ids.erase(found);
      peer->retired_clients.insert(wire);
      if (peer->retired_clients.size() >= 4096) {peer->retire = true;}
      const auto prefix = std::to_string(client) + ":";
      for (auto it = peer->ordering.begin(); it != peer->ordering.end(); ) {
        if (it->first.rfind(prefix, 0) == 0) {it = peer->ordering.erase(it);} else {++it;}
      }
      refresh_catalog();
    } else if (kind == "command" && !proxy) {
      const int client = client_id(peer, value.at("client"));
      if (client < 0) {++rejected; return;}
      const auto topic = value.at("topic").get<std::string>();
      const auto type = value.at("type").get<std::string>();
      const bool latest = value.value("latest", false);
      const auto sequence = value.at("sequence").get<uint64_t>();
      const auto key = std::to_string(client) + ":" + topic;
      const auto deadline = value.at("deadline").get<int64_t>();
      if (!peer->ordering.count(key) && peer->ordering.size() >= 4096) {++rejected; return;}
      auto & order = peer->ordering[key];
      bool valid = order.accept(sequence, deadline, now, latest ? 300 : 3000);
      const bool catalog = topic == ControlLeaseManager::control_topic_catalog_topic();
      if (catalog && reliable && !deadline) {valid = true;}
      auto bytes = value.at("data").get_binary();
      horuslink::ChannelDescriptor channel;
      channel.topic = topic; channel.type_name = type;
      channel.delivery = latest ? horuslink::Delivery::ReplaceLatest : horuslink::Delivery::ReliableFifo;
      bool applied = false;
      if (valid && (reliable || latest)) {
        if (reliable && topic == ControlLeaseManager::lease_request_topic()) {
          const auto request = Json::parse(decode_string(bytes));
          if (request.value("action", "") == "release") {stop_motion(peer, client);}
        }
        applied = apply(client, channel, bytes, value.value("flags", uint8_t(0)));
        if (applied && catalog) {refresh_catalog();}
        if (applied && latest && type == "geometry_msgs/msg/Twist" && bytes.size() == 48) {
          if (is_zero_twist(type, bytes)) {peer->active.erase(key);}
          else {peer->active[key] = {client, channel, deadline};}
        }
      }
      if (applied) {++accepted;} else {++rejected;}
      if (reliable) {send(peer, {{"kind", "ack"}, {"sequence", sequence}, {"applied", applied}}, true);}
    }
  }
  void tick()
  {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    const auto now = now_ms();
    if (proxy && peers.empty() && now >= retry_at) {
      make_peer("operator_" + std::to_string(now) + "_" + std::to_string(reinterpret_cast<uintptr_t>(this)));
    }
    std::deque<Event> events;
    {std::lock_guard<std::mutex> queue(inbox_mutex); events.swap(inbox);}
    for (const auto & event : events) {
      if (event.peer.empty() && !proxy && event.data.value("kind", "") == "lease_state") {
        latest_lease_state = event.data.at("data").get<std::string>();
        for (const auto & peer : peers) {send(peer.second, event.data, true);}
        continue;
      }
      auto peer = peers.find(event.peer);
      if (peer == peers.end()) {continue;}
      try {receive(peer->second, event.data, event.reliable);} catch (const std::exception &) {++rejected;}
    }
    for (auto it = peers.begin(); it != peers.end(); ) {
      auto peer = it->second;
      const auto state = peer->pc->state();
      if (peer->retire || state == rtc::PeerConnection::State::Failed || state == rtc::PeerConnection::State::Closed ||
        now - (peer->last_seen ? peer->last_seen : peer->started) > (peer->last_seen ? 3000 : 15000))
      {
        close(peer); it = peers.erase(it); retry_at = now + 1000; continue;
      }
      if (!peer->local_sdp.empty() && now - peer->last_signal >= 1000 && !peer->last_seen) {
        std_msgs::msg::String message;
        message.data = Json({{"version", 1}, {"session", peer->id}, {"type", proxy ? "offer" : "answer"},
          {"sdp", peer->local_sdp}}).dump();
        signaling->publish(message); peer->last_signal = now;
      }
      if (proxy && now - peer->last_ping >= 500) {
        if (send(peer, {{"kind", "ping"}, {"sent", now}}, true)) {peer->last_ping = now;}
      }
      for (auto active = peer->active.begin(); active != peer->active.end(); ) {
        if (active->second.deadline < now) {
          apply(active->second.client, active->second.channel, std::vector<uint8_t>(48, 0), 0);
          active = peer->active.erase(active);
        } else {++active;}
      }
      ++it;
    }
    if (now - last_status > 1000) {
      last_status = now;
      std_msgs::msg::String message; message.data = statistics().dump(); status->publish(message);
    }
  }
  Json statistics() const
  {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    Json links = Json::array();
    for (const auto & entry : peers) {
      links.push_back({{"session", entry.first}, {"connected", entry.second->pc->state() == rtc::PeerConnection::State::Connected},
        {"clock_valid", !proxy || entry.second->clock.valid(now_ms())},
        {"clock_uncertainty_ms", proxy ? entry.second->clock.uncertainty_ms : 0}});
    }
    return {{"mode", mode}, {"links", links}, {"accepted", accepted}, {"rejected", rejected},
      {"acknowledged", acknowledged}, {"inbox_dropped", inbox_dropped.load()},
      {"rearm_required_clients", blocked_clients.size()}};
  }
};

ControlLink::ControlLink(rclcpp::Node * node, const std::string & mode, Apply apply,
  Disconnect disconnect, Snapshot snapshot, Catalog catalog)
{
  if (mode != "proxy" && mode != "authority") {throw std::invalid_argument("control.mode must be local, proxy or authority");}
  impl_ = std::make_shared<Impl>(node, mode, std::move(apply), std::move(disconnect), std::move(snapshot), std::move(catalog));
  impl_->initialize();
}
ControlLink::~ControlLink()
{
  impl_->signals.reset();
  std::lock_guard<std::recursive_mutex> lock(impl_->mutex);
  for (const auto & entry : impl_->peers) {impl_->close(entry.second);}
  impl_->peers.clear();
}
bool ControlLink::is_proxy() const {return impl_->proxy;}
bool ControlLink::handles(const std::string & topic) const
{
  std::lock_guard<std::recursive_mutex> lock(impl_->mutex);
  return topic == ControlLeaseManager::lease_request_topic() ||
         topic == ControlLeaseManager::control_topic_catalog_topic() || impl_->protected_topics.count(topic) ||
         (topic.rfind("/horus/teach/", 0) == 0 && topic.size() > 19 &&
         topic.compare(topic.size() - 6, 6, "/input") == 0);
}
bool ControlLink::forward(int client, const horuslink::ChannelDescriptor & channel,
  const std::vector<uint8_t> & payload, uint8_t flags, int64_t remaining_input_ms)
{
  try {return impl_->forward(client, channel, payload, flags, remaining_input_ms);}
  catch (const std::exception &) {return false;}
}
void ControlLink::disconnected(int client)
{
  if (!impl_->proxy) {return;}
  std::lock_guard<std::recursive_mutex> lock(impl_->mutex);
  impl_->catalogs.erase(client); impl_->blocked_clients.erase(client);
  impl_->refresh_catalog();
  const auto found = impl_->wire_clients.find(client);
  if (found != impl_->wire_clients.end()) {
    for (const auto & entry : impl_->peers) {
      impl_->send(entry.second, {{"kind", "disconnect"}, {"client", found->second}}, true);
    }
    impl_->wire_clients.erase(found);
  }
}
void ControlLink::tick() {impl_->tick();}
void ControlLink::publish_lease_state(const std::string & snapshot)
{
  impl_->enqueue("", {{"kind", "lease_state"}, {"data", snapshot}}, true);
}
nlohmann::json ControlLink::statistics() const {return impl_->statistics();}
}  // namespace horus_unity_bridge
