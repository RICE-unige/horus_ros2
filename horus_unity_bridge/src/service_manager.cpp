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

// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#include "horus_unity_bridge/service_manager.hpp"
#include <rclcpp/typesupport_helpers.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>

namespace horus_unity_bridge
{

ServiceManager::GenericServiceClientWrapper::~GenericServiceClientWrapper()
{
  for (const auto & [id, call] : pending) {
    client->remove_pending_request(call.request_id);
  }
}

ServiceManager::ServiceManager(rclcpp::Node * node, std::chrono::milliseconds request_timeout)
: node_(node), request_timeout_(request_timeout)
{
  if (request_timeout <= std::chrono::milliseconds::zero()) {
    throw std::invalid_argument("Service request timeout must be positive");
  }
}

bool ServiceManager::register_ros_service(
  const std::string & service_name,
  const std::string & service_type)
{
  std::lock_guard<std::mutex> lock(ros_clients_mutex_);
  const auto normalized_type = normalize_service_type(service_type);
  auto existing = ros_service_clients_.find(service_name);
  if (existing != ros_service_clients_.end()) {
    return existing->second->type_name == normalized_type;
  }
  try {
    auto wrapper = std::make_shared<GenericServiceClientWrapper>();
    wrapper->type_support_library =
      rclcpp::get_typesupport_library(normalized_type, "rosidl_typesupport_cpp");
    wrapper->type_support = rclcpp::get_service_typesupport_handle(
      normalized_type, "rosidl_typesupport_cpp", *wrapper->type_support_library);
    wrapper->type_name = normalized_type;
    wrapper->client = node_->create_generic_client(
      service_name, normalized_type, rclcpp::ServicesQoS());
    ros_service_clients_.emplace(service_name, std::move(wrapper));
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to register ROS service %s: %s",
      service_name.c_str(), e.what());
    stats_.errors++;
    return false;
  }
}

bool ServiceManager::call_ros_service_async(
  uint32_t srv_id,
  const std::string & service_name,
  const std::vector<uint8_t> & request)
{
  std::lock_guard<std::mutex> lock(ros_clients_mutex_);
  auto it = ros_service_clients_.find(service_name);
  if (srv_id == 0 || pending_service_names_.count(srv_id) ||
    it == ros_service_clients_.end() || !it->second->client->service_is_ready())
  {
    stats_.errors++;
    return false;
  }
  auto & wrapper = it->second;
  try {
    // GenericClient accepts a native ROS message, not a serialized-message struct.
    const auto * request_support = wrapper->type_support->request_typesupport;
    const auto * introspection = get_message_typesupport_handle(
      request_support, rosidl_typesupport_introspection_cpp::typesupport_identifier);
    if (!introspection) {
      throw std::runtime_error("Request introspection is unavailable");
    }
    const auto * members =
      static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
      introspection->data);
    auto storage = std::make_unique<uint8_t[]>(members->size_of_);
    members->init_function(storage.get(), rosidl_runtime_cpp::MessageInitialization::ZERO);
    std::unique_ptr<void, std::function<void(void *)>> native_request(
      storage.release(), [members](void * value) {
        members->fini_function(value);
        delete[] static_cast<uint8_t *>(value);
      });
    rclcpp::SerializedMessage serialized;
    detail::fill_serialized_message(detail::add_cdr_header(request), serialized);
    rclcpp::SerializationBase(request_support).deserialize_message(
      &serialized, native_request.get());

    auto result = wrapper->client->async_send_request(native_request.get());
    GenericServiceClientWrapper::PendingCall call{
      result.share(), std::chrono::steady_clock::now(), result.request_id};
    wrapper->pending.emplace(srv_id, std::move(call));
    pending_service_names_.emplace(srv_id, service_name);
    stats_.ros_services_called++;
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to call ROS service %s: %s",
      service_name.c_str(), e.what());
    stats_.errors++;
    return false;
  }
}

bool ServiceManager::cancel_ros_service_call(uint32_t srv_id)
{
  std::lock_guard<std::mutex> lock(ros_clients_mutex_);
  auto name = pending_service_names_.find(srv_id);
  if (name == pending_service_names_.end()) {
    return false;
  }
  auto & wrapper = ros_service_clients_.at(name->second);
  auto call = wrapper->pending.find(srv_id);
  if (call != wrapper->pending.end()) {
    wrapper->client->remove_pending_request(call->second.request_id);
    wrapper->pending.erase(call);
  }
  pending_service_names_.erase(name);
  return true;
}

void ServiceManager::process_pending_responses()
{
  struct Completion
  {
    uint32_t id;
    bool success;
    std::vector<uint8_t> payload;
  };
  std::vector<Completion> completed;
  {
    std::lock_guard<std::mutex> lock(ros_clients_mutex_);
    const auto now = std::chrono::steady_clock::now();
    for (auto name = pending_service_names_.begin(); name != pending_service_names_.end(); ) {
      auto & wrapper = ros_service_clients_.at(name->second);
      auto call = wrapper->pending.find(name->first);
      const bool ready =
        call->second.future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready;
      if (!ready && now - call->second.start_time < request_timeout_) {
        ++name;
        continue;
      }
      Completion result{name->first, false, {}};
      if (ready) {
        try {
          auto native_response = call->second.future.get();
          rclcpp::SerializedMessage serialized;
          rclcpp::SerializationBase(wrapper->type_support->response_typesupport)
          .serialize_message(native_response.get(), &serialized);
          const auto & data = serialized.get_rcl_serialized_message();
          result.payload.assign(data.buffer, data.buffer + data.buffer_length);
          result.success = true;
          stats_.responses_sent++;
        } catch (const std::exception & e) {
          RCLCPP_ERROR(node_->get_logger(), "ROS service response failed: %s", e.what());
          stats_.errors++;
        }
      } else {
        wrapper->client->remove_pending_request(call->second.request_id);
        stats_.timeouts++;
        RCLCPP_WARN(node_->get_logger(), "ROS service timed out: %s", name->second.c_str());
      }
      wrapper->pending.erase(call);
      name = pending_service_names_.erase(name);
      completed.push_back(std::move(result));
    }
  }
  // Never call routing/user code while holding the client lifecycle lock.
  for (const auto & result : completed) {
    if (result.success && ros_response_callback_) {
      ros_response_callback_(result.id, result.payload);
    } else if (!result.success && ros_failure_callback_) {
      ros_failure_callback_(result.id);
    }
  }
}

bool ServiceManager::register_unity_service(
  const std::string & service_name,
  const std::string & service_type,
  UnityServiceRequestCallback request_callback)
{
  std::lock_guard<std::mutex> lock(unity_servers_mutex_);

  if (unity_service_servers_.find(service_name) != unity_service_servers_.end()) {
    return true;
  }

  auto server = create_unity_service_server(service_name, service_type, request_callback);

  if (!server) {
    stats_.errors++;
    return false;
  }

  unity_service_servers_[service_name] = server;

  RCLCPP_INFO(node_->get_logger(), "Registered Typed Unity service server: %s [%s]",
              service_name.c_str(), service_type.c_str());
  return true;
}

bool ServiceManager::unregister_ros_service(const std::string & service_name)
{
  std::vector<uint32_t> cancelled;
  {
    std::lock_guard<std::mutex> lock(ros_clients_mutex_);
    auto it = ros_service_clients_.find(service_name);
    if (it == ros_service_clients_.end()) {
      return false;
    }
    for (const auto & [id, call] : it->second->pending) {
      cancelled.push_back(id);
      pending_service_names_.erase(id);
    }
    ros_service_clients_.erase(it);
  }
  for (auto id : cancelled) {
    if (ros_failure_callback_) {
      ros_failure_callback_(id);
    }
  }
  return true;
}

bool ServiceManager::unregister_unity_service(const std::string & service_name)
{
  std::lock_guard<std::mutex> lock(unity_servers_mutex_);
  auto it = unity_service_servers_.find(service_name);
  if (it == unity_service_servers_.end()) {
    return false;
  }
  unity_service_servers_.erase(it);
  RCLCPP_INFO(node_->get_logger(), "Unregistered Unity service server: %s", service_name.c_str());
  return true;
}

void ServiceManager::handle_unity_service_response(
  uint32_t srv_id,
  const std::vector<uint8_t> & response)
{
  std::lock_guard<std::mutex> lock(unity_servers_mutex_);

  auto name_it = pending_unity_service_names_.find(srv_id);
  if (name_it == pending_unity_service_names_.end()) {
    RCLCPP_WARN(node_->get_logger(), "Received response for unknown/expired Unity service ID: %u",
        srv_id);
    return;
  }

  std::string service_name = name_it->second;
  auto server_it = unity_service_servers_.find(service_name);
  if (server_it == unity_service_servers_.end()) {
    RCLCPP_ERROR(node_->get_logger(), "Unity service server not found: %s", service_name.c_str());
    pending_unity_service_names_.erase(name_it);
    return;
  }

  // Try to handle as AddTwoInts
  auto add_two_ints_server = std::dynamic_pointer_cast<TypedUnityServiceServer<example_interfaces::srv::AddTwoInts>>(
    server_it->second);
  if (add_two_ints_server) {
    std::lock_guard<std::mutex> srv_lock(add_two_ints_server->mutex);
    auto it = add_two_ints_server->pending.find(srv_id);
    if (it != add_two_ints_server->pending.end()) {
      // Deserialize response
      auto resp = std::make_shared<example_interfaces::srv::AddTwoInts::Response>();
      const auto normalized = detail::add_cdr_header(response);
      rclcpp::SerializedMessage serialized_msg;
      detail::fill_serialized_message(normalized, serialized_msg);

      rclcpp::Serialization<example_interfaces::srv::AddTwoInts::Response> serializer;
      serializer.deserialize_message(&serialized_msg, resp.get());

      it->second.set_value(resp);
      add_two_ints_server->pending.erase(it);
      stats_.responses_sent++;
      pending_unity_service_names_.erase(name_it);
      return;
    }
  }

  // Try Trigger
  auto trigger_server = std::dynamic_pointer_cast<TypedUnityServiceServer<std_srvs::srv::Trigger>>(
    server_it->second);
  if (trigger_server) {
    std::lock_guard<std::mutex> srv_lock(trigger_server->mutex);
    auto it = trigger_server->pending.find(srv_id);
    if (it != trigger_server->pending.end()) {
      auto resp = std::make_shared<std_srvs::srv::Trigger::Response>();
      const auto normalized = detail::add_cdr_header(response);
      rclcpp::SerializedMessage serialized_msg;
      detail::fill_serialized_message(normalized, serialized_msg);

      rclcpp::Serialization<std_srvs::srv::Trigger::Response> serializer;
      serializer.deserialize_message(&serialized_msg, resp.get());

      it->second.set_value(resp);
      trigger_server->pending.erase(it);
      stats_.responses_sent++;
      pending_unity_service_names_.erase(name_it);
      return;
    }
  }

  // Try SetBool
  auto set_bool_server = std::dynamic_pointer_cast<TypedUnityServiceServer<std_srvs::srv::SetBool>>(
    server_it->second);
  if (set_bool_server) {
    std::lock_guard<std::mutex> srv_lock(set_bool_server->mutex);
    auto it = set_bool_server->pending.find(srv_id);
    if (it != set_bool_server->pending.end()) {
      auto resp = std::make_shared<std_srvs::srv::SetBool::Response>();
      const auto normalized = detail::add_cdr_header(response);
      rclcpp::SerializedMessage serialized_msg;
      detail::fill_serialized_message(normalized, serialized_msg);

      rclcpp::Serialization<std_srvs::srv::SetBool::Response> serializer;
      serializer.deserialize_message(&serialized_msg, resp.get());

      it->second.set_value(resp);
      set_bool_server->pending.erase(it);
      stats_.responses_sent++;
      pending_unity_service_names_.erase(name_it);
      return;
    }
  }

  RCLCPP_ERROR(node_->get_logger(), "Failed to handle Unity service response [ID: %u]", srv_id);
}

std::shared_ptr<ServiceManager::UnityServiceServerBase> ServiceManager::create_unity_service_server(
  const std::string & service_name,
  const std::string & service_type,
  UnityServiceRequestCallback callback)
{
  // Factory for creating typed Unity service servers
  if (service_type.find("AddTwoInts") != std::string::npos) {
    auto service_callback = [this, callback, service_name](
      const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
      std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
      {
      // Generate unique service ID
        uint32_t srv_id;
        {
          std::lock_guard<std::mutex> lock(unity_servers_mutex_);
          srv_id = next_service_id_++;
          pending_unity_service_names_[srv_id] = service_name;
        }

      // Serialize request
        rclcpp::Serialization<example_interfaces::srv::AddTwoInts::Request> serializer;
        rclcpp::SerializedMessage serialized_msg;
        serializer.serialize_message(request.get(), &serialized_msg);

        auto & rcl_msg = serialized_msg.get_rcl_serialized_message();
        std::vector<uint8_t> request_data(rcl_msg.buffer, rcl_msg.buffer + rcl_msg.buffer_length);

      // Create promise for response
        std::promise<example_interfaces::srv::AddTwoInts::Response::SharedPtr> response_promise;
        auto response_future = response_promise.get_future();

      // Store pending call
        auto server_ptr = std::static_pointer_cast<TypedUnityServiceServer<example_interfaces::srv::AddTwoInts>>(
        unity_service_servers_[service_name]);
        {
          std::lock_guard<std::mutex> lock(server_ptr->mutex);
          server_ptr->pending[srv_id] = std::move(response_promise);
        }

        stats_.unity_services_called++;

      // Send request to Unity
        if (callback) {
          callback(srv_id, service_name, request_data);
        }

      // Wait for Unity response (with timeout)
        auto status = response_future.wait_for(std::chrono::seconds(10));

        if (status == std::future_status::ready) {
          auto resp = response_future.get();
          *response = *resp;
        } else {
          RCLCPP_ERROR(node_->get_logger(), "Unity service timeout: %s [ID: %u]",
                     service_name.c_str(), srv_id);
          stats_.timeouts++;

          {
            std::lock_guard<std::mutex> lock(server_ptr->mutex);
            server_ptr->pending.erase(srv_id);
          }
          {
            std::lock_guard<std::mutex> lock(unity_servers_mutex_);
            pending_unity_service_names_.erase(srv_id);
          }
        }
      };

    auto server = node_->create_service<example_interfaces::srv::AddTwoInts>(
      service_name, service_callback);

    auto unity_server = std::make_shared<TypedUnityServiceServer<example_interfaces::srv::AddTwoInts>>(
      server, callback);

    return unity_server;
  } else if (service_type.find("Trigger") != std::string::npos) {
    auto service_callback = [this, callback, service_name](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        uint32_t srv_id;
        {
          std::lock_guard<std::mutex> lock(unity_servers_mutex_);
          srv_id = next_service_id_++;
          pending_unity_service_names_[srv_id] = service_name;
        }

        rclcpp::Serialization<std_srvs::srv::Trigger::Request> serializer;
        rclcpp::SerializedMessage serialized_msg;
        serializer.serialize_message(request.get(), &serialized_msg);

        auto & rcl_msg = serialized_msg.get_rcl_serialized_message();
        std::vector<uint8_t> request_data(rcl_msg.buffer, rcl_msg.buffer + rcl_msg.buffer_length);

        std::promise<std_srvs::srv::Trigger::Response::SharedPtr> response_promise;
        auto response_future = response_promise.get_future();

        auto server_ptr = std::static_pointer_cast<TypedUnityServiceServer<std_srvs::srv::Trigger>>(
        unity_service_servers_[service_name]);
        {
          std::lock_guard<std::mutex> lock(server_ptr->mutex);
          server_ptr->pending[srv_id] = std::move(response_promise);
        }

        stats_.unity_services_called++;

        if (callback) {
          callback(srv_id, service_name, request_data);
        }

        auto status = response_future.wait_for(std::chrono::seconds(10));

        if (status == std::future_status::ready) {
          auto resp = response_future.get();
          *response = *resp;
        } else {
          RCLCPP_ERROR(node_->get_logger(), "Unity service timeout: %s [ID: %u]",
                     service_name.c_str(), srv_id);
          stats_.timeouts++;
          {
            std::lock_guard<std::mutex> lock(server_ptr->mutex);
            server_ptr->pending.erase(srv_id);
          }
          {
            std::lock_guard<std::mutex> lock(unity_servers_mutex_);
            pending_unity_service_names_.erase(srv_id);
          }
        }
      };

    auto server = node_->create_service<std_srvs::srv::Trigger>(service_name, service_callback);
    return std::make_shared<TypedUnityServiceServer<std_srvs::srv::Trigger>>(server, callback);
  } else if (service_type.find("SetBool") != std::string::npos) {
    auto service_callback = [this, callback, service_name](
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response)
      {
        uint32_t srv_id;
        {
          std::lock_guard<std::mutex> lock(unity_servers_mutex_);
          srv_id = next_service_id_++;
          pending_unity_service_names_[srv_id] = service_name;
        }

        rclcpp::Serialization<std_srvs::srv::SetBool::Request> serializer;
        rclcpp::SerializedMessage serialized_msg;
        serializer.serialize_message(request.get(), &serialized_msg);

        auto & rcl_msg = serialized_msg.get_rcl_serialized_message();
        std::vector<uint8_t> request_data(rcl_msg.buffer, rcl_msg.buffer + rcl_msg.buffer_length);

        std::promise<std_srvs::srv::SetBool::Response::SharedPtr> response_promise;
        auto response_future = response_promise.get_future();

        auto server_ptr = std::static_pointer_cast<TypedUnityServiceServer<std_srvs::srv::SetBool>>(
        unity_service_servers_[service_name]);
        {
          std::lock_guard<std::mutex> lock(server_ptr->mutex);
          server_ptr->pending[srv_id] = std::move(response_promise);
        }

        stats_.unity_services_called++;

        if (callback) {
          callback(srv_id, service_name, request_data);
        }

        auto status = response_future.wait_for(std::chrono::seconds(10));

        if (status == std::future_status::ready) {
          auto resp = response_future.get();
          *response = *resp;
        } else {
          RCLCPP_ERROR(node_->get_logger(), "Unity service timeout: %s [ID: %u]",
                     service_name.c_str(), srv_id);
          stats_.timeouts++;
          {
            std::lock_guard<std::mutex> lock(server_ptr->mutex);
            server_ptr->pending.erase(srv_id);
          }
          {
            std::lock_guard<std::mutex> lock(unity_servers_mutex_);
            pending_unity_service_names_.erase(srv_id);
          }
        }
      };

    auto server = node_->create_service<std_srvs::srv::SetBool>(service_name, service_callback);
    return std::make_shared<TypedUnityServiceServer<std_srvs::srv::SetBool>>(server, callback);
  }

  RCLCPP_ERROR(node_->get_logger(), "Unsupported Unity service type: %s", service_type.c_str());
  return nullptr;
}

std::string ServiceManager::normalize_service_type(const std::string & type)
{
  // Same logic as TopicManager but ensuring /srv/
  std::string normalized = type;
  if (normalized.find("/srv/") != std::string::npos) {
    return normalized;
  }
  size_t slash_pos = normalized.find('/');
  if (slash_pos != std::string::npos) {
    return normalized.substr(0, slash_pos) + "/srv/" + normalized.substr(slash_pos + 1);
  }
  return normalized;
}

} // namespace horus_unity_bridge
