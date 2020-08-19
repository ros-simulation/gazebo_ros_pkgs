// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include <gazebo_ros/qos.hpp>

#include <rclcpp/expand_topic_or_service_name.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rcl/rcl.h>
#include <rcl/remap.h>
#include <rmw/types.h>

#include <chrono>
#include <map>
#include <memory>
#include <unordered_map>
#include <sstream>
#include <string>
#include <utility>

namespace gazebo_ros
{

/// Storage container for QoS overrides for a single profile.
struct QoSOverrides
{
  rmw_qos_reliability_policy_t reliability;
  rmw_qos_durability_policy_t durability;
  rmw_qos_history_policy_t history;
  rmw_qos_liveliness_policy_t liveliness;
  size_t depth;
  std::chrono::milliseconds deadline;
  std::chrono::milliseconds lifespan;
  std::chrono::milliseconds liveliness_lease;

  QoSOverrides()
  : reliability(RMW_QOS_POLICY_RELIABILITY_UNKNOWN),
    durability(RMW_QOS_POLICY_DURABILITY_UNKNOWN),
    history(RMW_QOS_POLICY_HISTORY_UNKNOWN),
    liveliness(RMW_QOS_POLICY_LIVELINESS_UNKNOWN),
    depth(0),
    deadline(0),
    lifespan(0),
    liveliness_lease(0)
  {}
};

struct QoSPrivate
{
  /// Helper function for parsing an SDF to get QoS overrides.
  static QoSOverrides get_qos_overrides_from_sdf(sdf::ElementPtr _sdf);

  /// Helper function for applying overrides on top of a default QoS profile.
  static rclcpp::QoS apply_overrides(
    const QoSOverrides & overrides, const rclcpp::QoS default_qos);

  /// Get the remapped name for a topic
  std::string get_remapped_topic_name(const std::string topic) const;

  /// Map topic names to publisher QoS overrides
  std::map<std::string, QoSOverrides> publisher_qos_overrides_map_;
  /// Map topic names to subscription QoS overrides
  std::map<std::string, QoSOverrides> subscription_qos_overrides_map_;

  // The following members are needed for getting topic remappings

  /// The name of the node associated with the QoS settings
  std::string node_name_;

  /// The namespace of the node associated with the QoS settings
  std::string node_namespace_;

  /// The options of the node associated with the QoS settings
  rclcpp::NodeOptions node_options_;
};

/// Helper function for creating an rclcpp::QoS from SDF and a default QoS as a base.
QoSOverrides QoSPrivate::get_qos_overrides_from_sdf(sdf::ElementPtr _sdf)
{
  // Map strings to QoS policies
  static std::unordered_map<std::string, rmw_qos_reliability_policy_t> reliability_map = {
    {"system", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT},
    {"reliable", RMW_QOS_POLICY_RELIABILITY_RELIABLE},
    {"best_effort", RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT},
  };
  static std::unordered_map<std::string, rmw_qos_durability_policy_t> durability_map = {
    {"system", RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT},
    {"volatile", RMW_QOS_POLICY_DURABILITY_VOLATILE},
    {"transient_local", RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL},
  };
  static std::unordered_map<std::string, rmw_qos_history_policy_t> history_map = {
    {"system", RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT},
    {"keep_last", RMW_QOS_POLICY_HISTORY_KEEP_LAST},
    {"keep_all", RMW_QOS_POLICY_HISTORY_KEEP_ALL},
  };
  static std::unordered_map<std::string, rmw_qos_liveliness_policy_t> liveliness_map = {
    {"system", RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT},
    {"automatic", RMW_QOS_POLICY_LIVELINESS_AUTOMATIC},
    {"manual_by_topic", RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC},
  };

  QoSOverrides qos_overrides;

  // Parse 'reliability' QoS
  if (_sdf->HasElement("reliability")) {
    auto reliability = _sdf->GetElement("reliability")->Get<std::string>();
    auto find_result = reliability_map.find(reliability);
    if (find_result != reliability_map.end()) {
      qos_overrides.reliability = find_result->second;
    } else {
      std::ostringstream oss;
      oss << "invalid setting for reliability '" << reliability << "'";
      throw InvalidQoSException(oss.str());
    }
  }

  // Parse 'durability' QoS
  if (_sdf->HasElement("durability")) {
    auto durability = _sdf->GetElement("durability")->Get<std::string>();
    auto find_result = durability_map.find(durability);
    if (find_result != durability_map.end()) {
      qos_overrides.durability = find_result->second;
    } else {
      std::ostringstream oss;
      oss << "invalid setting for durability '" << durability << "'";
      throw InvalidQoSException(oss.str());
    }
  }

  // Parse 'history' QoS
  if (_sdf->HasElement("history")) {
    auto history_element = _sdf->GetElement("history");
    auto history = history_element->Get<std::string>();
    auto find_result = history_map.find(history);
    if (find_result != history_map.end()) {
      qos_overrides.history = find_result->second;
      if (RMW_QOS_POLICY_HISTORY_KEEP_LAST == find_result->second) {
        if (!history_element->HasAttribute("depth")) {
          std::ostringstream oss;
          oss << "'" << find_result->first << "' used without providing a depth";
          throw InvalidQoSException(oss.str());
        }
        qos_overrides.depth = history_element->Get<uint64_t>("depth");
      }
    } else {
      std::ostringstream oss;
      oss << "invalid setting for history '" << history << "'";
      throw InvalidQoSException(oss.str());
    }
  }

  // Parse 'deadline' QoS
  if (_sdf->HasElement("deadline")) {
    auto deadline = _sdf->GetElement("deadline")->Get<uint64_t>();
    qos_overrides.deadline = std::chrono::milliseconds(deadline);
  }

  // Parse 'lifespan' QoS
  if (_sdf->HasElement("lifespan")) {
    auto lifespan = _sdf->GetElement("lifespan")->Get<uint64_t>();
    qos_overrides.lifespan = std::chrono::milliseconds(lifespan);
  }

  // Parse 'liveliness' QoS
  if (_sdf->HasElement("liveliness")) {
    auto liveliness = _sdf->GetElement("liveliness")->Get<std::string>();
    auto find_result = liveliness_map.find(liveliness);
    if (find_result != liveliness_map.end()) {
      qos_overrides.liveliness = find_result->second;
    } else {
      std::ostringstream oss;
      oss << "invalid setting for liveliness '" << liveliness << "'";
      throw InvalidQoSException(oss.str());
    }
  }

  // Parse 'liveliness_lease_duration' QoS
  if (_sdf->HasElement("liveliness_lease_duration")) {
    auto lease = _sdf->GetElement("liveliness_lease_duration")->Get<uint64_t>();
    qos_overrides.liveliness_lease = std::chrono::milliseconds(lease);
  }

  return qos_overrides;
}

QoS::QoS()
: impl_(std::make_unique<QoSPrivate>()) {}

// Cannot do this in header, due to QoSPrivate being an incomplete type.
QoS::QoS(QoS && other) = default;
QoS & QoS::operator=(QoS && other) = default;
QoS::~QoS() = default;

QoS::QoS(const QoS & other)
: impl_(nullptr)
{
  if (!other.impl_) {
    std::runtime_error("QoS object with null implementation");
  }
  impl_ = std::make_unique<QoSPrivate>(*other.impl_);
}

QoS & QoS::operator=(const QoS & other)
{
  QoS copy = other;
  *this = std::move(copy);
  return *this;
}

QoS::QoS(
  sdf::ElementPtr _sdf,
  const std::string node_name,
  const std::string node_namespace,
  const rclcpp::NodeOptions & options)
: QoS()
{
  impl_->node_name_ = node_name;
  impl_->node_namespace_ = node_namespace;
  impl_->node_options_ = options;

  // If there's no <qos> tag, then there's nothing to do
  if (!_sdf->HasElement("qos")) {
    return;
  }
  sdf::ElementPtr qos_sdf = _sdf->GetElement("qos");

  // If there's no <topic> tags, then there's nothing to do
  if (!qos_sdf->HasElement("topic")) {
    return;
  }
  sdf::ElementPtr topic_sdf = qos_sdf->GetElement("topic");

  while (topic_sdf) {
    if (!topic_sdf->HasAttribute("name")) {
      throw InvalidQoSException("topic element missing required 'name' attribute");
    }
    auto topic_name = topic_sdf->Get<std::string>("name");
    std::string fqn_topic_name = rclcpp::expand_topic_or_service_name(
      topic_name,
      this->impl_->node_name_,
      this->impl_->node_namespace_,
      false);  // false = not a service

    // For each topic, get the publisher QoS overrides
    if (topic_sdf->HasElement("publisher")) {
      impl_->publisher_qos_overrides_map_.emplace(
        fqn_topic_name, QoSPrivate::get_qos_overrides_from_sdf(topic_sdf->GetElement("publisher")));
    }

    // For each topic, get the subscription QoS overrides
    if (topic_sdf->HasElement("subscription")) {
      impl_->subscription_qos_overrides_map_.emplace(
        fqn_topic_name,
        QoSPrivate::get_qos_overrides_from_sdf(topic_sdf->GetElement("subscription")));
    }

    topic_sdf = topic_sdf->GetNextElement("topic");
  }
}

rclcpp::QoS QoSPrivate::apply_overrides(
  const QoSOverrides & overrides, const rclcpp::QoS default_qos)
{
  rclcpp::QoS qos = default_qos;
  if (overrides.reliability != RMW_QOS_POLICY_RELIABILITY_UNKNOWN) {
    qos.reliability(overrides.reliability);
  }

  if (overrides.durability != RMW_QOS_POLICY_DURABILITY_UNKNOWN) {
    qos.durability(overrides.durability);
  }

  if (overrides.history == RMW_QOS_POLICY_HISTORY_KEEP_ALL) {
    qos.keep_all();
  } else if (overrides.history == RMW_QOS_POLICY_HISTORY_KEEP_LAST) {
    qos.keep_last(overrides.depth);
  }

  if (overrides.liveliness != RMW_QOS_POLICY_LIVELINESS_UNKNOWN) {
    qos.liveliness(overrides.liveliness);
  }

  if (overrides.deadline != std::chrono::milliseconds::zero()) {
    qos.deadline(overrides.deadline);
  }

  if (overrides.lifespan != std::chrono::milliseconds::zero()) {
    qos.lifespan(overrides.lifespan);
  }

  if (overrides.liveliness_lease != std::chrono::milliseconds::zero()) {
    qos.liveliness_lease_duration(overrides.liveliness_lease);
  }

  return qos;
}


rclcpp::QoS QoS::get_publisher_qos(const std::string topic, rclcpp::QoS default_qos) const
{
  const std::string remapped_topic = this->impl_->get_remapped_topic_name(topic);
  auto topic_overrides = impl_->publisher_qos_overrides_map_.find(remapped_topic);
  // If there is no profile override, return the default
  if (impl_->publisher_qos_overrides_map_.end() == topic_overrides) {
    return default_qos;
  }
  return QoSPrivate::apply_overrides(topic_overrides->second, default_qos);
}

rclcpp::QoS QoS::get_subscription_qos(const std::string topic, rclcpp::QoS default_qos) const
{
  const std::string remapped_topic = this->impl_->get_remapped_topic_name(topic);
  auto topic_overrides = impl_->subscription_qos_overrides_map_.find(remapped_topic);
  // If there is no profile override, return the default
  if (impl_->subscription_qos_overrides_map_.end() == topic_overrides) {
    return default_qos;
  }
  return QoSPrivate::apply_overrides(topic_overrides->second, default_qos);
}

// TODO(jacobperron): Use a rclcpp API for getting remapped topic names instead when one exists
std::string QoSPrivate::get_remapped_topic_name(const std::string topic) const
{
  // Get the node options
  const rcl_node_options_t * node_options = this->node_options_.get_rcl_node_options();
  if (nullptr == node_options) {
    throw std::runtime_error("invalid node options in impl_->get_remapped_topic_name()");
  }

  const rcl_arguments_t * global_args = nullptr;
  if (node_options->use_global_arguments) {
    const std::shared_ptr<rcl_context_t> context = this->node_options_.context()->get_rcl_context();
    if (nullptr != context) {
      global_args = &(context->global_arguments);
    }
  }

  const std::string fqn_topic_name = rclcpp::expand_topic_or_service_name(
    topic,
    this->node_name_,
    this->node_namespace_,
    false);  // false = not a service

  std::string result = fqn_topic_name;

  char * remapped_topic_name = nullptr;
  rcl_ret_t ret = rcl_remap_topic_name(
    &(node_options->arguments),
    global_args,
    fqn_topic_name.c_str(),
    this->node_name_.c_str(),
    this->node_namespace_.c_str(),
    node_options->allocator,
    &remapped_topic_name);
  if (RCL_RET_OK != ret) {
    throw std::runtime_error(
            "failed to remap topic '" + topic + "': " + rcl_get_error_string().str);
  }
  if (nullptr != remapped_topic_name) {
    result = remapped_topic_name;
    node_options->allocator.deallocate(remapped_topic_name, node_options->allocator.state);
  }

  // Validate remapped topic name
  // There does not exist an rclcpp API for only validating a topic name,
  // but expand_topic_or_service_name() will do the validation and throw if there's an issue.
  return rclcpp::expand_topic_or_service_name(
    result,
    this->node_name_,
    this->node_namespace_,
    false);  // false = not a service
}

}  // namespace gazebo_ros
