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

#include <rclcpp/qos.hpp>
#include <rmw/types.h>

#include <chrono>
#include <unordered_map>
#include <sstream>
#include <string>

namespace gazebo_ros
{

/// Helper function for creating an rclcpp::QoS from SDF and a default QoS as a base.
QoS::QoSOverrides QoS::get_qos_overrides_from_sdf(sdf::ElementPtr _sdf)
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
    {"manual_by_node", RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_NODE},
    {"manual_by_topic", RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC},
  };

  QoS::QoSOverrides qos_overrides;

  // Parse 'reliability' QoS
  if (_sdf->HasElement("reliability")) {
    auto reliability = _sdf->GetElement("reliability")->Get<std::string>();
    auto find_result = reliability_map.find(reliability);
    if (find_result != reliability_map.end()) {
      qos_overrides.reliability = find_result->second;
    } else {
      std::stringstream oss;
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
      std::stringstream oss;
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
          std::stringstream oss;
          oss << "'" << find_result->first << "' used without providing a depth";
          throw InvalidQoSException(oss.str());
        }
        qos_overrides.depth = history_element->Get<size_t>("depth");
      }
    } else {
      std::stringstream oss;
      oss << "invalid setting for history '" << history << "'";
      throw InvalidQoSException(oss.str());
    }
  }

  // Parse 'deadline' QoS
  if (_sdf->HasElement("deadline")) {
    auto deadline = _sdf->GetElement("deadline")->Get<size_t>();
    qos_overrides.deadline = std::chrono::milliseconds(deadline);
  }

  // Parse 'lifespan' QoS
  if (_sdf->HasElement("lifespan")) {
    auto lifespan = _sdf->GetElement("lifespan")->Get<size_t>();
    qos_overrides.lifespan = std::chrono::milliseconds(lifespan);
  }

  // Parse 'liveliness' QoS
  if (_sdf->HasElement("liveliness")) {
    auto liveliness = _sdf->GetElement("liveliness")->Get<std::string>();
    auto find_result = liveliness_map.find(liveliness);
    if (find_result != liveliness_map.end()) {
      qos_overrides.liveliness = find_result->second;
    } else {
      std::stringstream oss;
      oss << "invalid setting for liveliness '" << liveliness << "'";
      throw InvalidQoSException(oss.str());
    }
  }

  // Parse 'liveliness_lease_duration' QoS
  if (_sdf->HasElement("liveliness_lease_duration")) {
    auto lease = _sdf->GetElement("liveliness_lease_duration")->Get<size_t>();
    qos_overrides.liveliness_lease = std::chrono::milliseconds(lease);
  }

  return qos_overrides;
}

QoS::QoS(sdf::ElementPtr _sdf)
{
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

    // For each topic, get the publisher QoS overrides
    if (topic_sdf->HasElement("publisher")) {
      publisher_qos_overrides_map_.emplace(
        topic_name, QoS::get_qos_overrides_from_sdf(topic_sdf->GetElement("publisher")));
    }

    // For each topic, get the subscription QoS overrides
    if (topic_sdf->HasElement("subscription")) {
      subscription_qos_overrides_map_.emplace(
        topic_name, QoS::get_qos_overrides_from_sdf(topic_sdf->GetElement("subscription")));
    }

    topic_sdf = topic_sdf->GetNextElement("topic");
  }
}

rclcpp::QoS QoS::apply_overrides(const QoS::QoSOverrides & overrides, const rclcpp::QoS default_qos)
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
  auto topic_overrides = publisher_qos_overrides_map_.find(topic);
  // If there is no profile override, return the default
  if (publisher_qos_overrides_map_.end() == topic_overrides) {
    return default_qos;
  }
  return QoS::apply_overrides(topic_overrides->second, default_qos);
}

rclcpp::QoS QoS::get_subscription_qos(const std::string topic, rclcpp::QoS default_qos) const
{
  auto topic_overrides = subscription_qos_overrides_map_.find(topic);
  // If there is no profile override, return the default
  if (subscription_qos_overrides_map_.end() == topic_overrides) {
    return default_qos;
  }
  return QoS::apply_overrides(topic_overrides->second, default_qos);
}

}  // namespace gazebo_ros
