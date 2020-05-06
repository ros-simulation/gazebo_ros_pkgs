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

#ifndef GAZEBO_ROS__QOS_HPP_
#define GAZEBO_ROS__QOS_HPP_

#include <sdf/sdf.hh>

#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>

#include <map>
#include <stdexcept>
#include <string>

namespace gazebo_ros
{

/// Exception thrown when there is an invalid QoS configuration.
class InvalidQoSException : public std::runtime_error
{
public:
  explicit InvalidQoSException(const std::string & msg)
  : std::runtime_error(msg)
  {}
};

/// Quality of service profile for ROS node entities
/**
 * \class QoS qos.hpp <gazebo_ros/qos.hpp>
 * Encapsulates QoS profiles for a group of ROS publishers and subscriptions.
 * Supports parsing SDF <qos> tags.
 *
 * Note, this implementation groups QoS profiles by entity type (publisher or subscription) and
 * topic name.
 * For example, two publishers using the same topic will have the same QoS returned by this class.
 * \include gazebo_ros/qos.hpp
 */
class QoS
{
public:
  QoS() = default;

  /// Constructor with SDF
  /**
   * \details Create a new QoS profile for a collection of node entities from SDF.
   * SDF should have the form:
   * \code{.xml}
   * <qos>
   *  <!-- Zero or more topic tags with topic names to configure -->
   *  <topic name="my/topic/name">
   *    <!-- Optional tag for setting the QoS profile for subscriptions -->
   *    <subscription>
   *      <!-- Optional tags for overriding default settings (see below)-->
   *    </subscription>
   *    <!-- Optional tag for setting the QoS profile for publishers -->
   *    <publisher>
   *      <!-- Optionally tags for overriding default settings (see below) -->
   *    </publisher>
   *  </topic>
   * </qos>
   * \endcode
   *
   * The available QoS settings that can be overridden are outlined below.
   * For more information on each option and their possible values, see
   * http://design.ros2.org/articles/qos.html and
   * http://design.ros2.org/articles/qos_deadline_liveliness_lifespan.html.
   *
   * The <reliability> element must contain one of the following: "reliable", "best_effort",
   * or "system".
   *
   * The <durability> element must contain one of the following: "volatile", "transient_local",
   * or "system".
   *
   * The <history> element must contain one of the following: "keep_last", "keep_all", or "system".
   * If "keep_last" is used, a "depth" attribute must be present and set to a positive integer.
   *
   * The <deadline> element must contain a positive integer representing a duration in
   * milliseconds.
   *
   * The <lifespan> element must contain a positive integer representing a duration in
   * milliseconds.
   *
   * The <liveliness> element must contain one of the following: "automatic", "manual_by_topic",
   * or "system".
   *
   * The <liveliness_lease_duration> element must contain a positive integer representing a
   * duration in milliseconds.
   *
   * For any QoS setting that is not specified in the SDF, the default QoS object passed
   * to the getter methods is used.
   *
   * The topic name should be the name expected after any remapping occurs.
   * For example, if a <remapping> tag remaps a topic 'foo' to the name 'bar', then the <topic>
   * tag's name attribute should have the value 'bar' to override any QoS settings:
   *
   * \code{.xml}
   * <remapping>foo:=bar</remapping>
   * <qos>
   *   <topic name='bar'>
   *   </topic>
   * </qos>
   * \endcode
   *
   * \param[in] _sdf An SDF element in the style documented above
   * \param[in] node_name The name of the node associated with these QoS settings.
   * \param[in] node_namespace The namespace of the node associated with these QoS settings.
   * \param[in] options Node options that were also passed to the node associated with these QoS
   *   settings.
   *   This contains the necessary information extracting the remappings.
   * \throws gazebo_ros::InvalidQoSException if there is an invalid QoS value or a topic element
   *   is missing a "name" attribute.
   */
  QoS(
    sdf::ElementPtr _sdf,
    const std::string node_name,
    const std::string node_namespace,
    const rclcpp::NodeOptions & options);

  /// Get the QoS for a publisher
  /*
   * \param[in] topic: Get QoS for publishers on this topic name.
   * \param[in] default_qos: The default quality of service used for settings that have not been
   *   overridden.
   */
  rclcpp::QoS get_publisher_qos(
    const std::string topic, rclcpp::QoS default_qos = rclcpp::QoS(10)) const;

  /// Get the QoS for a subscription
  /*
   * \param[in] topic: Get QoS for subscriptions on this topic name.
   * \param[in] default_qos: The default quality of service used for settings that have not been
   *   overridden.
   */
  rclcpp::QoS get_subscription_qos(
    const std::string topic, rclcpp::QoS default_qos = rclcpp::QoS(10)) const;

private:
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

}  // namespace gazebo_ros
#endif  // GAZEBO_ROS__QOS_HPP_
