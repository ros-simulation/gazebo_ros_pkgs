/*
 * Copyright 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef _GAZEBO_ROS_PLUGINS_MOTORTHERMALPLUGIN_HH_
#define _GAZEBO_ROS_PLUGINS_MOTORTHERMALPLUGIN_HH_

#include <sdf/sdf.hh>

// Gazebo
#include <gazebo/plugins/MotorThermalPlugin.hh>

// ROS
#include <ros/ros.h>

namespace gazebo
{
  /// \brief ROS implementation of the MotorThermal plugin
  class GazeboRosMotorThermal : public MotorThermalPlugin
  {
    /// \brief Constructor
    public: GazeboRosMotorThermal();

    /// \brief Destructor
    public: virtual ~GazeboRosMotorThermal();

    /// \brief Load the plugin.
    /// \param[in] _model Pointer to the Model
    /// \param[in] _sdf Pointer to the SDF element of the plugin.
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Queue to handle callbacks.
    private: void QueueThread();

    /// \brief On Gazebo torque message
    private: void OnTorque(ConstTorquePtr _msg);

    /// \brief On Gazebo case temperature message
    private: void OnCase(ConstTemperaturePtr _msg);

    /// \brief On Gazebo coil temperature message
    private: void OnCoil(ConstTemperaturePtr _msg);

    /// \brief Gazebo node pointer
    private: gazebo::transport::NodePtr node;

    /// \brief Gazebo torque subscriber
    private: gazebo::transport::SubscriberPtr torqueSub;

    /// \brief Gazebo case subscriber
    private: gazebo::transport::SubscriberPtr caseSub;

    /// \brief Gazebo coil subscriber
    private: gazebo::transport::SubscriberPtr coilSub;

    /// \brief ros node handle
    private: ros::NodeHandle *rosnode;

    /// \brief Publishes torque values
    private: ros::Publisher torquePub;

    /// \brief Publishes coil temperature values
    private: ros::Publisher coilPub;

    /// \brief Publishes coil temperature values
    private: ros::Publisher casePub;

    /// \brief Custom Callback Queue
    private: ros::CallbackQueue queue;

    // \brief Custom Callback Queue thread
    private: boost::thread callbackQueueThread;
  };
}

#endif
