/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Open Source Robotics Foundation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <algorithm>
#include <boost/foreach.hpp>
#include <pluginlib/class_list_macros.h>
#include "gazebo_ros_control/combined_robot_hw_sim.h"

namespace gazebo_ros_control
{
CombinedRobotHWSim::CombinedRobotHWSim() : class_loader_("gazebo_ros_control", "gazebo_ros_control::RobotHWSim")
{
}

bool CombinedRobotHWSim::initSim(const std::string& robot_namespace, ros::NodeHandle model_nh,
                                 gazebo::physics::ModelPtr parent_model, const urdf::Model* const urdf_model,
                                 std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  // Get 
  std::vector<std::string> robots;
  std::string param_name = "robot_hardware";
  if (!model_nh.getParam(param_name, robots))
  {
    ROS_ERROR("Param %s not in %s", param_name.c_str(), model_nh.getNamespace().c_str());
    return false;
  }

  // Load each RobotHWSim
  BOOST_FOREACH (std::string& name, robots)
  {
    ROS_DEBUG("Will load robot HW '%s'", name.c_str());

    // Create local node handle in namespace of this simulated hardware
    ros::NodeHandle c_nh;
    try
    {
      c_nh = ros::NodeHandle(model_nh, name);
    }
    catch (std::exception const& e)
    {
      ROS_ERROR("Exception thrown while constructing nodehandle for robot HW with name '%s':\n%s", name.c_str(),
                e.what());
      return false;
    }

    // Get class type of this hardware from param
    gazebo_ros_control::RobotHWSimSharedPtr robot_hw;
    std::string type;
    if (!c_nh.getParam("type", type))
    {
      ROS_ERROR("Could not load robot HW '%s' because the type was not specified. Did you load the robot HW "
                "configuration on the parameter server (namespace: '%s')?",
                name.c_str(), c_nh.getNamespace().c_str());
      return false;
    }

    // Load in plugin for this type
    try
    {
      robot_hw = class_loader_.createInstance(type);
    }
    catch (const pluginlib::PluginlibException& ex)
    {
      ROS_ERROR("Could not load class %s: %s", type.c_str(), ex.what());
      return false;
    }

    // Initializes the robot HW
    ROS_DEBUG("Initializing robot HW '%s'", name.c_str());
    try
    {
      robot_hw->initSim(robot_namespace, c_nh, parent_model, urdf_model, transmissions);
    }
    catch (std::exception& e)
    {
      ROS_ERROR("Exception thrown while initializing robot HW %s.\n%s", name.c_str(), e.what());
      return false;
    }

    // Register hardware and add to vector
    robot_hw_list_.push_back(robot_hw);
    this->registerInterfaceManager(robot_hw.get());
    ROS_DEBUG("Successfully load robot HW '%s'", name.c_str());
  }

  return true;
}

bool CombinedRobotHWSim::prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                                       const std::list<hardware_interface::ControllerInfo>& stop_list)
{
  // Prepare switch for each contained RobotHWSim, passing each a filtered list of controllers
  BOOST_FOREACH (gazebo_ros_control::RobotHWSimSharedPtr& hw, robot_hw_list_)
  {
    std::list<hardware_interface::ControllerInfo> filtered_start_list;
    std::list<hardware_interface::ControllerInfo> filtered_stop_list;

    filterControllerList(start_list, filtered_start_list, hw);
    filterControllerList(stop_list, filtered_stop_list, hw);

    if (!hw->prepareSwitch(filtered_start_list, filtered_stop_list))
      return false;
  }
  return true;
}

void CombinedRobotHWSim::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                                  const std::list<hardware_interface::ControllerInfo>& stop_list)
{
  // Prepare switch for each contained RobotHWSim, passing each a filtered list of controllers
  BOOST_FOREACH (gazebo_ros_control::RobotHWSimSharedPtr& hw, robot_hw_list_)
  {
    std::list<hardware_interface::ControllerInfo> filtered_start_list;
    std::list<hardware_interface::ControllerInfo> filtered_stop_list;

    // Generate a filtered version of start_list and stop_list for each RobotHW before calling doSwitch
    filterControllerList(start_list, filtered_start_list, hw);
    filterControllerList(stop_list, filtered_stop_list, hw);

    hw->doSwitch(filtered_start_list, filtered_stop_list);
  }
}

void CombinedRobotHWSim::readSim(ros::Time time, ros::Duration period)
{
  // Call the write method of each RobotHWSim
  BOOST_FOREACH (gazebo_ros_control::RobotHWSimSharedPtr& hw, robot_hw_list_)
  {
    hw->readSim(time, period);
  }
}

void CombinedRobotHWSim::writeSim(ros::Time time, ros::Duration period)
{
  // Call the read method of each RobotHWSim
  BOOST_FOREACH (gazebo_ros_control::RobotHWSimSharedPtr& hw, robot_hw_list_)
  {
    hw->writeSim(time, period);
  }
}

void CombinedRobotHWSim::filterControllerList(const std::list<hardware_interface::ControllerInfo>& list,
                                              std::list<hardware_interface::ControllerInfo>& filtered_list,
                                              gazebo_ros_control::RobotHWSimSharedPtr robot_hw)
{
  filtered_list.clear();
  for (std::list<hardware_interface::ControllerInfo>::const_iterator it = list.begin(); it != list.end(); ++it)
  {
    hardware_interface::ControllerInfo filtered_controller;
    filtered_controller.name = it->name;
    filtered_controller.type = it->type;

    if (it->claimed_resources.empty())
    {
      filtered_list.push_back(filtered_controller);
      continue;
    }
    for (std::vector<hardware_interface::InterfaceResources>::const_iterator res_it = it->claimed_resources.begin();
         res_it != it->claimed_resources.end(); ++res_it)
    {
      hardware_interface::InterfaceResources filtered_iface_resources;
      filtered_iface_resources.hardware_interface = res_it->hardware_interface;
      std::vector<std::string> r_hw_ifaces = robot_hw->getNames();

      std::vector<std::string>::iterator if_name =
          std::find(r_hw_ifaces.begin(), r_hw_ifaces.end(), filtered_iface_resources.hardware_interface);
      if (if_name == r_hw_ifaces.end())  // this hardware_interface is not registered in r_hw, so we filter it out
      {
        continue;
      }

      std::vector<std::string> r_hw_iface_resources =
          robot_hw->getInterfaceResources(filtered_iface_resources.hardware_interface);
      std::set<std::string> filtered_resources;
      for (std::set<std::string>::const_iterator ctrl_res = res_it->resources.begin();
           ctrl_res != res_it->resources.end(); ++ctrl_res)
      {
        std::vector<std::string>::iterator res_name =
            std::find(r_hw_iface_resources.begin(), r_hw_iface_resources.end(), *ctrl_res);
        if (res_name != r_hw_iface_resources.end())
        {
          filtered_resources.insert(*ctrl_res);
        }
      }
      filtered_iface_resources.resources = filtered_resources;
      filtered_controller.claimed_resources.push_back(filtered_iface_resources);
    }
    filtered_list.push_back(filtered_controller);
  }
}
}
PLUGINLIB_EXPORT_CLASS(gazebo_ros_control::CombinedRobotHWSim, gazebo_ros_control::RobotHWSim);
