/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  Copyright (c) 2013, The Johns Hopkins University
 *  Copyright (c) 2014, Fraunhofer IPA
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

/* Author: Dave Coleman, Jonathan Bohren, Felix Messmer
   Desc:   Gazebo plugin for ros_control that allows 'hardware_interfaces' to be plugged in
           using pluginlib. It extends gazebo_ros_control_plugin with harware_interface switching capability.
*/

// GazeboRosControlPlugin
#include <gazebo_ros_control/gazebo_ros_control_plugin.h>

#include <gazebo_ros_control/multi_hw_interface_robot_hw_sim.h>




namespace gazebo_ros_control
{



class GazeboControllerManager : public controller_manager::ControllerManager
{
  MultiHWInterfaceRobotHWSim *robot_hw_sim_;
  
public:

  GazeboControllerManager(RobotHWSim *robot_hw,
                          const ros::NodeHandle& nh=ros::NodeHandle())
                        : controller_manager::ControllerManager(robot_hw, nh), robot_hw_sim_(static_cast<MultiHWInterfaceRobotHWSim*>(robot_hw)) {}
  
  bool notifyHardwareInterface(const std::list<hardware_interface::ControllerInfo> &info_list) 
  {
    //for (std::list<hardware_interface::ControllerInfo>::const_iterator it=info_list.begin(); it != info_list.end(); ++it)
    //{
      //ROS_DEBUG_STREAM("Name: " << it->name << ", Type: " << it->type << ", Hardware-Interface: " << it->hardware_interface);
      //ROS_DEBUG("ResourcesSetLength: %zu", it->resources.size());
    //}
    
    //canSwitchHWInterface
    for (std::list<hardware_interface::ControllerInfo>::const_iterator list_it=info_list.begin(); list_it != info_list.end(); ++list_it)
    {
      for(std::set<std::string>::iterator set_it=list_it->resources.begin(); set_it!=list_it->resources.end(); ++set_it)
      {
        if(!robot_hw_sim_->canSwitchHWInterface(*set_it, list_it->hardware_interface))
          return false;
      }
    }
    
    //doSwitchHWInterface
    for (std::list<hardware_interface::ControllerInfo>::const_iterator list_it=info_list.begin(); list_it != info_list.end(); ++list_it)
    {
      for(std::set<std::string>::iterator set_it=list_it->resources.begin(); set_it!=list_it->resources.end(); ++set_it)
      {
        if(!robot_hw_sim_->doSwitchHWInterface(*set_it, list_it->hardware_interface))
          return false;
      }
    }
    
    ROS_INFO("Done switching HW-Interface! Ready to switch Controllers!");
    return true;
  }
};



class MultiHWInterfaceGazeboRosControlPlugin : public GazeboRosControlPlugin
{
public:

  // Overloaded Gazebo entry point
  virtual void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf);

};


}
