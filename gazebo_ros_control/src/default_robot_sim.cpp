/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
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

/* Author: Dave Coleman, Johnathan Bohren
   Desc:   Hardware Interface for any simulated robot in Gazebo
*/

#ifndef __GAZEBO_ROS_CONTROL_PLUGIN_DEFAULT_ROBOT_SIM_H_
#define __GAZEBO_ROS_CONTROL_PLUGIN_DEFAULT_ROBOT_SIM_H_

// ros_control
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

// ROS
#include <ros/ros.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

// ros_control + gazebo
#include <gazebo_ros_control/robot_sim.h>

namespace gazebo_ros_control
{

  class DefaultRobotSim : public gazebo_ros_control::RobotSim
  {
  public:

    bool initSim(
        ros::NodeHandle model_nh, 
        gazebo::physics::ModelPtr parent_model,
        std::vector<std::string> joint_names) 
    {
      // Store the joint names
      joint_names_ = joint_names;

      // Resize vectors to our DOF
      n_dof_ = joint_names_.size();
      joint_position_.resize(n_dof_);
      joint_velocity_.resize(n_dof_);
      joint_effort_.resize(n_dof_);
      joint_effort_command_.resize(n_dof_);
      joint_velocity_command_.resize(n_dof_);

      // Initialize values
      for(unsigned int j=0; j < n_dof_; j++)
      {
        joint_position_[j] = 1.0;
        joint_velocity_[j] = 0.0;
        joint_effort_[j] = 1.0;  // N/m for continuous joints
        joint_effort_command_[j] = 0.0;
        joint_velocity_command_[j] = 0.0;

        // Register joints
        js_interface_.registerHandle(hardware_interface::JointStateHandle(
                joint_names_[j], &joint_position_[j],&joint_velocity_[j], &joint_effort_[j]));          
        ej_interface_.registerHandle(hardware_interface::JointHandle(
                js_interface_.getHandle(joint_names_[j]),&joint_effort_command_[j]));          
        vj_interface_.registerHandle(hardware_interface::JointHandle(
                js_interface_.getHandle(joint_names_[j]),&joint_velocity_command_[j]));          
      }

      // Register interfaces
      registerInterface(&js_interface_);
      registerInterface(&ej_interface_);
      registerInterface(&vj_interface_);
      // Get the gazebo joints that correspond to the robot joints
      for(unsigned int j=0; j < n_dof_; j++)
      {
        ROS_DEBUG_STREAM("Getting pointer to gazebo joint: "<<joint_names_[j]);
        gazebo::physics::JointPtr joint = parent_model->GetJoint(joint_names_[j]);
        if (joint)
        {
          sim_joints_.push_back(joint);
        }
        else
        {
          ROS_ERROR_STREAM("This robot has a joint named \""<<joint_names_[j]<<"\" which is not in the gazebo model.");
          return false;
        }
      }

      return true;
    }

    void readSim(ros::Time time, ros::Duration period)
    {
      for(unsigned int j=0; j < n_dof_; j++)
      {
        // Gazebo has an interesting API...
        joint_position_[j] += angles::shortest_angular_distance(joint_position_[j], 
                                                                sim_joints_[j]->GetAngle(0).Radian());
        joint_velocity_[j] = sim_joints_[j]->GetVelocity(0);
        joint_effort_[j] = sim_joints_[j]->GetForce((unsigned int)(0));
      }
    }

    void writeSim(ros::Time time, ros::Duration period)
    {
      for(unsigned int j=0; j < n_dof_; j++)
      {
        // Gazebo has an interesting API...
        sim_joints_[j]->SetForce(0,joint_effort_command_[j]);
      }
    }

  private:
    unsigned int n_dof_;

    hardware_interface::JointStateInterface    js_interface_;
    hardware_interface::EffortJointInterface   ej_interface_;
    hardware_interface::VelocityJointInterface vj_interface_;

    std::vector<std::string> joint_names_;
    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;
    std::vector<double> joint_effort_command_;
    std::vector<double> joint_velocity_command_;

    std::vector<gazebo::physics::JointPtr> sim_joints_;
  };

  typedef boost::shared_ptr<DefaultRobotSim> DefaultRobotSimPtr;

}

PLUGINLIB_DECLARE_CLASS(gazebo_ros_control,
                        DefaultRobotSim,
                        gazebo_ros_control::DefaultRobotSim,
                        gazebo_ros_control::RobotSim)

#endif // #ifndef __GAZEBO_ROS_CONTROL_PLUGIN_DEFAULT_ROBOT_SIM_H_
