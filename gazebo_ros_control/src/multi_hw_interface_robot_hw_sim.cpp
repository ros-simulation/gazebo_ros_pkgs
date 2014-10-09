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

/* Author: Dave Coleman, Johnathan Bohren, Felix Messmer
   Desc:   Hardware Interface for any simulated robot in Gazebo supporting multiple hardware_interfaces
*/


// gazebo_ros_control
#include <gazebo_ros_control/multi_hw_interface_robot_hw_sim.h>


namespace gazebo_ros_control
{

bool MultiHWInterfaceRobotHWSim::initSim(
  const std::string& robot_namespace,
  ros::NodeHandle model_nh,
  gazebo::physics::ModelPtr parent_model,
  const urdf::Model *const urdf_model,
  std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  // getJointLimits() searches joint_limit_nh for joint limit parameters. The format of each
  // parameter's name is "joint_limits/<joint name>". An example is "joint_limits/axle_joint".
  const ros::NodeHandle joint_limit_nh(model_nh, robot_namespace);

  // Resize vectors to our DOF
  n_dof_ = transmissions.size();
  joint_names_.resize(n_dof_);
  joint_types_.resize(n_dof_);
  joint_lower_limits_.resize(n_dof_);
  joint_upper_limits_.resize(n_dof_);
  joint_effort_limits_.resize(n_dof_);
  joint_control_methods_.resize(n_dof_);
  pid_controllers_.resize(n_dof_);
  map_hwinterface_to_joints_.clear();
  map_hwinterface_to_controlmethod_.clear();
  map_controlmethod_to_pidcontrollers_.clear();
  joint_position_.resize(n_dof_);
  joint_velocity_.resize(n_dof_);
  joint_effort_.resize(n_dof_);
  joint_effort_command_.resize(n_dof_);
  joint_position_command_.resize(n_dof_);
  joint_velocity_command_.resize(n_dof_);
  
  std::vector<control_toolbox::Pid> pid_controllers_pos;
  pid_controllers_pos.resize(n_dof_);
  map_controlmethod_to_pidcontrollers_.insert( std::pair< ControlMethod, std::vector<control_toolbox::Pid> >(POSITION_PID, pid_controllers_pos) );
  std::vector<control_toolbox::Pid> pid_controllers_vel;
  pid_controllers_vel.resize(n_dof_);
  map_controlmethod_to_pidcontrollers_.insert( std::pair< ControlMethod, std::vector<control_toolbox::Pid> >(VELOCITY_PID, pid_controllers_vel) );

  // Initialize values
  for(unsigned int j=0; j < n_dof_; j++)
  {
    // Check that this transmission has one joint
    if(transmissions[j].joints_.size() == 0)
    {
      ROS_WARN_STREAM_NAMED("default_robot_hw_sim","Transmission " << transmissions[j].name_
        << " has no associated joints.");
      continue;
    }
    else if(transmissions[j].joints_.size() > 1)
    {
      ROS_WARN_STREAM_NAMED("default_robot_hw_sim","Transmission " << transmissions[j].name_
        << " has more than one joint. Currently the default robot hardware simulation "
        << " interface only supports one.");
      continue;
    }

    std::vector<std::string> joint_interfaces = transmissions[j].joints_[0].hardware_interfaces_;
    if (joint_interfaces.empty() &&
        !(transmissions[j].actuators_.empty()) &&
        !(transmissions[j].actuators_[0].hardware_interfaces_.empty()))
    {
      // TODO: Deprecate HW interface specification in actuators in ROS J
      joint_interfaces = transmissions[j].actuators_[0].hardware_interfaces_;
      ROS_WARN_STREAM_NAMED("default_robot_hw_sim", "The <hardware_interface> element of tranmission " <<
        transmissions[j].name_ << " should be nested inside the <joint> element, not <actuator>. " <<
        "The transmission will be properly loaded, but please update " <<
        "your robot model to remain compatible with future versions of the plugin.");
    }
    if (joint_interfaces.empty())
    {
      ROS_WARN_STREAM_NAMED("default_robot_hw_sim", "Joint " << transmissions[j].joints_[0].name_ <<
        " of transmission " << transmissions[j].name_ << " does not specify any hardware interface. " <<
        "Not adding it to the robot hardware simulation.");
      continue;
    }
    else if (joint_interfaces.size() > 1)
    {
      ROS_DEBUG_STREAM_NAMED("default_robot_hw_sim", "Joint " << transmissions[j].joints_[0].name_ <<
        " of transmission " << transmissions[j].name_ << " specifies multiple hardware interfaces. " <<
        "This feature is now available.");
    }

    // Add data from transmission
    joint_names_[j] = transmissions[j].joints_[0].name_;
    joint_position_[j] = 1.0;
    joint_velocity_[j] = 0.0;
    joint_effort_[j] = 1.0;  // N/m for continuous joints
    joint_effort_command_[j] = 0.0;
    joint_position_command_[j] = 0.0;
    joint_velocity_command_[j] = 0.0;


    // Create joint state interface for all joints
    js_interface_.registerHandle(hardware_interface::JointStateHandle(
        joint_names_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));
    
    // Decide what kind of command interface this actuator/joint has
    hardware_interface::JointHandle joint_handle;
      
    // Parse all HW-Interfaces available for each joint and store information
    for(unsigned int i=0; i<joint_interfaces.size(); i++)
    {
      // Debug
      ROS_DEBUG_STREAM_NAMED("default_robot_hw_sim","Loading joint '" << joint_names_[j]
        << "' of type '" << joint_interfaces[i] << "'");
      
      // Add hardware interface and joint to map of map_hwinterface_to_joints_
      // ToDo: hardcoded namespace 'hardware_interface'?
      std::string hw_interface_type = "hardware_interface::"+joint_interfaces[i];
      if(map_hwinterface_to_joints_.find(hw_interface_type)!=map_hwinterface_to_joints_.end())
      {
        ROS_DEBUG_STREAM_NAMED("default_robot_hw_sim", "HW-Interface " << hw_interface_type << " already registered. Adding joint " << joint_names_[j] << " to list.");
        std::map< std::string, std::set<std::string> >::iterator it;
        it=map_hwinterface_to_joints_.find(hw_interface_type);
        it->second.insert(joint_names_[j]);
      }
      else
      {
        ROS_DEBUG_STREAM_NAMED("default_robot_hw_sim", "New HW-Interface registered " << hw_interface_type << ". Adding joint " << joint_names_[j] << " to list.");
        std::set<std::string> supporting_joints;
        supporting_joints.insert(joint_names_[j]);
        map_hwinterface_to_joints_.insert( std::pair< std::string, std::set<std::string> >(hw_interface_type, supporting_joints) );
      }
      
      if(joint_interfaces[i] == "EffortJointInterface")
      {
        // Create effort joint interface
        if(i==0){ joint_control_methods_[j] = EFFORT; } //use first entry for startup
        map_hwinterface_to_controlmethod_.insert( std::pair<std::string, ControlMethod>(hw_interface_type, EFFORT) );
        
        joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                      &joint_effort_command_[j]);
        ej_interface_.registerHandle(joint_handle);
        
        registerJointLimits(joint_names_[j], joint_handle, EFFORT,
                        joint_limit_nh, urdf_model,
                        &joint_types_[j], &joint_lower_limits_[j], &joint_upper_limits_[j],
                        &joint_effort_limits_[j]);
      }
      else if(joint_interfaces[i] == "PositionJointInterface")
      {
        ControlMethod control_method = POSITION;
        control_toolbox::Pid pid_controller;
        
        // Initialize the PID controller. If no PID gain values are found, use joint->SetAngle() or
        // joint->SetVelocity() to control the joint.
        const ros::NodeHandle nh(model_nh, robot_namespace + "/gazebo_ros_control/pid_gains/position/" +
                                 joint_names_[j]);
        if (pid_controller.init(nh, true))
        {
          control_method = POSITION_PID;
          map_controlmethod_to_pidcontrollers_.find(control_method)->second[j]=pid_controller;
        }
          
        // Create position joint interface
        if(i==0){ joint_control_methods_[j] = control_method; } //use first entry for startup
        map_hwinterface_to_controlmethod_.insert( std::pair<std::string, ControlMethod>(hw_interface_type, control_method) );
          
        joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                      &joint_position_command_[j]);
        pj_interface_.registerHandle(joint_handle);
        
        registerJointLimits(joint_names_[j], joint_handle, control_method,
                        joint_limit_nh, urdf_model,
                        &joint_types_[j], &joint_lower_limits_[j], &joint_upper_limits_[j],
                        &joint_effort_limits_[j]);
      }
      else if(joint_interfaces[i] == "VelocityJointInterface")
      {
        ControlMethod control_method = VELOCITY;
        control_toolbox::Pid pid_controller;
        
        // Initialize the PID controller. If no PID gain values are found, use joint->SetAngle() or
        // joint->SetVelocity() to control the joint.
        const ros::NodeHandle nh(model_nh, robot_namespace + "/gazebo_ros_control/pid_gains/velocity/" +
                                 joint_names_[j]);
        if (pid_controller.init(nh, true))
        {
          control_method = VELOCITY_PID;
          map_controlmethod_to_pidcontrollers_.find(control_method)->second[j]=pid_controller;
        }
          
        // Create velocity joint interface
        if(i==0){ joint_control_methods_[j] = control_method; } //use first entry for startup
        map_hwinterface_to_controlmethod_.insert( std::pair<std::string, ControlMethod>(hw_interface_type, control_method) );
         
        joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                      &joint_velocity_command_[j]);
        vj_interface_.registerHandle(joint_handle);
        
        registerJointLimits(joint_names_[j], joint_handle, control_method,
                        joint_limit_nh, urdf_model,
                        &joint_types_[j], &joint_lower_limits_[j], &joint_upper_limits_[j],
                        &joint_effort_limits_[j]);
      }
      else
      {
        ROS_FATAL_STREAM_NAMED("default_robot_hw_sim","No matching hardware interface found for '"
          << joint_interfaces[i] );
        return false;
      }
    }

    // Get the gazebo joint that corresponds to the robot joint.
    //ROS_DEBUG_STREAM_NAMED("default_robot_hw_sim", "Getting pointer to gazebo joint: "
    //  << joint_names_[j]);
    gazebo::physics::JointPtr joint = parent_model->GetJoint(joint_names_[j]);
    if (!joint)
    {
      ROS_ERROR_STREAM_NAMED("default_robot_hw_sim", "This robot has a joint named \"" << joint_names_[j]
        << "\" which is not in the gazebo model.");
      return false;
    }
    sim_joints_.push_back(joint);
      
    
    // ToDo: Can a joint (gazebo::physics::JointPtr) be used for EFFORT if joint->SetMaxForce has been called before?
    if (joint_control_methods_[j] == VELOCITY || joint_control_methods_[j] == POSITION)
    {
      // joint->SetMaxForce() must be called if joint->SetAngle() or joint->SetVelocity() are
      // going to be called. joint->SetMaxForce() must *not* be called if joint->SetForce() is
      // going to be called.
      joint->SetMaxForce(0, joint_effort_limits_[j]);
    }
  }

  // Register interfaces
  registerInterface(&js_interface_);
  registerInterface(&ej_interface_);
  registerInterface(&pj_interface_);
  registerInterface(&vj_interface_);

  return true;
}

  
bool MultiHWInterfaceRobotHWSim::canSwitchHWInterface(const std::string &joint_name, const std::string &hwinterface_name)
{
  ROS_DEBUG_STREAM_NAMED("canSwitchHWInterface", "Joint " << joint_name << " requests HW-Interface of type " << hwinterface_name);
  std::map< std::string, std::set<std::string> >::iterator it = map_hwinterface_to_joints_.find(hwinterface_name);
  if(it->second.find(joint_name)!=it->second.end()) { return true; }
  
  ROS_ERROR_STREAM_NAMED("default_robot_hw_sim", "Joint " << joint_name << " does not provide a HW-Interface of type " << hwinterface_name);
  return false;
}

bool MultiHWInterfaceRobotHWSim::doSwitchHWInterface(const std::string &joint_name, const std::string &hwinterface_name)
{
  ROS_DEBUG_STREAM_NAMED("doSwitchHWInterface", "Joint " << joint_name << " requests HW-Interface of type " << hwinterface_name);
  for(unsigned int i=0; i<joint_names_.size(); i++)
  {
    if(joint_names_[i] == joint_name)
    {
      if(map_hwinterface_to_controlmethod_.find(hwinterface_name)!=map_hwinterface_to_controlmethod_.end())
      {
        ControlMethod current_control_method = map_hwinterface_to_controlmethod_.find(hwinterface_name)->second;
        joint_control_methods_[i] = current_control_method;
        if(current_control_method == POSITION_PID || current_control_method == VELOCITY_PID)
        {
          pid_controllers_ = map_controlmethod_to_pidcontrollers_.find(current_control_method)->second;
        }
        
        ROS_DEBUG_STREAM_NAMED("default_robot_hw_sim", "Joint " << joint_name << " now uses HW-Interface type: " << hwinterface_name);
        return true;
      }
    }
  }
  
  ROS_ERROR_STREAM_NAMED("default_robot_hw_sim", "An error occured while trying to switch HW-Interface for Joint " << joint_name << " (HW-Interface type: " << hwinterface_name << ")");
  return false;
}

}

PLUGINLIB_EXPORT_CLASS(gazebo_ros_control::MultiHWInterfaceRobotHWSim, gazebo_ros_control::RobotHWSim)
