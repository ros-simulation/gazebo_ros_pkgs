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

#ifndef _GAZEBO_ROS_CONTROL__COMBINED_ROBOT_HW_SIM_H_
#define _GAZEBO_ROS_CONTROL__COMBINED_ROBOT_HW_SIM_H_

#include <list>
#include <map>
#include <typeinfo>
#include <hardware_interface/internal/demangle_symbol.h>
#include <hardware_interface/internal/interface_manager.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/robot_hw.h>
#include <gazebo_ros_control/robot_hw_sim.h>
#include <pluginlib/class_loader.hpp>
#include <ros/console.h>
#include <ros/node_handle.h>

namespace gazebo_ros_control
{
/** \brief CombinedRobotHW
 *
 * This class provides a way to combine RobotHW objects.
 *
 *
 *
 */
class CombinedRobotHWSim : public gazebo_ros_control::RobotHWSim
{
protected:
  typedef pluginlib::ClassLoader<gazebo_ros_control::RobotHWSim> class_loader_t;
  class_loader_t class_loader_;
  std::vector<gazebo_ros_control::RobotHWSimSharedPtr> robot_hw_list_;

  /** \brief Filters the start and stop lists so that they only contain the controllers and
   * resources that correspond to the robot_hw interface manager
   */
  void filterControllerList(const std::list<hardware_interface::ControllerInfo>& list,
                            std::list<hardware_interface::ControllerInfo>& filtered_list,
                            gazebo_ros_control::RobotHWSimSharedPtr robot_hw);

public:
  CombinedRobotHWSim();

  virtual ~CombinedRobotHWSim()
  {
  }

  /// \brief Initialize the simulated robot hardware
  ///
  //  Reads in configuration for multiple RobotHWSim classes by reading
  //  the robot_hardware parameter in the robotNamespace set in the sdf.
  //  Example:
  //  <rosparam ns="my_robot">
  //    robot_hardware:
  //      - my_hardware
  //      - custom_hardware
  //    my_hardware:
  //        type: gazebo_ros_control/DefaultRobotHWSim
  //        pid_gains:
  //          my_joint:
  //              p: 0.1
  //              i: 0.001
  //              d: 0
  //    custom_hardware
  //        type: my_package/MyPackageRobotHWSim
  //  <rosparam>
  /// \param robot_namespace  Robot namespace.
  /// \param model_nh  Model node handle.
  /// \param parent_model  Parent model.
  /// \param urdf_model  URDF model.
  /// \param transmissions  Transmissions.
  ///
  /// \return  \c true if the simulated robot hardware is initialized successfully, \c false if not.
  virtual bool initSim(const std::string& robot_namespace, ros::NodeHandle model_nh,
                       gazebo::physics::ModelPtr parent_model, const urdf::Model* const urdf_model,
                       std::vector<transmission_interface::TransmissionInfo> transmissions);

  /// \brief Read state data from the simulated robot hardware
  ///
  /// Read state data, such as joint positions and velocities, from the simulated robot hardware.
  ///
  /// \param time  Simulation time.
  /// \param period  Time since the last simulation step.
  virtual void readSim(ros::Time time, ros::Duration period);

  /// \brief Write commands to the simulated robot hardware
  ///
  /// Write commands, such as joint position and velocity commands, to the simulated robot hardware.
  ///
  /// \param time  Simulation time.
  /// \param period  Time since the last simulation step.
  virtual void writeSim(ros::Time time, ros::Duration period);

  /// \brief Set the emergency stop state
  ///
  /// Set the simulated robot's emergency stop state. The default implementation of this function does nothing.
  ///
  /// \param active  \c true if the emergency stop is active, \c false if not.
  virtual void eStopActive(const bool active)
  {
  }

  /**
  * Check (in non-realtime) if given controllers could be started and stopped from the current state of the RobotHW
  * with regard to necessary hardware interface switches and prepare the switching. Start and stop list are disjoint.
  * This handles the check and preparation, the actual switch is commited in doSwitch()
  */
  virtual bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                             const std::list<hardware_interface::ControllerInfo>& stop_list);

  /**
  * Perform (in realtime) all necessary hardware interface switches in order to start and stop the given controllers.
  * Start and stop list are disjoint. The feasability was checked in prepareSwitch() beforehand.
  */
  virtual void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                        const std::list<hardware_interface::ControllerInfo>& stop_list);
};
}

#endif
