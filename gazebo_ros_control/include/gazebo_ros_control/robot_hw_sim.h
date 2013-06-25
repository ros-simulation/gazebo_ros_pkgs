#ifndef __ROS_CONTROL_GAZEBO_ROBOT_HW_SIM_H
#define __ROS_CONTROL_GAZEBO_ROBOT_HW_SIM_H

#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>

namespace gazebo_ros_control {

  class RobotHWSim : public hardware_interface::RobotHW 
  {
  public:

    virtual ~RobotHWSim() { }

    virtual bool initSim(
        ros::NodeHandle model_nh, 
        gazebo::physics::ModelPtr parent_model,
        std::vector<std::string> joint_names) 
    {
      return true; 
    };

    virtual void readSim(ros::Time time, ros::Duration period) = 0;

    virtual void writeSim(ros::Time time, ros::Duration period) = 0;

  };

}

#endif // ifndef __ROS_CONTROL_GAZEBO_ROBOT_HW_SIM_H
