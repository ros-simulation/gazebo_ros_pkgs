#ifndef __ROS_CONTROL_GAZEBO_ROBOT_SIM_H
#define __ROS_CONTROL_GAZEBO_ROBOT_SIM_H

#include <gazebo/physics/physics.hh>

#include <hardware_interface/robot_hw.h>

namespace ros_control_gazebo {

  class RobotSim : public hardware_interface::RobotHW 
  {
  public:

    virtual ~RobotSim() { }

    virtual bool initSim(
        ros::NodeHandle nh, 
        gazebo::physics::ModelPtr model) 
    {
      return true; 
    };

    virtual void readSim(ros::Time time, ros::Duration period) = 0;

    virtual void writeSim(ros::Time time, ros::Duration period) = 0;

  };

}

#endif // ifndef __ROS_CONTROL_GAZEBO_ROBOT_SIM_H
