#ifndef __ROS_CONTROL_GAZEBO_ROBOT_SIM_H
#define __ROS_CONTROL_GAZEBO_ROBOT_SIM_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <hardware_interface/robot_hw.h>

namespace ros_control_gazebo {

  class RobotSim : public hardware_interface::RobotHW {
  public:
    virtual bool init_sim(const gazebo::physics::ModelPtr model) { return true; };
    virtual void read_sim(const gazebo::physics::ModelPtr model) = 0;
    virtual void write_sim(gazebo::physics::ModelPtr model) = 0;

  };

}

#endif // ifndef __ROS_CONTROL_GAZEBO_ROBOT_SIM_H
