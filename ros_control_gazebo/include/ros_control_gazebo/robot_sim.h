#ifndef __ROS_CONTROL_GAZEBO_ROBOT_SIM_H
#define __ROS_CONTROL_GAZEBO_ROBOT_SIM_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace ros_control_gazebo {

  class RobotSim {
  public:
    virtual void read_state(gazebo::physics::Model const *model) = 0;
    virtual void write_state(gazebo::physics::Model *model) = 0;
  };

}

#endif // ifndef __ROS_CONTROL_GAZEBO_ROBOT_SIM_H
