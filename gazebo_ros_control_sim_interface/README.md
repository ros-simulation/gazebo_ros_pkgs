ros_control_gazebo
==================

This package provides tools for using `ros_control`-based controllers in the
Gazebo simulator along with the `ros_control` controller manager.

Interfaces
----------

### ros_control_gazebo::RobotSim : public hardware_interface::RobotHW

This class is a subclass of `hardware_interface::RobotHW` which provides an 
API for the `ros_control_gazebo_plugin` to read and command joint properties 
in the [Gazebo simulator](gazebosim.org).

Sub-classes of `RobotSim` are built into dynamically-loaded
[pluginlib](http://www.ros.org/wiki/pluginlib) plugins. The
respective `RobotSim` sub-class is specified in an URDF/SDF model and 
is loaded when the robot model is loaded.
