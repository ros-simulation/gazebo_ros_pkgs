ros_control_gazebo
==================

This package provides tools for using `ros_control`-based controllers in the
Gazebo simulator.

It provides a service for loading RobotHW sub-classes which provide 
that

Interfaces
----------

### RobotHWSim
This is like `hardware_interface::RobotHW` except instead of providing access
to the actual robot hardware, it provides an API for the `ros_control` gazebo
plugin.
