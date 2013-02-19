ros_control_gazebo_tests
========================

This package provides test and examples using `ros_control`-based
controllers in the Gazebo simulator along with the `ros_control` controller
manager.

Tests
-----

### Effort-Controlled R-R Manipulator

To bring up the simulator with a simple R-R manipulator and effort
controllers, run:

```bash
roslaunch ros_control_gazebo_tests rr_manipulator.launch
rosrun rosservice call /rr/ros_control/controller_manager/load_controller "name: 'joint1_position_controller'"
rosrun rosservice call /rr/ros_control/controller_manager/load_controller "name: 'joint2_position_controller'"
rosservice call /rr/ros_control/controller_manager/switch_controller "{start_controllers: ['joint1_position_controller','joint2_position_controller'], stop_controllers: [], strictness: 0}" 
rostopic pub /rr/ros_control/joint2_position_controller/command std_msgs/Float64 "data: 1.0"
```
