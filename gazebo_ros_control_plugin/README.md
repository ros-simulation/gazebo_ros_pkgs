gazebo_ros_control_plugin
=========================

This package provides tools for using `ros_control`-based controllers in the
Gazebo simulator along with the `ros_control` controller manager.

Gazebo Plugin
--------------

The `<plugin>` tag has the following required elements:
 * `<controlPeriod>`: The period of the controller update (in seconds)

It also has the following optional elements:
 * `<robotSimType>`: The pluginlib name of a custom robot sim interface to be used
 * `<robot_namespace>`: The ROS namespace to be used for this instance of the plugin

The controller plugin can be specified by adding the following to your URDF or SDF:

```xml
<gazebo>
  <plugin name="ros_control" filename="libgazebo_ros_control.so">
    <ns>rr/ros_control</ns>
    <robotSimType>ros_control_gazebo_tests/RobotSimRR</robotSimType>
    <controlPeriod>0.001</controlPeriod>
  </plugin>
</gazebo>
```

Gazebo-ros_control Default Plugin
---------------------------------

This plugin provides a default behavior for interfacing a robot in gazebo with
ros_control-based controllers. It will attempt to get all of the information it
needs out of the SDF or URDF.
