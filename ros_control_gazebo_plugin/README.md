ros_control_gazebo_plugin
=========================

This package provides tools for using `ros_control`-based controllers in the
Gazebo simulator along with the `ros_control` controller manager.

Plugins
-------

### ros_control_gazebo_plugin

The `<controller:ros_control_gazebo_plugin>` tag has the following required elements:
 * `<robotSimType>`: The pluginlib name of the robot sim interface to be used
 * `<controlPeriod>`: The period of the controller update (in seconds)

It also has the following optional elements:
 * `<ns>`: The ROS namespace to be used for this instance of the plugin

The controller plugin can be specified like the following:

```xml
<gazebo>
  <controller:ros_control_gazebo_plugin
    name="ros_control" 
    plugin="$(find ros_control_gazebo_plugin)/lib/libros_control_gazebo_plugin.so">
    <robotSimType>ros_control_gazebo_tests/RobotSimRR</robotSimType>
    <controlPeriod>0.001</controlPeriod>
  </controller:ros_control_gazebo_plugin>
</gazebo>
```
