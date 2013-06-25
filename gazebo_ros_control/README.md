Gazebo ros_control Interfaces
=============================

This is a ROS package for integrating the `ros_control` controller architecture
with the [Gazebo](gazebosim.org) simulator. 

Gazebo Plugin
-------------

This package provides a Gazebo plugin which instantiates a ros_control
controller manager and connects it to a Gazebo model.

The `<plugin>` tag has the following required elements:
 * `<controlPeriod>`: The period of the controller update (in seconds)
 * `<joint>`: A tag for each joint to be controlled by the ros_control controller manager

It also has the following optional elements:
 * `<robotNamespace>`: The ROS namespace to be used for this instance of the plugin
 * `<robotSimType>`: The pluginlib name of a custom robot sim interface to be used (see below for more details)

The plugin can be specified by adding the following to your URDF or SDF:

```xml
<gazebo>
  <plugin name="ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>rr/ros_control</robotNamespace>
    <controlPeriod>0.001</controlPeriod>
    <joint>shoulder_joint</joint>
    <joint>elbow_joint</joint>
  </plugin>
</gazebo>
```

Default Gazebo-ros_control Behavior
-----------------------------------

By default, without a `<robotSimType>` tag, `gazebo_ros_control` will attempt
to get all of the information it needs to interface with ros_control-based
controller out of the SDF or URDF. This is sufficient for most cases, and good
for getting started.

The default behavior provides the following ros_control interfaces:
 * `hardware_interface::JointStateInterface`
 * `hardware_interface::EffortJointInterface`
 * `hardware_interface::VelocityJointInterface`

Custom ros_control Simulation Plugins
-------------------------------------

The gazebo_ros_control Gazebo plugin also provides a
[pluginlib](http://www.ros.org/wiki/pluginlib)-based interface to implement
custom interfaces between Gazebo and ros_control for simulating more complex
mechanisms (nonlinear springs, linkages, etc).

These plugins must inherit `gazebo_ros_control::RobotSim` which implements a
simulated ros_control `hardware_interface::RobotHW`. `RobotSim` provides
API-level access to read and command joint properties in the [Gazebo
simulator](gazebosim.org).

The respective `RobotSim` sub-class is specified in an URDF/SDF model and is
loaded when the robot model is loaded. For example, the following XML will load
the default plugin (same behavior as when using no `<robotSimType>` tag:

```xml
<gazebo>
  <plugin name="ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>rr/ros_control</robotNamespace>
    <controlPeriod>0.001</controlPeriod>
    <robotSimType>gazebo_ros_control/DefaultRobotSim</robotSimType>
    <joint>shoulder_joint</joint>
    <joint>elbow_joint</joint>
  </plugin>
</gazebo>
```
