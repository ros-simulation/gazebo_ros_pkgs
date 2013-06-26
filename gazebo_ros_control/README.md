# Gazebo ros_control Interfaces

This is a ROS package for integrating the `ros_control` controller architecture
with the [Gazebo](http://gazebosim.org/) simulator. 

## Gazebo Plugin

This package provides a Gazebo plugin which instantiates a ros_control
controller manager and connects it to a Gazebo model.

The `<plugin>` tag has the following required elements:
 * `<controlPeriod>`: The period of the controller update (in seconds)

It also has the following optional elements:
 * `<robotNamespace>`: The ROS namespace to be used for this instance of the plugin, defaults to robot name in URDF/SDF
 * `<robotParam>`: The location of the robot_descrption (URDF XML) on the parameter server, defaults to '/robot_description'
 * `<robotSimType>`: The pluginlib name of a custom robot sim interface to be used (see below for more details), defaults to 'DefaultRobotHWSim'

The plugin is used by adding the following to your URDF or SDF:

```xml
<gazebo>
  <plugin name="ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/MYROBOT</robotNamespace>
    <controlPeriod>0.001</controlPeriod>
  </plugin>
</gazebo>
```

To specify which joints are to be actuated by ros_control, you must add transmission tags to your URDF for every joint:

```xml
  <transmission name="tran1" type="transmission_interface/SimpleTransmission">
    <joint name="joint1"/>
    <actuator name="motor1" type="EffortJointInterface" />
    <mechanicalReduction>1</mechanicalReduction>
  </transmission>
```

For the purposes of gazeb_ros_control in its current implementation, the only important information is:
 * `<joint name="">` - the name must correspond to a joint else where in your URDF
 * `<actuator type="">` - the type of actuator tells the gazebo_ros_plugin what hardware interface to load

The rest of the names and elements are ignored for the time being.

## Default gazebo_ros_control Behavior

By default, without a `<robotSimType>` tag, `gazebo_ros_control` will attempt
to get all of the information it needs to interface with ros_control-based
controller out of the SDF or URDF. This is sufficient for most cases, and good
for getting started.

The default behavior provides the following ros_control interfaces:
 * `hardware_interface::JointStateInterface`
 * `hardware_interface::EffortJointInterface`
 * `hardware_interface::VelocityJointInterface`

## Custom ros_control Simulation Plugins

The gazebo_ros_control Gazebo plugin also provides a
[pluginlib](http://www.ros.org/wiki/pluginlib)-based interface to implement
custom interfaces between Gazebo and ros_control for simulating more complex
mechanisms (nonlinear springs, linkages, etc).

These plugins must inherit `gazebo_ros_control::RobotHWSim` which implements a
simulated ros_control `hardware_interface::RobotHW`. `RobotHWSim` provides
API-level access to read and command joint properties in the [Gazebo
simulator](gazebosim.org).

The respective `RobotHWSim` sub-class is specified in an URDF/SDF model and is
loaded when the robot model is loaded. For example, the following XML will load
the default plugin (same behavior as when using no `<robotSimType>` tag:

```xml
<gazebo>
  <plugin name="ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/MYROBOT</robotNamespace>
    <controlPeriod>0.001</controlPeriod>
    <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
  </plugin>
</gazebo>
```

## Future Direction

A todo list:
 - Implement tranmissions
 - In the default plugin support more hardware interfaces