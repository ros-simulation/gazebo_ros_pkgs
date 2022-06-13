^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gazebo_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.7.0 (2022-06-13)
------------------

3.6.0 (2022-05-10)
------------------
* gazebo_ros_wheel_slip: publish wheel slip (`#1331 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1331>`_)
* Contributors: Audrow Nash

3.5.2 (2021-03-15)
------------------

3.5.1 (2020-11-25)
------------------
* [ROS 2] Bridge to republish PerformanceMetrics in ROS 2 (`#1147 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1147>`_)
  Signed-off-by: ahcorde <ahcorde@gmail.com>
* Contributors: Alejandro Hernández Cordero

3.5.0 (2020-06-19)
------------------
* Merge pull request `#1129 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1129>`_ from ros-simulation/e_to_f_june_2020
  Eloquent ➡️ Foxy
* Dashing -> Eloquent
* 3.3.5
* Contributors: Jose Luis Rivero, Louise Poubel

3.4.4 (2020-05-08)
------------------

3.4.3 (2020-02-18)
------------------
* Add maintainer (`#985 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/985>`_)
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* Address reviews on `#868 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/868>`_ (`#972 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/972>`_)
  * [ros2] World plugin to get/set entity state services (`#839 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/839>`_)
  remove status_message
  * [ros2] Port time commands (pause / reset) (`#866 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/866>`_)
  * [ros2] Migration of get/set world, model, joint, link, light properties
  * Trying to pass CI test, try n1.
  * clean up some linter warnings
  * Requested changes in review, unfinished
  * Fix uncrustify
  * Address reviews
  * more tests, joint types
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
  * Revert changes to GetModelProperties message
  Document gazebo_ros_properties header
  * Convert msgs pose to math pose and use it on SetCoG
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* [ros2] Port joint pose trajectory to ROS2 (`#955 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/955>`_)
  * [ros2] Port joint pose trajectory to ROS2
  * Add conversion tests
* Contributors: Louise Poubel, Shivesh Khaitan, chapulina

3.4.2 (2019-11-12)
------------------

3.4.1 (2019-10-10)
------------------

3.4.0 (2019-10-03)
------------------
* Add maintainer (`#985 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/985>`_)
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* Merge pull request `#980 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/980>`_ from shiveshkhaitan/forward_port
  [forward_port] dashing -> ros2
* [ros2] Port joint pose trajectory to ROS2 (`#955 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/955>`_)
* [ros2] Port apply/clear wrench and effort services (`#941 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/941>`_)
  * Change gazebo_ros_effort to gazebo_ros_force_system. Change usage of body to link
* Crystal changes for dashing (`#933 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/933>`_)
  * [ros2] World plugin to get/set entity state services (`#839 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/839>`_)
  * [ros2] Port time commands (pause / reset) (`#866 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/866>`_)
  * relative -> reference
* Contributors: Shivesh Khaitan, chapulina

3.3.5 (2020-05-08)
------------------

3.3.4 (2019-09-18)
------------------

3.3.3 (2019-08-23)
------------------
* Add maintainer (`#985 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/985>`_)
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* Address reviews on `#868 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/868>`_ (`#972 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/972>`_)
  * [ros2] World plugin to get/set entity state services (`#839 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/839>`_)
  remove status_message
  * [ros2] Port time commands (pause / reset) (`#866 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/866>`_)
  * [ros2] Migration of get/set world, model, joint, link, light properties
  * Trying to pass CI test, try n1.
  * clean up some linter warnings
  * Requested changes in review, unfinished
  * Fix uncrustify
  * Address reviews
  * more tests, joint types
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
  * Revert changes to GetModelProperties message
  Document gazebo_ros_properties header
  * Convert msgs pose to math pose and use it on SetCoG
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* [ros2] Port joint pose trajectory to ROS2 (`#955 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/955>`_)
  * [ros2] Port joint pose trajectory to ROS2
  * Add conversion tests
  Minor fixes
* Contributors: Shivesh Khaitan, chapulina

3.3.2 (2019-07-31)
------------------
* Crystal changes for dashing (`#933 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/933>`_)
  * [ros2] World plugin to get/set entity state services (`#839 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/839>`_)
  remove status_message
  * [ros2] Port time commands (pause / reset) (`#866 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/866>`_)
  * relative -> reference
* Contributors: Shivesh Khaitan

3.3.1 (2019-05-30)
------------------

3.3.0 (2019-05-21)
------------------

3.1.0 (2018-12-10)
------------------

3.0.0 (2018-12-07)
------------------
* [ros2] Port spawn/delete methods   (`#808 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/808>`_)
  * First port of ROS2 of factory method. Still a work in progress
  * Install gazebo_ros_factory
  * Changes proposed by uncrustify
  * Make cpplint happy
  * Remove unneded header
  * fix merge
  * remove ported ROS 1 code
  * SpawnEntity service, initialize after world is created, remove XML strip since it's not needed, simplify Is* functions
  * support robot_namespace inside <plugin><ros><namespace>
  * a bit more tweaks and cleanup
  * Use libsdformat to parse the XML, instead of tinyxml, significantly reducing the code
  * uncrustify
  * port delete services
  * linters
  * spawn and delete tests, must check light test
  * fix spawning lights, compile error for non implemented conversions, linters
* Fix name of duration field (`#800 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/800>`_)
* Adding ros1_bridge mapping rules for ODEJointStates message.
  The hiStop and loStop fields in this message were renamed to
  hi_stop and lo_stop to comply with ROS 2 naming rules.
  This prevented the automatic mapping rules in ros1_bridge from
  generating mapping code.
  This change adds a manual mapping to allow the ros1_bridge to
  successfully generate mappings for this message.
* Merge pull request `#770 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/770>`_ from ironmig/ros2-gazebo-dev
  ROS2: Port gazebo_dev to ament package
* Restore url tags to gazebo_msgs package.xml
* Merge pull request `#769 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/769>`_ from ironmig/ros2-devel
  ROS2: boostrap repo and convert gazebo_msgs
* Switch rosidl_default_generators to buildtool_depend
* Converted gazebo_msgs to ros2.
* Contributors: Carl Delsey, Ernesto Corbellini, Jose Luis Rivero, Kevin Allen, dhood

2.8.4 (2018-07-06)
------------------
* Correct documentation on SetModelConfiguration.srv
* Contributors: Kevin Allen

2.8.3 (2018-06-04)
------------------

2.8.2 (2018-05-09)
------------------

2.8.1 (2018-05-05)
------------------

2.7.4 (2018-02-12)
------------------

2.7.3 (2017-12-11)
------------------

2.7.2 (2017-05-21)
------------------

2.7.1 (2017-04-28)
------------------
* Add catkin package(s) to provide the default version of Gazebo - take II (kinetic-devel) (`#571 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/571>`_)
* Contributors: Jose Luis Rivero

2.5.12 (2017-04-25)
-------------------

2.5.11 (2017-04-18)
-------------------
* Changed the spawn model methods to spawn also lights. (`#511 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/511>`_)
* Contributors: Alessandro Ambrosano

2.5.10 (2017-03-03)
-------------------

2.5.9 (2017-02-20)
------------------
* Removed all trailing whitespace
* Contributors: Dave Coleman

2.5.8 (2016-12-06)
------------------

2.5.7 (2016-06-10)
------------------

2.5.6 (2016-04-28)
------------------

2.5.5 (2016-04-27)
------------------
* merge indigo, jade to kinetic-devel
* Update maintainer for Kinetic release
* Contributors: Jose Luis Rivero, Steven Peters

2.5.3 (2016-04-11)
------------------

2.5.2 (2016-02-25)
------------------
* merging from indigo-devel
* 2.4.9
* Generate changelog
* GetModelState modification for jade
* Contributors: John Hsu, Jose Luis Rivero, Markus Bader

2.5.1 (2015-08-16)
------------------

2.5.0 (2015-04-30)
------------------

2.4.10 (2016-02-25)
-------------------

2.4.9 (2015-08-16)
------------------

2.4.8 (2015-03-17)
------------------

2.4.7 (2014-12-15)
------------------
* Update Gazebo/ROS tutorial URL
* Contributors: Jose Luis Rivero

2.4.6 (2014-09-01)
------------------

2.4.5 (2014-08-18)
------------------

2.4.4 (2014-07-18)
------------------
* Fix repo names in package.xml's
* Contributors: Jon Binney

2.4.3 (2014-05-12)
------------------

2.4.2 (2014-03-27)
------------------
* merging from hydro-devel
* bump patch version for indigo-devel to 2.4.1
* merging from indigo-devel after 2.3.4 release
* "2.4.0"
* catkin_generate_changelog
* Contributors: John Hsu

2.4.1 (2013-11-13)
------------------
* rerelease because sdformat became libsdformat, but we also based change on 2.3.4 in hydro-devel.

2.4.0 (2013-10-14)
------------------

2.3.5 (2014-03-26)
------------------

2.3.4 (2013-11-13)
------------------

2.3.3 (2013-10-10)
------------------

2.3.2 (2013-09-19)
------------------

2.3.1 (2013-08-27)
------------------

2.3.0 (2013-08-12)
------------------

2.2.1 (2013-07-29)
------------------

2.2.0 (2013-07-29)
------------------

2.1.5 (2013-07-18)
------------------

2.1.4 (2013-07-14)
------------------

2.1.3 (2013-07-13)
------------------

2.1.2 (2013-07-12)
------------------
* Cleaned up CMakeLists.txt for all gazebo_ros_pkgs
* 2.1.1

2.1.1 (2013-07-10 19:11)
------------------------

2.1.0 (2013-06-27)
------------------

2.0.2 (2013-06-20)
------------------

2.0.1 (2013-06-19)
------------------
* Incremented version to 2.0.1

2.0.0 (2013-06-18)
------------------
* Changed version to 2.0.0 based on gazebo_simulator being 1.0.0
* Updated package.xml files for ros.org documentation purposes
* Imported from bitbucket.org
