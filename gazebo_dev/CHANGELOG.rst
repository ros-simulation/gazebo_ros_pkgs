^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gazebo_dev
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.9.0 (2025-01-27)
------------------
* Update Gazebo web links (`#1548 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1548>`_)
* Contributors: Alejandro Hernández Cordero

3.8.0 (2024-07-02)
------------------
* Bloom-ignored all the packages except gazebo_msgs on jazzy (`#1534 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1534>`_)
* Contributors: Jose Luis Rivero

3.7.0 (2022-06-13)
------------------

3.6.0 (2022-05-10)
------------------
* Fix test failures (`#1380 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1380>`_)
* Include TBB in gazebo-dev cmake to fix `#1372 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1372>`_ (`#1373 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1373>`_)
* Contributors: Daisuke Nishimatsu, Jose Luis Rivero, Steve Peters

3.5.2 (2021-03-15)
------------------

3.5.1 (2020-11-25)
------------------
* colcon.pkg: build gazebo first in colcon workspace (`#1192 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1192>`_)
  Add a colcon.pkg file to gazebo_dev with gazebo's cmake project
  name "Gazebo" listed as a dependency to support building
  gazebo from source in a colcon workspace.
  * Add colcon.pkg files for other packages
  Copy colcon.pkg to gazebo_ros, gazebo_plugins, and
  gazebo_ros_control so that --merge-install won't be required.
  Signed-off-by: Steve Peters <scpeters@openrobotics.org>
* Contributors: Steve Peters

3.5.0 (2020-06-19)
------------------
* Merge pull request `#1129 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1129>`_ from ros-simulation/e_to_f_june_2020
  Eloquent ➡️ Foxy
* Dashing -> Eloquent
* Gazebo 11 for Foxy (`#1093 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1093>`_)
  * Gazebo 11 for Foxy
* 3.3.5
* Contributors: Jose Luis Rivero, Louise Poubel

3.4.4 (2020-05-08)
------------------

3.4.3 (2020-02-18)
------------------

3.4.2 (2019-11-12)
------------------

3.4.1 (2019-10-10)
------------------

3.4.0 (2019-10-03)
------------------
* Add maintainer (`#985 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/985>`_)
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* Contributors: chapulina

3.3.5 (2020-05-08)
------------------

3.3.4 (2019-09-18)
------------------

3.3.3 (2019-08-23)
------------------
* Add maintainer (`#985 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/985>`_)
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* Contributors: chapulina

3.3.2 (2019-07-31)
------------------

3.3.1 (2019-05-30)
------------------

3.3.0 (2019-05-21)
------------------

3.1.0 (2018-12-10)
------------------

3.0.0 (2018-12-07)
------------------
* Merge pull request `#796 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/796>`_ from ros-simulation/ros2_fix_ci_authors
  [ros2] Fix missing dependencies to run CI and update maintainers
* Update maintainers for ROS2 in gazebo_dev
* Merge pull request `#770 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/770>`_ from ironmig/ros2-gazebo-dev
  ROS2: Port gazebo_dev to ament package
* Port gazebo_dev to ament package
  * currently dependency on gazebo is commented out as it cannot find it
* move gazebo_dev back to root for porting
* Contributors: Jose Luis Rivero, Kevin Allen

2.8.4 (2018-07-06)
------------------

2.8.3 (2018-06-04)
------------------

2.8.2 (2018-05-09)
------------------

2.8.1 (2018-05-05)
------------------
* Replace gazebo7 by gazebo9 in gazebo_dev. Gazebo9 is the official version supported in Melodic
* Contributors: Jose Luis Rivero

2.7.4 (2018-02-12)
------------------

2.7.3 (2017-12-11)
------------------

2.7.2 (2017-05-21)
------------------
* Revert gazebo8 changes in Lunar and back to use gazebo7 (`#583 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/583>`_)
* Contributors: Jose Luis Rivero

2.7.1 (2017-04-28)
------------------
* Use gazeob8 as exec_depend
* Use 2.7.0 as starting version
* Depend on gazebo8 instead of gazebo7
* Add catkin package(s) to provide the default version of Gazebo - take II (kinetic-devel) (`#571 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/571>`_)
  * Added catkin package gazebo_dev which provides the cmake config of the installed Gazebo version
  Conflicts:
  gazebo_plugins/package.xml
  gazebo_ros/package.xml
  gazebo_ros_control/package.xml
  * gazebo_plugins/gazebo_ros: removed dependency SDF from CMakeLists.txt
  The sdformat library is an indirect dependency of Gazebo and does not need to be linked explicitly.
  * gazebo_dev: added execution dependency gazebo
* Contributors: Jose Luis Rivero

* Use gazeob8 as exec_depend
* Use 2.7.0 as starting version
* Depend on gazebo8 instead of gazebo7
* Add catkin package(s) to provide the default version of Gazebo - take II (kinetic-devel) (`#571 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/571>`_)
  * Added catkin package gazebo_dev which provides the cmake config of the installed Gazebo version
  Conflicts:
  gazebo_plugins/package.xml
  gazebo_ros/package.xml
  gazebo_ros_control/package.xml
  * gazebo_plugins/gazebo_ros: removed dependency SDF from CMakeLists.txt
  The sdformat library is an indirect dependency of Gazebo and does not need to be linked explicitly.
  * gazebo_dev: added execution dependency gazebo
* Contributors: Jose Luis Rivero

2.5.12 (2017-04-25)
-------------------

2.5.11 (2017-04-18)
-------------------

2.5.10 (2017-03-03)
-------------------

2.5.9 (2017-02-20)
------------------

2.5.8 (2016-12-06)
------------------

2.5.7 (2016-06-10)
------------------

2.5.6 (2016-04-28)
------------------

2.5.4 (2016-04-27)
------------------

2.5.3 (2016-04-11)
------------------

2.5.2 (2016-02-25)
------------------

2.5.1 (2015-08-16 02:31)
------------------------

2.5.0 (2015-04-30)
------------------

2.4.9 (2015-08-16 01:30)
------------------------

2.4.8 (2015-03-17)
------------------

2.4.7 (2014-12-15)
------------------

2.4.6 (2014-09-01)
------------------

2.4.5 (2014-08-18 21:44)
------------------------

2.4.4 (2014-07-18)
------------------

2.4.3 (2014-05-12)
------------------

2.4.2 (2014-03-27)
------------------

2.4.1 (2013-11-13 18:52)
------------------------

2.4.0 (2013-10-14)
------------------

2.3.6 (2014-08-18 20:22)
------------------------

2.3.5 (2014-03-26)
------------------

2.3.4 (2013-11-13 18:05)
------------------------

2.3.3 (2013-10-10)
------------------

2.3.2 (2013-09-19)
------------------

2.3.1 (2013-08-27)
------------------

2.3.0 (2013-08-12)
------------------

2.2.1 (2013-07-29 18:02)
------------------------

2.2.0 (2013-07-29 13:55)
------------------------

2.1.5 (2013-07-18)
------------------

2.1.4 (2013-07-14)
------------------

2.1.3 (2013-07-13)
------------------

2.1.2 (2013-07-12)
------------------

2.1.1 (2013-07-10)
------------------

2.1.0 (2013-06-27)
------------------

2.0.2 (2013-06-20)
------------------

2.0.1 (2013-06-19)
------------------

2.0.0 (2013-06-18)
------------------
