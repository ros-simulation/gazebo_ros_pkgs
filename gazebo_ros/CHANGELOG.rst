^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gazebo_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.9.0 (2025-01-27)
------------------
* Add Gazebo Classic EOL notice (`#1562 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1562>`_)
* Fix: Fixed uninitialized warning (`#1560 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1560>`_)
* Update Gazebo web links (`#1548 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1548>`_)
* Add support to parse the parameters file directly (`#1546 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1546>`_)
* Contributors: Addisu Z. Taddese, Alejandro Hernández Cordero, Sai Kishor Kothakota, Janosch Machowinski

3.8.0 (2024-07-02)
------------------
* Bloom-ignored all the packages except gazebo_msgs on jazzy (`#1534 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1534>`_)
* Changed Throttler to use simTime instead of realTime (`#1325 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1325>`_)
  Changed Throttler for the clock publisher to use simTime instead of realTime. This fixes gazebo_ros_pkgs`#1324 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1324>`_ and should allow ROS applications to continue to cooperate with Gazebo when the simulation is paused and is being stepped.
* gzserver.launch.py: fix _arg_command (`#1502 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1502>`_)
  Set default value of join_with paramete to '='.
* gazebo_ros: Fix error message formatting in spawn_entity.py (`#1479 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1479>`_)
  It appears that the gazebo_namespace was missing from this format
  string.
* gzserver.launch.py: add initial_sim_time argument
  Expose gzserver CLI parameter added in
  `gazebosim/gazebo-classic#3294 <https://github.com/gazebosim/gazebo-classic/issues/3294>`_.
* gzserver.launch.py: refactor _arg_command
  This refactors the _arg_command helper function to
  remove the condition parameter (which is now unused
  since the _conditional_command helper was added) and
  always passes the LaunchConfiguration along with the
  --command with a "join_with" separator. This simplifies
  the `cmd` array definition.
* gzserver.launch.py: _conditional_command helper
  Add a helper function to replace the functionality
  of _arg_command with the condition argument.
* gazebo_ros_factory: more reliable model spawning (`#1453 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1453>`_)
  Use the World::InsertModelString API instead of a
  gazebo transport topic to spawn models. This is more
  reliable, since gazebo transport may lose messages.
* gazebo_ros: Add inline keyword to template definitions (`#1367 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1367>`_)
  * gazebo_ros: Add inline keyword to template definitions
  * Uncrustify
  Co-authored-by: Gerard Heshusius <gHeshusius@lely.com>
  Co-authored-by: Jacob Perron <jacob@openrobotics.org>
* Set gzserver to log output to "both" in launch (`#1435 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1435>`_)
  Log output to console and process log, otherwise no log messages
  from gzserver or plugins are saved to disk.
  Co-authored-by: Mark B. Allan <Mark.B.Allan@nasa.gov>
* Fix deprecation warnings (`#1429 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1429>`_)
  * Fix deprecation warning in spawn_entity.py
  * Fix cv_bridge deprecation warning
  Co-authored-by: Steve Peters <scpeters@openrobotics.org>
* Fix console errors and warnings when launching gzserver (`#1414 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1414>`_)
  * Remove spaces from gzserver and gzclient launch commands
  They are not necessary and can cause console warnings/errors for some options.
  * Fix console error when launching gzserver
  If one of the substitutions in the gzserver command results in an empty string, then we see the following warning:
  [Err] [Server.cc:472] Could not open file[]
  [Wrn] [Server.cc:381] Falling back on worlds/empty.world
  This is because gzserver is parsing any empty string as the world argument.
  To fix the issue, we explicitly launch an empty world if no launch argument for the
  world is given.
  * Add test to check output
  We expect some output on stderr when passing the 'verbose' argument.
* Contributors: Gerard Heshusius, Jacob Perron, Jose Luis Rivero, Mark B. Allan, Scott K Logan, Steve Peters, shonigmann

3.7.0 (2022-06-13)
------------------
* Fix gzserver launch file breaking when no ros args provided (`#1395 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1395>`_)
* Multiple nodes with same name issue (`#1394 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1394>`_)
* Expose launch parameter for ``params_file`` (`#1391 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1391>`_)
* Contributors: Brian Chen, Deepanshu Bansal, Jacob Perron

3.6.0 (2022-05-10)
------------------
* Fixes gazebo shutdown error when using nested launch files (`#1376 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1376>`_)
* Fix test failures (`#1380 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1380>`_)
* Fix spawn_entity_demo: use executable (`#1349 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1349>`_)
* Add runtime warning when user sets use_sim_time parameter
* Add ROS parameter to toggle performance metrics in gazebo_ros (`#1295 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1295>`_)
* Deprecate -spawn_service_timeout option (`#1238 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1238>`_)
* Contributors: Aditya Pande, Daisuke Nishimatsu, Felix Exner, Jacob Perron

3.5.2 (2021-03-15)
------------------
* Remove slash from gazebo_ros scripts Python package name (`#1251 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1251>`_)
* Fix line length in gazebo_ros/test/CMakeLists.txt (`#1249 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1249>`_)
* gazebo_ros: use lxml in spawn_entity.py (`#1221 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1221>`_)
  The python xml.etree.ElementTree library does not handle xml namespaces well,
  replacing namespaces of prefixed elements with ns0, ns1, etc. This switches
  to using lxml instead, which has the same syntax and is already used by other
  ros2 packages.  * Add a test
* Fix tests for cyclonedds (`#1228 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1228>`_)
  The default RMW implementation changed recently and some tests are now
  failing. This fixes the tests.  * Use KeepLast(1) with transient_local in
  tests There are some QoS incompatibilities in some tests that use
  SystemDefaultsQoS, so this changes them to use KeepLast(1) with
  transient_local instead. This fixes some of the test failures but not all.
  * test_sim_time: allow more startup messages
  * Fix QoS and initialization of joint state pub test
* Fix executor to avoid random exceptions when shutting down (`#1212 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1212>`_)
  * Fix executor to avoid random exceptions when shutting down
  * Add link to related issue in rclcpp
* ros2: Only subscribe to /gazebo/performance_metrics when necessary (`#1205 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1205>`_)
  We are currently subscribing to the /gazebo/performance_metrics topic
  even if there are no subscribers to the ROS topic forwarding this data.
  This changes gazebo_ros_init to only subscribe to the gazebo topic
  if there are any subscribers to the corresponding ROS topic.
  While advertiser callbacks are used in ROS 1 but are not yet in ROS2,
  here we use polling in the GazeboRosInitPrivate::PublishSimTime
  callback to check for subscribers since it is called for each Gazebo
  time step.
  This also helps workaround the deadlock documented in `#1175 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1175>`_ and
  `osrf/gazebo#2902 <https://github.com/osrf/gazebo/issues/2902>`_.
  This also adds a macro to reduce duplication of the version checking
  logic.
* Contributors: Ivan Santiago Paunovic, Michel Hidalgo, Steve Peters

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
* Fixed Parameterized testing on Rolling (`#1184 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1184>`_)
  Signed-off-by: ahcorde <ahcorde@gmail.com>
* [ROS 2] Bridge to republish PerformanceMetrics in ROS 2 (`#1147 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1147>`_)
  Signed-off-by: ahcorde <ahcorde@gmail.com>
* [Windows] Add missing visibility control. (`#1150 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1150>`_)
* [ros2] Enable the force system on launch files (`#1035 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1035>`_)
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* make compile wo/ warnings on osx (`#1149 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1149>`_)
  Signed-off-by: Karsten Knese <Karsten1987@users.noreply.github.com>
* Added lockstep argument to gzserver (`#1146 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1146>`_)
  Signed-off-by: ahcorde <ahcorde@gmail.com>
* Contributors: Alejandro Hernández Cordero, Karsten Knese, Louise Poubel, Sean Yen, Steve Peters

3.5.0 (2020-06-19)
------------------
* Merge pull request `#1130 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1130>`_ from ros-simulation/foxy_tests
  Fix all Foxy tests
* Merge pull request `#1129 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1129>`_ from ros-simulation/e_to_f_june_2020
  Eloquent ➡️ Foxy
* Dashing -> Eloquent
* [ROS 2] Use "" as default in spawn_entity.py instead of self.get_namespace(). (`#1117 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1117>`_)
* Fix flake8 failures (`#1110 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1110>`_)
* Add gazebo_ros::QoS class (`#1091 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1091>`_)
  Contains logic for parsing <qos> SDF elements and creating rclcpp::QoS objects for ROS publishers and subscriptions.
  Co-authored-by: Ivan Santiago Paunovic <ivanpauno@ekumenlabs.com>
* [eloquent] Fix Windows build. (`#1077 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1077>`_)
* 3.3.5
* [forward port to Foxy] Add node required parameter to launch (`#1074 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1074>`_)  (`#1086 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1086>`_)
* Use configurable timeout in other wait for service calls (`#1073 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1073>`_)
  * Use configurable timeout in other wait for service calls
  Follow-up to `#1072 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1072>`_
* Increase spawn entity wait for service timeout (`#1072 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1072>`_)
  If we have a reaonsable complex launch file and lots of DDS discovery traffic, sometimes five seconds isn't enough.
  This change makes the timeout configurable and changes the default timeout to thirty seconds.
* Uncrustify (`#1060 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1060>`_)
  Style changes to conform to the new default setting introduced in https://github.com/ament/ament_lint/pull/210.
  Arguments that do not fit on one line must start on a new line.
* [ros2] make transient local reliable (`#1033 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1033>`_)
  Co-Authored-By: chapulina <louise@openrobotics.org>
* Contributors: Alejandro Hernández Cordero, Ivan Santiago Paunovic, Jacob Perron, Jose Luis Rivero, Karsten Knese, Louise Poubel, Mabel Zhang, Sean Yen

3.4.4 (2020-05-08)
------------------
* wait for service with variable timeout (`#1090 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1090>`_)
* Contributors: Karsten Knese, Louise Poubel

3.4.3 (2020-02-18)
------------------
* [backport][ros2] make transient local reliable (`#1033 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1033>`_) (`#1036 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1036>`_)
  * [ros2] make transient local reliable (`#1033 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1033>`_)
  * make transient local reliable
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * fix master
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * add launch test
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * make it actual latched
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * alpha sort
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * add launch_test dependency
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * more dependencies
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * remove debug print
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * is_initialized -> ok
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * Update gazebo_ros/test/entity_spawner.test.py
  Co-Authored-By: chapulina <louise@openrobotics.org>
  * use erase-remove idiom
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * use ReadyToTest()
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  Co-authored-by: chapulina <louise@openrobotics.org>
  * Set timeout and call gzserver directly
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
  Co-authored-by: chapulina <louise@openrobotics.org>
* fix pathsep for windows (`#1028 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1028>`_)
* Remove ROS-specific arguments before passing to argparse (`#994 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/994>`_)
  This resolves argparse errors when trying to launch the spawn_entity script as a ROS node.
  For example, a launch file like the following wouldn't work without this change:
  <launch>
  <arg name="model_urdf" default="$(find-pkg-share mymodels)/urdf/ball.urdf" />
  <node
  pkg="gazebo_ros"
  exec="spawn_entity.py"
  name="spawner"
  args="-entity foo -file /path/to/my/model/foo.urdf" />
  </launch>
  Signed-off-by: Jacob Perron <jacob@openrobotics.org>
* [ros2] Remove ported / deprecated (`#989 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/989>`_)
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* linter :sweat_smile:
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* [ros2] Uncommenting bond option on spawn_entity (wait Ctrl+C then remove entity) (`#986 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/986>`_)
  * Uncommenting bond option on spawn_entity (wait Ctrl+C then remove entity)
  Instead of waiting for a shutdown callback to be created in rclpy,
  we can use the try/except to get the SIGINT signal, then delete the entity.
  * Message formatting
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* [ros2] Conditional launch includes (`#979 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/979>`_)
  * [ros2] Conditional launch includes
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
  * remove unused import
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
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
* [ros2] Spawn <plugin> without <ros> (`#983 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/983>`_)
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* [ros2] Port spawn model to ROS2 (`#948 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/948>`_)
  * [ros2] Port spawn model to ROS2
  * Delete .ros1_unported files
  * Fixes and add demo
  Change spawn_model to spawn_entity
  * Rename demo launch and add checks for service
  * Fix reading xml file from param and model states
  * remove diplicate
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
  * Use gazebo launch file
  * Change topic behaviour
* [ros2] Port gazebo launch scripts to ROS2 (`#962 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/962>`_)
  * [ros2] Port gazebo launch scripts to ROS2
  * Add gdb and valgrind option
  * Use shell command for extra gazebo args
* [ros2] Port joint pose trajectory to ROS2 (`#955 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/955>`_)
  * [ros2] Port joint pose trajectory to ROS2
  * Add conversion tests
  Minor fixes
* Merge pull request `#977 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/977>`_ from ros-simulation/backport
  [backport] ros2 -> dashing
* [ros2] Port Link states to ROS2 (`#969 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/969>`_)
  * [ros2] Port model states to ROS2
  * [ros2] Port link states to ROS2
  * Change usage of body -> link
  * Remove link_states from .ros1_unported
* set gazebo library dirs (`#963 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/963>`_)
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
* [ros2] Port gazebo_ros_path plugin to ROS2 (`#925 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/925>`_)
  * [ros2] Port gazebo_ros_path plugin
  * Minor fixes
  * Change plugin launch file to python script
  * Fix for flake8 test
* [ros2] Port bumper sensor to ROS2 (`#943 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/943>`_)
  * [ros2] Port bumper sensor to ROS2
  * Add author name
  * Minor fixes and add contact msg conversion
  * Remove unused header includes
* [ros2] Fix tests on Dashing (`#953 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/953>`_)
  * [ros2] Fix camera triggered test on Dashing
  backport remove noe fix and re-enable distortion tests
  * improve robustness of joint state pub test
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* [ros2] Port model states to ROS2 (`#968 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/968>`_)
  * [ros2] Port model states to ROS2
  * remove unported code
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* [ros2] Port hand of god to ROS2 (`#957 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/957>`_)
  * [ros2] Port hand of god to ROS2
  * Minor fixes
* [ros2] Port planar move to ROS2 (`#958 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/958>`_)
  * [ros2] Port planar move to ROS2
  * Add test for pose conversion
* use c_str() (`#950 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/950>`_) (`#954 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/954>`_)
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
* Contributors: Jacob Perron, Jonathan Noyola, Karsten Knese, Louise Poubel, Shivesh Khaitan, alexfneves, chapulina

3.4.2 (2019-11-12)
------------------
* Merge branch 'ros2' into eloquent
* Remove ROS-specific arguments before passing to argparse (`#994 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/994>`_) (`#1013 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1013>`_)
  This resolves argparse errors when trying to launch the spawn_entity script as a ROS node.
  For example, a launch file like the following wouldn't work without this change:
  <launch>
  <arg name="model_urdf" default="$(find-pkg-share mymodels)/urdf/ball.urdf" />
  <node
  pkg="gazebo_ros"
  exec="spawn_entity.py"
  name="spawner"
  args="-entity foo -file /path/to/my/model/foo.urdf" />
  </launch>
  Signed-off-by: Jacob Perron <jacob@openrobotics.org>
* [ros2] Add remapping tag (`#1011 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1011>`_)
  * add --ros-args and a remapping element for ros arguments
  Signed-off-by: Mikael Arguedas <mikael.arguedas@gmail.com>
  * keep backward compatibility
  Signed-off-by: Mikael Arguedas <mikael.arguedas@gmail.com>
  * update docs and world file accordingly
  Signed-off-by: Mikael Arguedas <mikael.arguedas@gmail.com>
  * remap all the things :fist_raised:
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* catch const ref to fix -Wcatch-value warnings (`#1012 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1012>`_)
  Signed-off-by: Mikael Arguedas <mikael.arguedas@gmail.com>
* Contributors: Jacob Perron, Louise Poubel, Mikael Arguedas

3.4.1 (2019-10-10)
------------------

3.4.0 (2019-10-03)
------------------
* [ros2] Uncommenting bond option on spawn_entity (wait Ctrl+C then remove entity) (`#986 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/986>`_)
  * Uncommenting bond option on spawn_entity (wait Ctrl+C then remove entity)
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* [ros2] Conditional launch includes (`#979 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/979>`_)
  * remove unused import
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* Add maintainer (`#985 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/985>`_)
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* [ros2] Spawn <plugin> without <ros> (`#983 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/983>`_)
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* Merge pull request `#980 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/980>`_ from shiveshkhaitan/forward_port
  [forward_port] dashing -> ros2
* [ros2] Port spawn model to ROS2 (`#948 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/948>`_)
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* [ros2] Port gazebo launch scripts to ROS2 (`#962 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/962>`_)
* [ros2] Port joint pose trajectory to ROS2 (`#955 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/955>`_)
* [ros2] Port Link states to ROS2 (`#969 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/969>`_)
* [ros2] Fix tests on Dashing (`#953 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/953>`_)
  * [ros2] Fix camera triggered test on Dashing
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* [ros2] Port model states to ROS2 (`#968 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/968>`_)
  * [ros2] Port model states to ROS2
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* [ros2] Port hand of god to ROS2 (`#957 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/957>`_)
* [ros2] Port planar move to ROS2 (`#958 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/958>`_)
* [ros2] Port apply/clear wrench and effort services (`#941 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/941>`_)
* [ros2] Port gazebo_ros_path plugin to ROS2 (`#925 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/925>`_)
* set gazebo library dirs (`#963 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/963>`_)
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
* [ros2] Port bumper sensor to ROS2 (`#943 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/943>`_)
* Fix for multiple video plugins (`#898 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/898>`_) (`#937 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/937>`_)
* use c_str() (`#950 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/950>`_)
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
* Crystal changes for dashing (`#933 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/933>`_)
  * [ros2] World plugin to get/set entity state services (`#839 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/839>`_)
  * [ros2] Port time commands (pause / reset) (`#866 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/866>`_)
  * relative -> reference
* Contributors: Karsten Knese, Louise Poubel, Shivesh Khaitan, alexfneves, chapulina

3.3.5 (2020-05-08)
------------------
* fix pathsep for windows (`#1028 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1028>`_)
* Contributors: Jonathan Noyola

3.3.4 (2019-09-18)
------------------
* Remove ROS-specific arguments before passing to argparse (`#994 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/994>`_)
  This resolves argparse errors when trying to launch the spawn_entity script as a ROS node.
  For example, a launch file like the following wouldn't work without this change:
  <launch>
  <arg name="model_urdf" default="$(find-pkg-share mymodels)/urdf/ball.urdf" />
  <node
  pkg="gazebo_ros"
  exec="spawn_entity.py"
  name="spawner"
  args="-entity foo -file /path/to/my/model/foo.urdf" />
  </launch>
  Signed-off-by: Jacob Perron <jacob@openrobotics.org>
* [ros2] Remove ported / deprecated (`#989 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/989>`_)
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* linter :sweat_smile:
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* [ros2] Uncommenting bond option on spawn_entity (wait Ctrl+C then remove entity) (`#986 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/986>`_)
  * Uncommenting bond option on spawn_entity (wait Ctrl+C then remove entity)
  Instead of waiting for a shutdown callback to be created in rclpy,
  we can use the try/except to get the SIGINT signal, then delete the entity.
  * Message formatting
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* Contributors: Jacob Perron, Louise Poubel, alexfneves, chapulina

3.3.3 (2019-08-23)
------------------
* [ros2] Conditional launch includes (`#979 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/979>`_)
  * [ros2] Conditional launch includes
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
  * remove unused import
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
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
* [ros2] Spawn <plugin> without <ros> (`#983 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/983>`_)
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* [ros2] Port spawn model to ROS2 (`#948 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/948>`_)
  * [ros2] Port spawn model to ROS2
  * Delete .ros1_unported files
  * Fixes and add demo
  Change spawn_model to spawn_entity
  * Rename demo launch and add checks for service
  * Fix reading xml file from param and model states
  * remove diplicate
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
  * Use gazebo launch file
  * Change topic behaviour
* [ros2] Port gazebo launch scripts to ROS2 (`#962 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/962>`_)
  * [ros2] Port gazebo launch scripts to ROS2
  * Add gdb and valgrind option
  * Use shell command for extra gazebo args
* [ros2] Port joint pose trajectory to ROS2 (`#955 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/955>`_)
  * [ros2] Port joint pose trajectory to ROS2
  * Add conversion tests
  Minor fixes
* Merge pull request `#977 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/977>`_ from ros-simulation/backport
  [backport] ros2 -> dashing
* [ros2] Port Link states to ROS2 (`#969 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/969>`_)
  * [ros2] Port model states to ROS2
  * [ros2] Port link states to ROS2
  * Change usage of body -> link
  * Remove link_states from .ros1_unported
* set gazebo library dirs (`#963 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/963>`_)
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
* [ros2] Port gazebo_ros_path plugin to ROS2 (`#925 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/925>`_)
  * [ros2] Port gazebo_ros_path plugin
  * Minor fixes
  * Change plugin launch file to python script
  * Fix for flake8 test
* [ros2] Port bumper sensor to ROS2 (`#943 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/943>`_)
  * [ros2] Port bumper sensor to ROS2
  * Add author name
  * Minor fixes and add contact msg conversion
  * Remove unused header includes
* [ros2] Fix tests on Dashing (`#953 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/953>`_)
  * [ros2] Fix camera triggered test on Dashing
  backport remove noe fix and re-enable distortion tests
  * improve robustness of joint state pub test
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* [ros2] Port model states to ROS2 (`#968 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/968>`_)
  * [ros2] Port model states to ROS2
  * remove unported code
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* [ros2] Port hand of god to ROS2 (`#957 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/957>`_)
  * [ros2] Port hand of god to ROS2
  * Minor fixes
* Contributors: Karsten Knese, Shivesh Khaitan, chapulina

3.3.2 (2019-07-31)
------------------
* [ros2] Port planar move to ROS2 (`#958 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/958>`_)
  * [ros2] Port planar move to ROS2
  * Add test for pose conversion
* use c_str() (`#950 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/950>`_) (`#954 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/954>`_)
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
* Crystal changes for dashing (`#933 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/933>`_)
  * [ros2] World plugin to get/set entity state services (`#839 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/839>`_)
  remove status_message
  * [ros2] Port time commands (pause / reset) (`#866 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/866>`_)
  * relative -> reference
* Contributors: Shivesh Khaitan, chapulina

3.3.1 (2019-05-30)
------------------
* Declare parameters and use overrides (`#931 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/931>`_)
  * Declare parameters and use overrides
  * PR feedback
  * fix linking error
* Contributors: chapulina

3.3.0 (2019-05-21)
------------------
* use latest dashing api (`#926 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/926>`_)
  * [gazebo_ros] use qos
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * [gazebo_ros] avoid unused warning
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * [gazebo_plugins] use qos
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * allow_undeclared_parameters
  * fix tests
  * forward port pull request `#901 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/901>`_
* Fix build to account for new NodeOptions interface. (`#887 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/887>`_)
* Fix Windows conflicting macros and missing usleep (`#885 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/885>`_)
  * Fix conflicting Windows macros and missing usleep
  * fix spacing
  * fix spacing again
  * remove lint
* Call rclcpp::init() only from gazebo_ros_init (`#859 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/859>`_)
* [ros] Revert sim time test (`#853 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/853>`_)
* Contributors: Carl Delsey, Jonathan Noyola, Karsten Knese, Tamaki Nishino, chapulina

3.1.0 (2018-12-10)
------------------
* [ros2] Camera and triggered camera (`#827 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/827>`_)
  * move gazebo_ros_camera and some functionality from gazebo_ros_camera_utils, needs master branch of image_transport and message_filters, not functional, but compiling
  * port PutCameraData, needs common_interfaces PR `#58 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/58>`_
  * move camera worlds, fix compilation, image can be seen on RViz
  * Port camera test: simplify world, use ServerFixture for better control and not to need launch - test is hanging on exit, not sure why
  * fix test hanging on exit
  * port camera16bit test and fix world copying on make
  * Start porting camera distortion tests: must port cam_info, 2nd test failing
  * sortout camera_name and frame_name
  * Port gazebo_ros_camera_triggered as part of gazebo_ros_camera, with test
  * Use camera_info_manager from branch ci_manager_port_louise, enable barrel distortion test - passes but segfaults at teardown, could be a problem with having 2 plugins side-by-side.
  * linters and comment out crashing test
  * Demo worlds, doxygen, more node tests
  * Use image_transport remapping
  * adapt to new image_transport pointer API
  * new API
* fix rclcpp::init when there are no arguments
* [ros2] Adapt sim time test to work around rclcpp issue
* Contributors: Louise Poubel, chapulina

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
* [ros2] Port diff_drive plugin to ros2 (`#806 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/806>`_)
  * copy gazebo_ros_diff_drive files from unported
  * Fix copy and paste error for exporting  gazebo_ros_joint_state_publisher
  * Add gazebo_ros_diff_drive to CMakeLists.txt
  * Basic structures updated
  includes updated
  include guards updated
  CMake rules added
  Not compiling yet
  * starting deboostifying
  updating lock
  header passing compile
  diff drive plugin compiling
  clear all references to callback queue
  * pimpl, remove joint state publisher
  * documentation, add TF publishers - commands and publishers work, but visualization on RViz is jerky, must check
  * pass linters
  * check that reset works now, rename params, add missing package
  * remap topics, add pub/sub test
  * sleep longer to see if it passes on Jenkins
* Remove node_name from <ros> SDF tag (`#804 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/804>`_)
  * Rename Node::Create to Node::Get
  * Node::Get without node name
  * Remove node_name support from SDF
  * wip get name from plugin name
  * Remove node name argument (will be inferred from sdf)
  * fix tests and implement static shared node
  * Adding test file
* [ros2] Split conversions into headers specific to message packages (`#803 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/803>`_)
  * Tests depend on sensor_msgs
  * Move conversions to different headers to minimise deps brought in
  * Remove conversions namespace
  * Include updates
  * Update message package dependencies
  gazebo_ros doesn't need sensor_msgs or geometry_msgs anymore
  * Export msg pacakges so downstream packages depend
  * Include msg headers used directly
  * removing redundant dependencies
  * fix build and cpplint
* working demo, notes and warnings about issues
* fix build by adding includes
* Test correctness of ray_sensor intensity
* Add Point32->ign vector conversion, fix pointcloud conversion
* Simplify ray_sensor using gazebo_ros conversions
* Add LaserScan conversions to gazebo_ros
* [ros2] Add clock publisher to gazebo_ros_init for use_sim_time support (`#794 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/794>`_)
  * Add Throttler to gazebo_ros utilities
  * Add sim time to gazebo_ros_init
  * Remove period constructor from Throttler
  * Improve sim time test
  * Fix compilation in isolation for gazebo_ros_init
  * Transient local durability for clock publisher
  * Linter fixup
  * Document Throttler will return true on first call
  * Store rate as double not Time
  * Import order improvements
* [ros2] Port gazebo_ros_imu_sensor (`#793 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/793>`_)
  * Move files to prepare for imu_sensor ROS2 port
  * Port gazebo_ros_imu_sensor
  * Address IMU Sensor PR comments
  * Remove empty <imu> tag
  * document that always_on is required
  * alphabetical order includes
  * Step far forward instead of multiple small steps
  * Fix test_conversions not finding quaternion.hpp
  * Apply force longer; check IMU values; robust to negative linear accel
  * linter fixup
* [ros2] gazebo_ros_joint_state_publisher (`#795 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/795>`_)
  * Port joint_state_publisher, copyright failing checker, still need to add a test
  * Fix copyright
  * Tests for joint state publisher
  * cleanup
  * depend on sensor_msgs
  * Use node's logger
* Merge pull request `#796 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/796>`_ from ros-simulation/ros2_fix_ci_authors
  [ros2] Fix missing dependencies to run CI and update maintainers
* Missing dependency in gazebo_ros
* Add SensorFrameID utility function
* Add NoiseVariance method for NoisePtr type
* Add geometry quaternion -> ignition conversion
* PR Comments for gazebo_ros utils
* Add gazebo_ros utils for utility functions
* Add time and quaternion conversions
* Add testing_utils to reduce duplicate code in tests
* PR feedback
* conversions
* improve example, add demo world, fix sdf warnings
* Add Node::Create with sdf element
  Move ament linting back to main CmakeList
  Various style fixes
  Only catch RCL_NOT_INIT exception in Node::Create
  Add larger timeouts to tests (stil flakey)
* [ros2] gazebo_ros_init plugin (`#776 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/776>`_)
  gazebo_ros_init plugin and very basic launch file
* Fix bug in test_plugins not ensuring all topics were received
* Call init from node in case it hasn't been called yet
* Remove internal logic to check init, add more tests
* Remove Node::Create using sdf until it is implemented
* Add simple test for gazebo_ros::Node
* Enable linters and make them happy
* Create base Node class for gazebo plugins with ROS2
* Move gazebo_ros files for porting
* Contributors: Jose Luis Rivero, Kevin Allen, Louise Poubel, Tully Foote, chapulina, dhood

2.8.4 (2018-07-06)
------------------
* Refactor spawn_model script
  * more robust -package_to_model implementation (issue #449)
  * add stdin as source option
  * parse arguments with argparse
  * remove deprecated/unused -gazebo and -trimesh options
* Fix physics reconfigure within namespace (issue #507)
* Contributors: Kevin Allen, Steven Peters

2.8.3 (2018-06-04)
------------------
* Use generic SIGINT parameter in kill command for gazebo script (melodic-devel) (`#724 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/724>`_)
  * Use generic SIGINT parameter in kill command for gazebo script
  * redirect to kill command to std_err
* Contributors: Jose Luis Rivero

2.8.2 (2018-05-09)
------------------
* Fix the build on Ubuntu Artful. (`#715 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/715>`_)
  Artful has some bugs in its cmake files for Simbody that
  cause it to fail the build.  If we are on artful, remove
  the problematic entries.
  Signed-off-by: Chris Lalancette <clalancette@openrobotics.org>
* Contributors: Chris Lalancette

2.8.1 (2018-05-05)
------------------
* Parameter to disable ROS network interaction from/to Gazebo (lunar-devel) (`#704 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/704>`_)
* Load the libgazebo_ros_api_plugin when starting gzclient so that the ROS event loop will turn over, which is required when you have a client-side Gazebo plugin that uses ROS. (`#676 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/676>`_)
* Pass verbose argument to gzclient (`#677 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/677>`_)
* strip comments from parsed urdf (`#698 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/698>`_)
  Remove comments from urdf before trying to find packages. Otherwise non-existant packages will produce a fatal error, even though they are not used.
* Contributors: Jose Luis Rivero

2.7.4 (2018-02-12)
------------------
* Fix last gazebo8 warnings! (lunar-devel) (`#664 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/664>`_)
* Fix for relative frame errors (lunar-devel) (`#663 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/663>`_)
* Fix gazebo8 warnings part 7: retry `#642 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/642>`_ on lunar (`#660 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/660>`_)
* Fix gazebo8 warnings part 10: ifdefs for GetModel, GetEntity, Light (lunar-devel) (`#657 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/657>`_)
* gazebo8 warnings: ifdefs for Get.*Vel() (`#655 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/655>`_)
* [gazebo_ros] don't overwrite parameter "use_sim_time" (lunar-devel) (`#607 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/607>`_)
* Fix gazebo8 warnings part 8: ifdef's for GetWorldPose (lunar-devel) (`#652 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/652>`_)
* Prevents GAZEBO_MODEL_DATABASE_URI from being overwritten (`#649 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/649>`_)
* for gazebo8+, call functions without Get (`#640 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/640>`_)
* Contributors: Jose Luis Rivero, Steven Peters

2.7.3 (2017-12-11)
------------------
* gazebo_ros_api_plugin: improve plugin xml parsing (`#627 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/627>`_)
* Fix gazebo8 warnings part 5: ignition math in gazebo_ros (lunar-devel) (`#636 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/636>`_)
* Fix gazebo8 warnings part 4: convert remaining local variables in plugins to ign-math (lunar-devel) (`#634 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/634>`_)
* gazebo_ros: fix support for python3 (`#629 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/629>`_)
* Replace Events::Disconnect* with pointer reset (`#626 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/626>`_)
* Install spawn_model using catkin_install_python (`#624 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/624>`_)
* Quote arguments to echo in libcommon.sh (`#591 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/591>`_)
* Contributors: Jose Luis Rivero

2.7.2 (2017-05-21)
------------------
* Revert gazebo8 changes in Lunar and back to use gazebo7 (`#583 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/583>`_)
* Contributors: Jose Luis Rivero

2.7.1 (2017-04-28)
------------------
* Fixes for compilation and warnings in Lunar-devel  (`#573 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/573>`_)
  Multiple fixes for compilation and warnings coming from Gazebo8 and ignition-math3
* Add catkin package(s) to provide the default version of Gazebo - take II (kinetic-devel) (`#571 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/571>`_)
* Contributors: Jose Luis Rivero

2.5.12 (2017-04-25)
-------------------

2.5.11 (2017-04-18)
-------------------
* Changed the spawn model methods to spawn also lights. (`#511 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/511>`_)
* Change build system to set DEPEND on Gazebo/SDFormat (fix catkin warning)
  Added missing DEPEND clauses to catkin_package to fix gazebo catkin warning.
  Note that after the change problems could appear related to -lpthreads
  errors. This is an known issue related to catkin:
  https://github.com/ros/catkin/issues/856.
* Use correct logerr method (`#557 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/557>`_)
* Contributors: Alessandro Ambrosano, Dave Coleman, Gary Servin

2.5.10 (2017-03-03)
-------------------
* Revert catkin warnings to fix regressions (problems with catkin -lpthreads errors)
  For reference and reasons, please check:
  https://discourse.ros.org/t/need-to-sync-new-release-of-rqt-topic-indigo-jade-kinetic/1410/4
  * Revert "Fix gazebo catkin warning, cleanup CMakeLists (`#537 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/537>`_)"
  This reverts commit 5a0305fcb97864b66bc2e587fc0564435b4f2034.
  * Revert "Fix gazebo and sdformat catkin warnings"
  This reverts commit 11f95d25dcd32faccd2401d45c722f7794c7542c.
* Contributors: Jose Luis Rivero

2.5.9 (2017-02-20)
------------------
* Fix gazebo catkin warning, cleanup CMakeLists (`#537 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/537>`_)
* Namespace console output (`#543 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/543>`_)
* Removed all trailing whitespace
* Contributors: Dave Coleman

2.5.8 (2016-12-06)
------------------
* Workaround to support gazebo and ROS arguments in the command line
* Fix ROS remapping by reverting "Remove ROS remapping arguments from gazebo_ros launch scripts.
* Fixed getlinkstate service's angular velocity return
* Honor GAZEBO_MASTER_URI in gzserver and gzclient
* Contributors: Jared, Jon Binney, Jordan Liviero, Jose Luis Rivero, Martin Pecka

2.5.7 (2016-06-10)
------------------

2.5.6 (2016-04-28)
------------------
* Remove deprecated spawn_gazebo_model service
* Contributors: Steven Peters

2.5.5 (2016-04-27)
------------------
* merge indigo, jade to kinetic-devel
* Upgrade to gazebo 7 and remove deprecated driver_base dependency
  * Upgrade to gazebo 7 and remove deprecated driver_base dependency
  * disable gazebo_ros_control until dependencies are met
  * Remove stray backslash
* spawn_model: adding -b option to bond to the model and delete it on sigint
* Update maintainer for Kinetic release
* Allow respawning gazebo node.
* Contributors: Hugo Boyer, Isaac IY Saito, Jackie Kay, Jonathan Bohren, Jose Luis Rivero, Steven Peters

2.5.3 (2016-04-11)
------------------
* Include binary in runtime
* Remove ROS remapping arguments from gazebo_ros launch scripts.
* Contributors: Jose Luis Rivero, Martin Pecka

2.5.2 (2016-02-25)
------------------
* merging from indigo-devel
* Merge pull request `#302 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/302>`_ from maxbader/jade-devel-GetModelState
  Header for GetModelState service request for jade-devel
* Fix invalid signal name on OS X
  scripts/gazebo: line 30: kill: SIGINT: invalid signal specification
* Fix invalid signal name on OS X
  scripts/gazebo: line 30: kill: SIGINT: invalid signal specification
* Restart package resolving from last position, do not start all over.
* 2.4.9
* Generate changelog
* Import changes from jade-branch
* Add range world and launch file
* fix crash
* Set GAZEBO_CXX_FLAGS to fix c++11 compilation errors
* GetModelState modification for jade
* Contributors: Bence Magyar, Boris Gromov, Guillaume Walck, Ian Chen, John Hsu, Jose Luis Rivero, Markus Bader, Steven Peters, hsu

2.5.1 (2015-08-16)
------------------
* Port of Pal Robotics range sensor plugin to Jade
* Added a comment about the need of libgazebo5-dev in runtime
* Added missing files
* Added elevator plugin
* Use c++11
* run_depend on libgazebo5-dev (`#323 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/323>`_)
  Declare the dependency.
  It can be fixed later if we don't want it.
* Contributors: Jose Luis Rivero, Nate Koenig, Steven Peters

* Port of Pal Robotics range sensor plugin to Jade
* Added a comment about the need of libgazebo5-dev in runtime
* Added missing files
* Added elevator plugin
* Use c++11
* run_depend on libgazebo5-dev
* Contributors: Jose Luis Rivero, Nate Koenig, Steven Peters

2.5.0 (2015-04-30)
------------------
* run_depend on libgazebo5-dev instead of gazebo5
* Changed the rosdep key for gazebo to gazebo5, for Jade Gazebo5 will be used.
* Contributors: Steven Peters, William Woodall

2.4.10 (2016-02-25)
-------------------
* Fix invalid signal name on OS X
  scripts/gazebo: line 30: kill: SIGINT: invalid signal specification
* Restart package resolving from last position, do not start all over.
* Contributors: Boris Gromov, Guillaume Walck

2.4.9 (2015-08-16)
------------------
* Import changes from jade-branch
* Add range world and launch file
* fix crash
* Set GAZEBO_CXX_FLAGS to fix c++11 compilation errors
* Contributors: Bence Magyar, Ian Chen, Jose Luis Rivero, Steven Peters

2.4.8 (2015-03-17)
------------------
* Specify physics engine in args to empty_world.launch
* Contributors: Steven Peters

2.4.7 (2014-12-15)
------------------
* temporary hack to **fix** the -J joint position option (issue `#93 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/93>`_), sleeping for 1 second to avoid race condition. this branch should only be used for debugging, merge only as a last resort.
* Fixing set model state method and test
* Extended the fix for `#246 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/246>`_ also to debug, gazebo, gzclient and perf scripts.
* Update Gazebo/ROS tutorial URL
* [gazebo_ros] Fix for `#246 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/246>`_
  Fixing issue `#246 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/246>`_ in gzserver.
* Fixing handling of non-world frame velocities in setModelState.
* update headers to apache 2.0 license
* update headers to apache 2.0 license
* Contributors: John Hsu, Jose Luis Rivero, Martin Pecka, Tom Moore, ayrton04

2.4.6 (2014-09-01)
------------------
* Merge pull request `#232 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/232>`_ from ros-simulation/fix_get_physics_properties_non_ode
  Fix get physics properties non ode
* Merge pull request `#183 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/183>`_ from ros-simulation/issue_182
  Fix STL iterator errors, misc. cppcheck (`#182 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/182>`_)
* check physics engine type before calling set_physics_properties and get_physics_properteis
* check physics engine type before calling set_physics_properties and get_physics_properteis
* Fixes for calling GetParam() with different physic engines.
* 2.3.6
* Update changelogs for the upcoming release
* Fixed boost any cast
* Removed a few warnings
* Update for hydro + gazebo 1.9
* Fix build with gazebo4 and indigo
* Fix STL iterator errors, misc. cppcheck (`#182 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/182>`_)
  There were some errors in STL iterators.
  Initialized values of member variables in constructor.
  Removed an unused variable (model_name).
* Contributors: Carlos Aguero, John Hsu, Jose Luis Rivero, Nate Koenig, Steven Peters, hsu, osrf

2.4.5 (2014-08-18)
------------------
* Port fix_build branch for indigo-devel
  See pull request `#221 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/221>`_
* Contributors: Jose Luis Rivero

2.4.4 (2014-07-18)
------------------
* Fix repo names in package.xml's
* fix issue `#198 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/198>`_
  Operator ``==`` is not recognized by sh scripts.
* Add verbose parameter
  Add verbose parameter for --verbose gazebo flag
* added osx support for gazebo start scripts
* Contributors: Arn-O, Jon Binney, Markus Achtelik, Vincenzo Comito

2.4.3 (2014-05-12)
------------------
* added osx support for gazebo start scripts
* Remove gazebo_ros dependency on gazebo_plugins
* Contributors: Markus Achtelik, Steven Peters

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

2.3.5 (2014-03-26)
------------------
* gazebo_ros: [less-than-minor] fix newlines
* gazebo_ros: remove assignment to self
  If this is needed for any twisted reason, it should be made clear
  anyway. Assuming this line is harmless and removing it because it
  generates cppcheck warnings.
* Contributors: Paul Mathieu

2.3.4 (2013-11-13)
------------------
* rerelease because sdformat became libsdformat, but we also based change on 2.3.4 in hydro-devel.
* remove debug statement
* fix sdf spawn with initial pose
* fix sdf spawn with initial pose
* Merge branch 'hydro-devel' into ``spawn_model_pose_fix``
* fix indentation
* Merge pull request `#142 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/142>`_ from hsu/hydro-devel
  fix issue `#38 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/38>`_, gui segfault on model deletion
* Merge pull request `#140 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/140>`_ from ``v4hn/spawn_model_sleep``
  replace time.sleep by rospy.Rate.sleep
* fix spawn initial pose.  When model has a non-zero initial pose and user specified initial model spawn pose, add the two.
* fix issue `#38 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/38>`_, gui segfault on model deletion by removing an obsolete call to set selected object state to "normal".
* replace time.sleep by rospy.Rate.sleep
  time was not even imported, so I don't know
  why this could ever have worked...
* Add time import
  When using the -wait option the script fails because is missing the time import
* Use pre-increment for iterators
* Fix iterator erase() problems

2.4.0 (2013-10-14)
------------------

2.3.3 (2013-10-10)
------------------
* Cleaned up unnecessary debug output that was recently added
* Fixed issue where ``catkin_find`` returns more than one library if it is installed from both source and debian

2.3.2 (2013-09-19)
------------------
* Make gazebo includes use full path
  In the next release of gazebo, it will be required to use the
  full path for include files. For example,
  `include <physics/physics.hh>` will not be valid
  `include <gazebo/physics/physics.hh>` must be done instead.
* update gazebo includes
* Fixed a minor typo in spawn_model error message when `-model` not specified

2.3.1 (2013-08-27)
------------------
* Cleaned up template, fixes for header files

2.3.0 (2013-08-12)
------------------
* gazebo_ros: fixed missing dependency on TinyXML
* gazebo_plugins: replace deprecated boost function
  This is related to `this gazebo issue <https://bitbucket.org/osrf/gazebo/issue/581/boost-shared_-_cast-are-deprecated-removed>`_

2.2.1 (2013-07-29)
------------------

2.2.0 (2013-07-29)
------------------
* Switched to pcl_conversions
* Remove find_package(SDF) from CMakeLists.txt
  It is sufficient to find gazebo, which will export the information
  about the SDFormat package.

2.1.5 (2013-07-18)
------------------
* gazebo_ros: fixed variable names in gazebo_ros_paths_plugin

2.1.4 (2013-07-14)
------------------

2.1.3 (2013-07-13)
------------------

2.1.2 (2013-07-12)
------------------
* Added author
* Tweak to make SDFConfig.cmake
* Cleaned up CMakeLists.txt for all gazebo_ros_pkgs
* Cleaned up gazebo_ros_paths_plugin
* 2.1.1

2.1.1 (2013-07-10 19:11)
------------------------
* Merge branch 'hydro-devel' of github.com:ros-simulation/gazebo_ros_pkgs into hydro-devel
* Reduced number of debug msgs
* Fixed physics dynamic reconfigure namespace
* gazebo_ros_api_plugin: set `plugin_loaded_` flag to true in
  GazeboRosApiPlugin::Load() function
* Actually we need `__init__.py`
* Cleaning up code
* Moved gazebo_interface.py from gazebo/ folder to gazebo_ros/ folder
* Removed searching for plugins under 'gazebo' pkg because of rospack warnings
* Minor print modification
* Added dependency to prevent missing msg header, cleaned up CMakeLists

2.1.0 (2013-06-27)
------------------
* gazebo_ros: added deprecated warning for packages that use gazebo as
  package name for exported paths
* Hiding some debug info
* gazebo_ros: use rosrun in debug script, as rospack find gazebo_ros returns the wrong path in install space
* Hide Model XML debut output to console
* gazebo_ros_api_plugin.h is no longer exposed in the include folder
* Added args to launch files, documentation
* Merge pull request `#28 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/28>`_ from osrf/no_roscore_handling
  Better handling of gazebo_ros run when no roscore started
* gazebo_ros: also support gazebo instead of gazebo_ros as package name for plugin_path, gazebo_model_path or gazebo_media_path exports
* gazebo_plugins/gazebo_ros: fixed install directories for include files and gazebo scripts
* changed comment location
* added block comments for walkChildAddRobotNamespace
* SDF and URDF now set robotNamespace for plugins
* Better handling of gazebo_ros run when no roscore started

2.0.2 (2013-06-20)
------------------
* Added Gazebo dependency
* changed the final kill to send a SIGINT and ensure only the last background process is killed.
* modified script to work in bash correctly (tested on ubuntu 12.04 LTS)

2.0.1 (2013-06-19)
------------------
* Incremented version to 2.0.1
* Fixed circular dependency, removed deprecated pkgs since its a stand alone pkg
* Shortened line lengths of function headers

2.0.0 (2013-06-18)
------------------
* Changed version to 2.0.0 based on gazebo_simulator being 1.0.0
* Updated package.xml files for ros.org documentation purposes
* Combined updateSDFModelPose and updateSDFName, added ability to spawn SDFs from model database, updates SDF version to lastest in parts of code, updated the tests
* Renamed Gazebo model to SDF model, added ability to spawn from online database
* Fixed really obvious error checking bug
* Deprecated -gazebo arg in favor of -sdf tag
* Reordered services and messages to be organized and reflect documentation. No code change
* Cleaned up file, addded debug info
* Merged changes from Atlas ROS plugins, cleaned up headers
* Small fixes per ffurrer's code review
* Deprecated warnings fixes
* Cleaned up comment blocks - removed from .cpp and added to .h
* Merged branches and more small cleanups
* Small compile error fix
* Standardized function and variable naming convention, cleaned up function comments
* Reduced debug output and refresh frequency of robot spawner
* Converted all non-Gazebo pointers to boost shared_ptrs
* Removed old Gazebo XML handling functions - has been replaced by SDF, various code cleanup
* Removed the physics reconfigure node handle, switched to async ROS spinner, reduced required while loops
* Fixed shutdown segfault, renamed `rosnode_` to `nh_`, made all member variables have `_` at end, formatted functions
* Added small comment
* adding install for gazebo_ros launchfiles
* Formatted files to be double space indent per ROS standards
* Started fixing thread issues
* Fixing install script names and adding gzserver and gdbrun to install command
* Fixed deprecated warnings, auto formatted file
* Cleaned up status messages
* Added -h -help --help arguemnts to spawn_model
* Removed broken worlds
* Removed deprecated namespace argument
* Using pkg-config to find the script installation path.
  Corrected a bash typo with client_final variable in gazebo script.
* Cleaning up world files
* Deprecated fix
* Moved from gazebo_worlds
* Cleaning up launch files
* Moved from gazebo_worlds
* Fixing renaming errors
* Updated launch and world files and moved to gazebo_ros
* Combined gzclient and gzserver
* Added finished loading msg
* All packages building in Groovy/Catkin
* Imported from bitbucket.org
