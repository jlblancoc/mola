^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mola_bridge_ros2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


1.0.3 (2024-04-22)
------------------
* BridgeROS2: more robust /tf find_transform by using tf2::BufferCore
* FIXBUG: inverse sensor poses in rosbag2 reader.
  Also: unify notation in C++ calls to lookupTransform()
* Fix package.xml website URL
* Contributors: Jose Luis Blanco-Claraco

1.0.2 (2024-04-04)
------------------
* update docs
* Contributors: Jose Luis Blanco-Claraco

1.0.1 (2024-03-28)
------------------
* BridgeROS2: do not quit on temporary /tf timeout
* mola_bridge_ros2: option to publish /tf_static for base_footprint
* mola_bridge_ros2: implement missing MOLA->ROS2 conversion for GNNS observations
* BUGFIX: Inverted value of "use_fixed_sensor_pose" was used
* Contributors: Jose Luis Blanco-Claraco

1.0.0 (2024-03-19)
------------------
* Comply with ROS2 REP-2003
* Merge ROS2 input and output in one module
* Contributors: Jose Luis Blanco-Claraco

0.2.2 (2023-09-08)
------------------