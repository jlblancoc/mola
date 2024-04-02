# mola_bridge_ros2
RawDataSource acting as a bidirectional bridge between ROS2 and MOLA modules.

Can be used to:
* ROS2->MOLA: Interface a real sensor using a ROS driver node and run SLAM with it. See `mola_lidar_odometry` [demos](https://github.com/MOLAorg/mola_lidar_odometry/tree/develop/ros2-launchs).
* MOLA->ROS2: Expose a dataset as ROS2 topics, from any of the datasets supported by MOLA. Example: [kitti](https://github.com/MOLAorg/mola/blob/develop/mola_demos/ros2-launchs/ros-kitti-play.launch.py)
* ROS2<->MOLA: Run SLAM on a live sensor stream, then send back the reconstructed map and trajectory to ROS for further processing or visualization in RViz. See `mola_lidar_odometry` [demos](https://github.com/MOLAorg/mola_lidar_odometry/tree/develop/ros2-launchs).

If you want to run SLAM on a rosbag, the module `mola_input_rosbag2` provides
a more convenient interface, e.g. allows fast-forwarding or skipping parts of a bag.

Building this module requires ROS 2 to be installed, and its `setup.bash`
activation script being sourced **before** invoking CMake to configure and build MOLA.

See package docs for instructions and options to install ROS prerequisites.

Provided MOLA modules:
* `BridgeROS2`, type RawDataSourceBase.

## Build and install
Refer to the [root MOLA repository](https://github.com/MOLAorg/mola).

## Docs and examples
See this package page [in the documentation](https://docs.mola-slam.org/latest/modules.html).

## License
This package is released under the BSD 3-clause license.
