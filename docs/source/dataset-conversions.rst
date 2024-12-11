.. _dataset-conversions:

======================
Dataset conversions
======================

This page gives instructions on how to convert robotics datasets between
ROS 1 and ROS 2 bag files or live messages, MRPT's cross-platform RawLog format, and
the custom formats of popular robotics and computer vision datasets.

.. contents::
   :depth: 1
   :local:
   :backlinks: none


1. Dataset formats
---------------------------------
Robotics datasets consist of **sequences** of raw sensory data as captured by
a mobile robot, a vehicle, or just a hand-held device, as they move through the environment.

Each robotic framework or library has defined its own formats over time:

- **ROS 1** introduced `rosbags <https://wiki.ros.org/Bags>`_ (~2007) as a binary serialization
  storage for heterogeneous ROS messages.
- **ROS 2** (~2014) improved rosbags, which now can use different storage implementations (sqlite3, mcap),
  and are more efficient and flexible than older versions.
- `MRPT <https://github.com/MRPT/mrpt>`_, on which MOLA is internally based on, 
  defined its own serialized dataset format (~2005) named
  `RawLogs <https://docs.mrpt.org/reference/latest/class_mrpt_obs_CRawlog.html>`_, which is ensured
  to be portable across operative systems, backwards compatible across MRPT versions,
  and processor architecture independent.

.. note::
   Apart of these *standardized formats*, many robotic datasets have their own custom
   data layout (e.g. one binary file for each 3D LiDAR scan or image).
   MOLA makes easy reading them by providing *adapter modules* that can read the most
   popular datasets (see :ref:`list <supported_sensors>`) through unified C++ APIs
   (:ref:`doxid-classmola_1_1_offline_dataset_source`)
   or :ref:`re-publishing them to ROS <mola-dataset-to-ros>`.

|

.. _rosbag2rawlog:

2. rosbag ⇒ rawlog
----------------------------
To convert a ROS bag into a RawLog, you need two items: 
- The program ``rosbag2rawlog``,
- And a configuration file specifying what ROS messages should be converted.

There are two versions of ``rosbag2rawlog`` and the correct one must be installed
depending on whether you need to convert **ROS 1 or ROS 2 bags**:

.. tab-set::

    .. tab-item:: ROS 2 bags
      :selected:

      ``rosbag2rawlog`` for **ROS 2 bags**. It is shipped with the ROS package `mrpt_rawlog <https://github.com/mrpt-ros-pkg/mrpt_navigation/tree/ros2/mrpt_rawlog>`_
      so it can be installed with:

      .. code-block:: bash

        sudo apt install ros-${ROS_DISTRO}-mrpt-rawlog

    .. tab-item:: ROS 1 bags
      :selected:

      ``rosbag2rawlog`` for **ROS 1 bags** belongs to upstream MRPT, and it is generated only *if*
      ROS 1 is detected at cmake configuration time. You can either build MRPT from sources after activating
      your ROS 1 environment, or install the Ubuntu package ``mrpt-apps``, which already ships ``rosbag2rawlog``
      for Ubuntu versions <=22.04 (In Ubuntu 24.04, ROS 1 packages were removed upstream).
      To ensure having the latest version, consider installing it from `the PPA <https://launchpad.net/~joseluisblancoc/+archive/ubuntu/mrpt-stable>`_.

|

Once you have ``rosbag2rawlog`` installed it can be invoked like:

.. dropdown:: rosbag2rawlog CLI arguments
  :open:

  .. code-block:: bash

      $ rosbag2rawlog --help

      USAGE: 

        rosbag2rawlog  [-b <base_link>] [-w] -c <config.yml> -o
                        <dataset_out.rawlog> [--] [--version] [-h] <log.bag> ...


      Where: 

        -b <base_link>,  --base-link <base_link>
          Reference /tf frame for the robot frame (Default: 'base_link')

        -w,  --overwrite
          Force overwrite target file without prompting.

        -c <config.yml>,  --config <config.yml>
          (required)  Config yaml file (*.yml)

        -o <dataset_out.rawlog>,  --output <dataset_out.rawlog>
          (required)  Output dataset (*.rawlog)

        --,  --ignore_rest
          Ignores the rest of the labeled arguments following this flag.

        --version
          Displays version information and exits.

        -h,  --help
          Displays usage information and exits.

        <log.bag>  (accepted multiple times)
          (required)  Input bag files (required) (*.bag)


For example:

.. code-block:: bash

    $ rosbag2rawlog -c config.yaml -o output.rawlog  input.mcap

With the ``config.yaml`` file created as explained below.


2.1. YAML config file format
==============================
``rosbag2rawlog`` needs an input configuration file to determine **what ROS messages** in the bag are to be **imported**.
The rest will be discarded, after emitting a warning to the terminal.

.. dropdown:: Example: basic Ouster LiDAR import
  :open:

  .. code-block:: yaml

    # Config file for rosbag2rawlog. It must contain a top-level "sensors" node
    sensors:
      # Then, one node per sensor to convert. This name will be used as
      # sensorLabel in MRPT observations.
      lidar:
        # Type: C++ class name (see mrpt::obs)
        type: 'CObservationPointCloud'
        topic: '/ouster/points'
        # If uncommented, /tf data will be ignored for this sensor pose:
        #fixed_sensor_pose: "0 0 0 0 0 0"  # 'x y z yaw_deg pitch_deg roll_deg''

.. dropdown:: Example: Newer College Dataset import

  .. code-block:: yaml

    # Config file for rosbag2rawlog. It must contain a top-level "sensors" node
    sensors:
      # Then, one node per sensor to convert. This name will be used as 
      # sensorLabel in MRPT observations.
      lidar:
        # Type: C++ class name (see mrpt::obs)
        type: 'CObservationPointCloud'
        # Parameters for this particular type of sensor.
        # Topic to subscribe for the pointcloud:
        topic: '/os_cloud_node/points'
        fixed_sensor_pose: "0 0 0 0 0 0"  # 'x y z yaw_deg pitch_deg roll_deg''

      cam0:
        type: 'CObservationImage'
        image_topic: '/alphasense_driver_ros/cam0/compressed'
        fixed_sensor_pose: "0 0 0 0 0 0"  # 'x y z yaw_deg pitch_deg roll_deg''

.. dropdown:: Example: LiDAR + wheels odometry from /odom

  .. code-block:: yaml

    # Config file for rosbag2rawlog. It must contain a top-level "sensors" node
    sensors:
      # Then, one node per sensor to convert. This name will be used as
      # sensorLabel in MRPT observations.
      lidar:
        # Type: C++ class name (see mrpt::obs)
        type: 'CObservationPointCloud'
        topic: '/ouster/points'
        # If uncommented, /tf data will be ignored for this sensor pose:
        #fixed_sensor_pose: "0 0 0 0 0 0"  # 'x y z yaw_deg pitch_deg roll_deg''
      odom:
        type: 'CObservationOdometry'
        topic: '/odom'

.. dropdown:: Example: LiDAR + wheels odometry from /tf

  .. code-block:: yaml
    
    # Config file for rosbag2rawlog. It must contain a top-level "sensors" node
    sensors:
      # Then, one node per sensor to convert. This name will be used as
      # sensorLabel in MRPT observations.
      lidar:
        # Type: C++ class name (see mrpt::obs)
        type: 'CObservationPointCloud'
        topic: '/ouster/points'
        # If uncommented, /tf data will be ignored for this sensor pose:
        #fixed_sensor_pose: "0 0 0 0 0 0"  # 'x y z yaw_deg pitch_deg roll_deg''

    # If provided, odometry observations will be generated from /tf messages
    # from `odom_frame_id` to `base_link` (frame_id can be changed via cli arguments):
    odom_from_tf:
        sensor_label: 'odom'  # mandatory entry
        #odom_frame_id: '/odom'


|


3. rawlog ⇒ ROS
----------------------------
For ROS 1 and ROS 2.

Write me!


|


.. _ros1_to_ros2:

4. ROS1 ⇒ ROS2
----------------------------
One way to use rosbags from ROS 1 with MOLA is to port them to ROS 2 bags.

You can use the Python package ``rosbags`` to `perform the conversion <https://ternaris.gitlab.io/rosbags/topics/convert.html>`_.

An alternative is to use :ref:`rosbag2rawlog <rosbag2rawlog>` (the ROS 1 version!) to convert them to RawLogs, then use them as input to MOLA.

|

.. _mola-dataset-to-ros:

5. MOLA data set module ⇒ ROS
------------------------------

All you need is to put together a :ref:`MOLA launch YAML file <yaml_slam_cfg_file>` with:

- A dataset source module.
- The ros2bridge module.

and launch it using `mola-cli`. See existing ROS launch examples
under `ros2-launch (mola_demos package) <https://github.com/MOLAorg/mola/tree/develop/mola_demos/ros2-launchs>`_,
with corresponding MOLA cli launch files in the `mola-cli-launchs <https://github.com/MOLAorg/mola/tree/develop/mola_demos/mola-cli-launchs>`_ directory.

.. dropdown:: Example #1: play back a KITTI dataset sequence to ROS 2
  :open:

  .. code-block:: bash

    ros2 launch mola_demos ros-kitti-play.launch.py kitti_sequence:=01

  Result: the mola_viz GUI + RViz.

  .. figure:: https://mrpt.github.io/imgs/screenshot_mola_demo_kitti_replay_to_ros.jpg
    :width: 600

.. dropdown:: Example #2: play back a Mulran dataset sequence to ROS 2
  :open:

  Refer to the :ref:`complete tutorial <tutorial-mulran-replay-to-ros2>`.

