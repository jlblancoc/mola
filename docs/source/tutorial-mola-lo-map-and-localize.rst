.. _tutorial-mola-lo-map-and-localize:

===============================================
MOLA-LO: Build a map and then localize
===============================================

This tutorial will show you how to build a map using MOLA-LO, then save the map to disk, 
and how to load that map to use the LO localization mode.

.. contents::
   :depth: 1
   :local:
   :backlinks: none

|

MOLA installation
----------------------------------
This tutorial requires the installation of these MOLA packages: ``mola_demos``, ``mola_viz``.

Following the default :ref:`installation instructions <installing>` is enough.

It also uses the MVSim simulator, although a live robot or LiDAR sensor can be used instead.

|

Create a map and save it
----------------------------------

Open three (3) terminals, and run these commands in each one:

In terminal #1, launch the simulator (or your custom launch for your real sensor):

.. code-block:: bash

    ros2 launch mvsim demo_warehouse.launch.py \
      do_fake_localization:=False \
      with_rviz:=False

|

In terminal #2, launch MOLA-LO, enabling saving the map in simple-map format:

.. code-block:: bash

    MOLA_GENERATE_SIMPLEMAP=true \
    ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
      lidar_topic_name:=/lidar1_points

.. note::

  Remember replacing ``/lidar1_points`` with your actual PointCloud2 topic with raw LiDAR data.

Next, move the robot around. In the simulator, you can click on the MVSim GUI and use keys ``asdw`` to drive around, or use a gamepad.

|

Once the map looks OK in the mola_viz GUI, let's save it.
In terminal #3, run:

.. code-block:: bash

    ros2 service call /map_save mola_msgs/srv/MapSave "map_path: '/tmp/my_map'"

