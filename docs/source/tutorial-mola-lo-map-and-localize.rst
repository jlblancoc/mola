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

This video shows the steps in the tutorial:

.. raw:: html

    <div style="margin-top:10px;">
      <iframe width="560" height="315" src="https://www.youtube.com/embed/xxxx" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
    </div>

|

Prerequisites: MOLA installation
----------------------------------
This tutorial requires the installation of these packages: ``mola_lidar_odometry``, ``mola_viz``, ``mvsim``.
The MVSim simulator is used as an example, but a live robot or LiDAR sensor can be used instead.

Following the default :ref:`installation instructions <installing>` is enough.


|

1. Create a map and save it
----------------------------------

Open **three terminals**, and run these commands in each one:

.. tab-set::

    .. tab-item:: #1: Simulator
      :selected:

      Launch the simulator or, if you are on a real robot, make sure to have the system up
      and LiDAR data coming out:

      .. code-block:: bash

          ros2 launch mvsim demo_warehouse.launch.py \
            do_fake_localization:=False \
            use_rviz:=False


    .. tab-item:: #2: MOLA-LO

        In terminal #2, launch MOLA-LO, enabling saving the map in simple-map format:

        .. code-block:: bash

            MOLA_GENERATE_SIMPLEMAP=true \
            ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
              lidar_topic_name:=/lidar1_points

        .. note::

          Remember replacing ``/lidar1_points`` with your actual PointCloud2 topic with raw LiDAR data.

        Next, **move the robot around**.
        In the simulator, you can click on the MVSim GUI and use keys ``a/d``+ ``s/w`` to drive around,
        or `use a gamepad to teleoperate it <https://mvsimulator.readthedocs.io/en/latest/teleoperation.html>`_.

    .. tab-item:: #3: Save the map

        Once the map looks OK in the mola_viz GUI, let's save it.
        In terminal #3, run:

        .. code-block:: bash

            ros2 service call /map_save mola_msgs/srv/MapSave "map_path: '/tmp/my_map'"

        Watch the response to check that ``success`` is ``true``. Your map is now
        stored as file(s) named ``/tmp/my_map*``.


|


2. Load a prebuilt map in localize-only mode
---------------------------------------------

..  note::

  Make sure of closing the former instance of MOLA-LO in terminal #2 used to build the map
  before going on with this part.

Again, we will use **three terminals**:

.. tab-set::

    .. tab-item:: #1: Simulator

      You can keep the former instance of the MVSim simulator running from the
      former step, or launch it again and move to a different pose, it is up to you!

    .. tab-item:: #2: MOLA-LO in localization mode
      :selected:

      Let's launch MOLA-LO in **non-mapping (localization only) mode** with:

      .. code-block:: bash

          MOLA_MAPPING_ENABLED=false \
          ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
            lidar_topic_name:=/lidar1_points

      .. note::

        Remember replacing ``/lidar1_points`` with your actual PointCloud2 topic with raw LiDAR data.

    .. tab-item:: #3: Load the map

        Next, in terminal #3, let's order MOLA-LO to **load our former map** from disk:

        .. code-block:: bash

            ros2 service call /map_load mola_msgs/srv/MapLoad "map_path: '/tmp/my_map'"



        Note that it is also possible to directly launch MOLA-LO with a map loaded from disk
        from the beginning, but it implies passing the two 

|

.. dropdown:: Directly loading the map from MOLA-LO start up

    Instead of first invoking MOLA-LO and then requesting to load the map via a ROS 2 service,
    it is possible to instruct MOLA-LO to start loading the map straightaway as it starts up
    by specifying the path to **both** map files, instead of the **map prefix** used
    in the ROS 2 service:

    .. code-block:: bash

        MOLA_MAPPING_ENABLED=false \
        MOLA_LOAD_MM=/tmp/my_map.mm \
        MOLA_LOAD_SM=/tmp/my_map.simplemap \
        ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
          lidar_topic_name:=/lidar1_points

    Of course, the ROS 2 service offers a greater flexibility to switch
    between maps at run-time.


|


3. Invoke relocalization
---------------------------------------------

As explained :ref:`here <localization-only_common>`, initial localization is a hard problem on its own
and can be handled in different ways.

Here we will show the common situation of wanting to re-localize the robot in a prebuilt map,
given we already know **a rough estimation of its actual pose**, including the orientation.
Check out :ref:`all the details <mola_ros2api_relocalization>` about requesting
relocalization via ROS 2 API.


.. tab-set::

    .. tab-item:: Re-localize with a topic
      :selected:

      Just use the RViz2's button ``2D pose estimate`` or FoxGlove's "Pose Estimate"
      to pick a pose and MOLA-LO will try to re-localize the vehicle in the given area.

    .. tab-item:: Re-localize with a service

      There is also a ROS 2 service for programmatically request a relocalization, and
      obtaining feedback about whether the request was received or not by a running MOLA module:

      - Service default name: ``/relocalize_near_pose``
      - Service interface: `mola_msgs::srv::RelocalizeNearPose <https://docs.ros.org/en/rolling/p/mola_msgs/interfaces/srv/RelocalizeNearPose.html>`_

