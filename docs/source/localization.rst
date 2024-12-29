.. _localization-only:

======================
Localization
======================
Localization differs from SLAM in that the map or world model is **not updated** while
it is being used to keep a robot or vehicle accurately localized within a pre-mapped environment :cite:`blanco2024mola_lo`.

Once :ref:`a map is built <building-maps>`, the result is a simple-map and/or a metric map (``*.mm``), optionally :ref:`georeferenced <geo-referencing>`.
This map can then be used to enable autonomous navigation ("Go from A to B"), for which localization is a fundamental
requirement ("Where is B?").

At present, this framework provides two fundamentally different algorithms for **localization**:

* **Based on optimization**, by using the :ref:`LiDAR odometry module <mola_lidar_odometry>`, without map updates.
* **Based on particle filtering**, by using any combination of metric maps (grid maps, point clouds, etc.) and wheels odometry.

____________________________________________

.. contents:: Table of Contents
   :depth: 1
   :local:
   :backlinks: none

____________________________________________

|

.. _localization-only_common:


1. Common aspects
--------------------------------------
Independently of the localization algorithm, the problem of finding the *rough* localization
of the vehicle at start up is quite special due to the large **initial uncertainty**.

MOLA provides a specific module for this task (:ref:`doxid-group__mola__relocalization__grp`)
which can be used by all position tracking algorithms to solve the initial relocalization problem.

Also, implementations below offer two ways to specify the **initial guess** about the robot
pose in the environment:

* From **a pose with a covariance**, which needs to be given manually either from a ROS topic, RViz/FoxGlove, or a parameter; and
* From **GNSS readings** ("Automatic re-localization via GPS"). This version requires :ref:`georeferenced metric maps <geo-referencing>`, naturally.

|

----

2. Localization with LiDAR odometry
--------------------------------------
See the step-by-step instructions in :ref:`this tutorial <tutorial-mola-lo-map-and-localize>`.
As a summary, you need to:

* Launch MOLA-LO in localization-only mode.
* Load a local map into it. In MOLA-LO, a local map comprises two files, a ``*.mm`` metric map and the keyframes in a ``*.simplemap`` file.
  (In fact: keyframes are only required if you want to extend the map, i.e. multi-session mapping).
* Request relocalization via a service call or via the topic ``/initialpose``.

This video shows the steps in the tutorial:

.. raw:: html

    <div style="margin-top:10px;">
      <iframe width="560" height="315" src="https://www.youtube.com/embed/CXiU_2vYMQE" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
    </div>

|

----

3. Localization with particle filtering
----------------------------------------
To localize with a particle filter (PF) you will need:

1. A metric map (the ``.mm`` file) of the environment. Built as described in: :ref:`building-maps`.
2. To publish that map using `mrpt_map_server <https://github.com/mrpt-ros-pkg/mrpt_navigation/tree/ros2/mrpt_map_server/>`_.
3. Launch `mrpt_pf_localization <https://github.com/mrpt-ros-pkg/mrpt_navigation/tree/ros2/mrpt_pf_localization>`_ and configure it
   to read from wheels odometry (if they exist), the input sensors (e.g. LiDAR), GPS (GNSS) if present. This node will subscribe to
   the published metric map and use the sensors to publish updates on the robot pose in the `map` frame.

.. note::

   Do **NOT** set the raw LiDAR pointcloud as input for the PF. It is just too large and it must be subsampled first.
   We can use `mrpt_pointcloud_pipeline <https://github.com/mrpt-ros-pkg/mrpt_navigation/tree/ros2/mrpt_pointcloud_pipeline>`_
   for such task, as can be seen in the tutorial below.

3.1. PF localization tutorial
================================
A complete demonstration has been put together on: https://github.com/MOLAorg/mola_warehouse_pf_tutorial

.. raw:: html

   <div style="width: 100%; overflow: hidden;">
     <video controls autoplay loop muted style="width: 512px;">
       <source src="https://mrpt.github.io/videos/mola-localization-pf-demo-warehouse.mp4" type="video/mp4">
     </video>
   </div>

Key points of this tutorial:

- The ROS 2 `launch file <https://github.com/MOLAorg/mola_warehouse_pf_tutorial/blob/develop/launch/tutorial_launch.py>`_.
- A `custom sm2mm pipeline file <https://github.com/MOLAorg/mola_warehouse_pf_tutorial/tree/develop/sm2mm-config>`_ to create
  a reference point cloud map sparse enough so localizing with a PF is not too slow.
- A `pipeline for mrpt_pointcloud_pipeline <https://github.com/MOLAorg/mola_warehouse_pf_tutorial/blob/develop/params/point-cloud-pipeline.yaml>`_
  to decimate the input raw 3D scan as input to the PF.


.. dropdown:: How to run the tutorial

   Clone the tutorial package (which already includes a prebuilt ``.mm``), make sure of having all dependencies,
   build and run it:

   .. code-block:: bash

         cd ~/ros2_ws/src
         git clone https://github.com/MOLAorg/mola_warehouse_pf_tutorial.git

         cd ~/ros2_ws
         rosdep install --from-paths src --ignore-src -r -y

         colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
         . install/setup.bash

         ros2 launch mola_warehouse_pf_tutorial tutorial_launch.py



