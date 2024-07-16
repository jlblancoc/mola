.. _solutions:

=========================
Solutions and pricing
=========================

Solutions
===============

1. Flexible LIDAR odometry and Localization
------------------------------------------
:ref:`LiDAR odometry <mola_lidar_odometry>` is one of the most advanced and flexible LIDAR odometry modules out there.
Check out the tutorial: :ref:`building-maps`.

.. image:: https://mrpt.github.io/imgs/mola-slam-kitti-demo.gif


|



2. Full SLAM solution (GNSS, submapping, loop closures)
--------------------------------------------------------

Build **geo-referenced** consistent global maps, even mixing indoor and outdoor scenarios.

.. image:: https://mrpt.github.io/imgs/kaist01_georef_sample.png

|

License and pricing
=====================
The complete framework comprises these software repositories:

.. _MRPT: https://github.com/MRPT/mrpt
.. |MRPT| replace:: **MRPT** 

.. _mp2p_icp: https://github.com/MOLAorg/mp2p_icp/
.. |mp2p_icp| replace:: **mp2p_icp** 

.. _mrpt_navigation: https://github.com/mrpt-ros-pkg/mrpt_navigation/
.. |mrpt_navigation| replace:: **mrpt_navigation** 

.. _MOLA: https://github.com/MOLAorg/mola
.. |MOLA| replace:: **MOLA** 

.. _mola_lidar_odometry: https://github.com/MOLAorg/mola_lidar_odometry/
.. |mola_lidar_odometry| replace:: **mola_lidar_odometry**

.. list-table:: Software repositories and modules
   :widths: 75 25
   :header-rows: 1

   * - Repository
     - License

   * - |MRPT|_

       Underlying C++ data structures, algorithms, serialization, RawLog datasets, etc.
     - BSD-3

   * - |mp2p_icp|_

       Generic ICP algorithm, metric map pipelines.
     - BSD-3

   * - |mrpt_navigation|_

       ROS 2 nodes: ``*.mm`` `metric map server <https://github.com/mrpt-ros-pkg/mrpt_navigation/tree/ros2/mrpt_map_server>`_,
       `AMCL-like localization <https://github.com/mrpt-ros-pkg/mrpt_navigation/tree/ros2/mrpt_pf_localization>`_,
       `point cloud pipeline <https://github.com/mrpt-ros-pkg/mrpt_navigation/tree/ros2/mrpt_pointcloud_pipeline>`_,
       etc.
     - BSD-3

   * - |MOLA|_

       MOLA modules: kernel, mola_viz, kinematic state estimator, relocalization, etc.
     - GNU-GPLv3

   * - |mola_lidar_odometry|_

       :ref:`LiDAR odometry <mola_lidar_odometry>` for mapping and optimization-based localization.
     - GNU-GPLv3

   * - **mola_sm_loop_closure**

       Map geo-referencing, SLAM with loop-closure for consistent large maps.
     - Upon licensing only

|

Contact
===========
To request details on **licensing a closed-source version for commercial usages** and/or **consulting services**, please use the contact form below:

.. raw:: html

    <div style="margin-top:10px;">
      <iframe src="https://docs.google.com/forms/d/e/1FAIpQLSdgFfPclN7MuB4uKIbENxUDgC-pmimcu_PGcq5-vAALjUAOrg/viewform?embedded=true" width="700" height="1500" frameborder="0" marginheight="0" marginwidth="0">Loading…</iframe>
    </div>
