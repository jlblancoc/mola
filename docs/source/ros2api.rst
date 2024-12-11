.. _mola_ros2api:

======================
ROS 2 API
======================
This page reflects the topics and services that a MOLA system will expose when running a SLAM 
or LiDAR-odometry module. At present, this applies to:

- Any MOLA system including the :ref:`BridgeROS2<doxid-classmola_1_1_bridge_r_o_s2>` module:
  This module acts as a wrapper of ``mola-kernel`` virtual interfaces implemented in other
  MOLA modules and the ROS 2 system.
- :ref:`mola_lidar_odometry`

.. note::

   It is recommended to start with the tutorial on how to :ref:`build a map <building-maps>`,
   then check out :ref:`how to launch MOLA-LO for ROS 2 <mola_lo_ros>`.

.. image:: https://mrpt.github.io/imgs/mola-lo-ros2-launch-demo-kitti.png



____________________________________________

.. contents:: Table of Contents
   :depth: 1
   :local:
   :backlinks: none

____________________________________________

|

1. Map loading / saving
--------------------------------------
Write me!


|

----

2. Re-localization
--------------------------------------
Write me!

|

----

3. ``/tf`` frames
--------------------------------------
These frames of reference exist when using MOLA :ref:`geo-referenced <geo-referencing>` maps:

.. figure:: https://mrpt.github.io/imgs/mola_mrpt_ros_geo_referenced_utm_frames.png
   :width: 500
   :align: center


.. note::

   For non geo-referenced maps, the meaning of all frames are the same but ``utm`` and ``enu`` 
   will not be present.






