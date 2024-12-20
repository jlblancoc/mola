.. _mola_ros2api:

======================
ROS 2 API
======================
This page reflects the topics and services that a MOLA system will expose when running a SLAM 
or LiDAR-odometry module. At present, this applies to:

- Any MOLA system including the :ref:`BridgeROS2 <doxid-classmola_1_1_bridge_r_o_s2>` module:
  This module acts as a wrapper of ``mola-kernel`` virtual interfaces implemented in other
  MOLA modules and the ROS 2 system.
- :ref:`mola_lidar_odometry`

.. image:: https://mrpt.github.io/imgs/mola-lo-ros2-launch-demo-kitti.png

____________________________________________

.. contents:: Table of Contents
   :depth: 1
   :local:
   :backlinks: none

____________________________________________

|

1. Nodes and launch files
--------------------------------------

.. _ros2_node_lo_docs:

1.1. ROS 2 node for Lidar Odometry (LO)
============================================

.. include:: ../../../mola_lidar_odometry/docs/mola_lo_ros_node.rst

|

----

2. Map loading / saving
--------------------------------------
During a live SLAM run, ``BridgeROS2`` will look for modules implementing
:ref:`MapServer <doxid-classmola_1_1_map_server>` and will expose
these **ROS 2 services** to load or save the current map:

* ``/map_load``: See ROS docs for `mola_msgs/MapLoad <https://docs.ros.org/en/rolling/p/mola_msgs/interfaces/srv/MapLoad.html>`_

* ``/map_save``: See ROS docs for `mola_msgs/MapSave <https://docs.ros.org/en/rolling/p/mola_msgs/interfaces/srv/MapSave.html>`_

.. dropdown:: Example ROS 2 cli service calls

   To save the current map:

   .. code-block:: bash

      ros2 service call /map_save mola_msgs/srv/MapSave "map_path: '/tmp/my_map_file_prefix'"

   To load a map from disk:

   .. code-block:: bash

      ros2 service call /map_load mola_msgs/srv/MapLoad "map_path: '/tmp/my_map_file_prefix'"

Note that filename **extension** should not be given, since each service implementation
may add a different extension, or even save several files that should all, together, be
later on loaded as one to load the map again.

Alternatively, you can enable saving the map when mapping is ended by checking
the corresponding checkbox in the
:ref:`MOLA-LO GUI <mola_lo_gui_common_parts>` (block "6" below):

.. image:: imgs/gui_parts.png


|

----

.. _mola_ros2api_relocalization:

3. Re-localization
--------------------------------------
Write me!

|

----

.. _mola_ros2_tf_frames:

4. Published ``/tf`` frames
--------------------------------------
These frames of reference exist when using MOLA :ref:`geo-referenced <geo-referencing>` maps:

.. figure:: https://mrpt.github.io/imgs/mola_mrpt_ros_geo_referenced_utm_frames.png
   :width: 500
   :align: center

.. note::

   For non geo-referenced maps, all frames remain the same but ``utm`` and ``enu`` will not exist.

These are the existing frames:

- ``base_link``: The robot reference frame. For ground vehicles, normally placed at the
  center of the rear axle.
- ``odom``: The arbitrary origin for odometry measurements.
- ``map``: The origin of the reference metric map used for localization.
- ``enu``: For geo-referenced maps, the North (``y`` axis), East (``x`` axis), Up (``z`` axis) frame for which
  we have reference geodetic coordinates (latitude and longitude). Different maps built in the same zone
  will surely have different ``enu`` frames, since it is defined by collected GNSS measurements.
- ``utm``: The origin of the `UTM zone <https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system>`_
  in which ``enu`` falls. Unlike ``enu``, it is **independent** of the trajectory followed while building the map.

And this is who is responsible of publishing each transformation:

- ``odom → base_link``: Odometry module. High-frequency, accurate in the short term, but drifts in the long term.
- ``map → odom``: :ref:`Localization <localization>` module, which corrects the odometry drift.
- ``enu → {map, utm}``: Published by ``mrpt_map_server`` (`github <https://github.com/mrpt-ros-pkg/mrpt_navigation/tree/ros2/mrpt_map_server/>`_),
  if fed with a geo-referenced metric map (``.mm``) file.

|

----

|

.. _ros2api_topics:

5. Published topics
--------------------------------------
Write me!

|

----

|

6. Map publishing
--------------------------------------
There are two ways of publishing maps to ROS:

* Using ``mrpt_map_server`` (`github <https://github.com/mrpt-ros-pkg/mrpt_navigation/tree/ros2/mrpt_map_server/>`_):
  the recommended way for static, previously-built maps. In this case, one ROS topic
  will be published for each map layer, as described in the package documentation.
  See also :ref:`this tutorial <tutorial-pub-map-server-to-ros>`.

* During a live map building process (e.g. MOLA-LO).

In this latter case, BridgeROS2 will look for modules implementing
:ref:`MapSourceBase <doxid-classmola_1_1_map_source_base>` and will publish
one **topic** named ``<METHOD>/<LAYER_NAME>`` for each map layer.
The metric map layer C++ class will determine the ROS topic type to use.

.. note::

   Using the default MOLA LiDAR odometry pipeline, only one map topic will
   be generated during mapping:

   * Name: ``/lidar_odometry/localmap_points``
   * Type: ``sensor_msgs/PointCloud2``

|

----

|

.. _ros2api_runtime_params:

7. Runtime dynamic reconfiguration
----------------------------------------
MOLA modules may expose a subset of their parameters through an interface that allows
runtime reconfiguration via ROS 2 service requests:

7.1. Runtime parameters for ``mola_lidar_odometry``
======================================================

List all existing parameters:

   .. code-block:: bash

      ros2 service call /mola_runtime_param_get mola_msgs/srv/MolaRuntimeParamGet

.. dropdown:: Example output
  :open:

   .. code-block:: bash

      requester: making request: mola_msgs.srv.MolaRuntimeParamGet_Request()

      response:
      mola_msgs.srv.MolaRuntimeParamGet_Response(parameters='mola::LidarOdometry:lidar_odom:\n  active: true\n  generate_simplemap: false\n  mapping_enabled: true\n')

   Returned ``parameters`` as YAML:

   .. code-block:: yaml

      mola::LidarOdometry:lidar_odom:
        active: true
        generate_simplemap: false
        mapping_enabled: true

Documented parameters:

- ``active``: Whether MOLA-LO should process incoming sensor data (``active: true``)
  or ignore them (``active: false``).

.. dropdown:: Copy & paste commands for ``active``

   .. code-block:: bash

      # active: true
      ros2 service call /mola_runtime_param_set mola_msgs/srv/MolaRuntimeParamSet \
         "{parameters: \"mola::LidarOdometry:lidar_odom:\n  active: true\n\"}"

      # active: false
      ros2 service call /mola_runtime_param_set mola_msgs/srv/MolaRuntimeParamSet \
         "{parameters: \"mola::LidarOdometry:lidar_odom:\n  active: false\n\"}"

- ``mapping_enabled``: Whether MOLA-LO should update the localmap (``true``) or just use
  it in localization-only mode (``false``).

.. dropdown:: Copy & paste commands for ``mapping_enabled``

   .. code-block:: bash

      # mapping_enabled: true
      ros2 service call /mola_runtime_param_set mola_msgs/srv/MolaRuntimeParamSet \
         "{parameters: \"mola::LidarOdometry:lidar_odom:\n  mapping_enabled: true\n\"}"

      # mapping_enabled: false
      ros2 service call /mola_runtime_param_set mola_msgs/srv/MolaRuntimeParamSet \
         "{parameters: \"mola::LidarOdometry:lidar_odom:\n  mapping_enabled: false\n\"}"

- ``generate_simplemap``: Whether MOLA-LO should build the keyframes-based map (apart of the local metric map),
  so you end up with a ``*.simplemap`` file.

.. dropdown:: Copy & paste commands for ``generate_simplemap``

   .. code-block:: bash

      # generate_simplemap: true
      ros2 service call /mola_runtime_param_set mola_msgs/srv/MolaRuntimeParamSet \
         "{parameters: \"mola::LidarOdometry:lidar_odom:\n  generate_simplemap: true\n\"}"

      # generate_simplemap: false
      ros2 service call /mola_runtime_param_set mola_msgs/srv/MolaRuntimeParamSet \
         "{parameters: \"mola::LidarOdometry:lidar_odom:\n  generate_simplemap: false\n\"}"
