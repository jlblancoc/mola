.. _geo-referencing:

======================
Georeferencing
======================
Georeferencing trajectories and metric maps is implemented in the :ref:`mola_sm_loop_closure <mola_licenses>` package.

The concept of using simple-maps as intermediary map format together with the layered metric map format (see :cite:`blanco2024mola_lo`)
enables embedding georeferenced coordinates to any kind of map typically used in robotics: grid maps, voxel maps, point clouds, etc.

|

.. image:: https://mrpt.github.io/imgs/kaist01_georef_sample.png
   :width: 250
   :align: right

.. contents:: Table of Contents
   :depth: 1
   :local:
   :backlinks: none

|

1. Frames for geo-referenced maps
--------------------------------------------
When working with ROS ``tf`` (transformations), MOLA packages use the following frame convention, 
which extends the standard `REP-105 <https://www.ros.org/reps/rep-0105.html>`_ with additional
``enu`` and ``utm`` frames:

.. figure:: https://mrpt.github.io/imgs/mola_mrpt_ros_geo_referenced_utm_frames.png
   :width: 500
   :align: center

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

2. How to build a georeferenced map
--------------------------------------------
First, build a simple-map from a dataset or a live robot as described in :ref:`building-maps`.
Make sure of having a GPS (GNSS) sensor source emitting observations, and that they were captured
by MOLA-LO (see :ref:`the corresponding variable <mola_lo_pipeline_sensor_inputs>` in the LO pipeline).

Then, build the corresponding metric map by applying a metric map generation pipeline (see :cite:`blanco2024mola_lo`
or :ref:`this step  <building-maps_step_mm>` in the tutorial: 

.. code-block:: bash

      # Build metric map (mm) from simplemap (sm):
      sm2mm -i datasetWithGPS.simplemap -o myMap.mm -p sm2mm_pipeline.yaml

Now, to find out the optimized map-to-ENU transformation and write it into the
map file, use:

.. code-block:: bash

      # georeference it:
      mola-sm-georeferencing -i datasetWithGPS.simplemap --write-into myMap.mm

Alternatively, the georeferenciation metadata can be also stored, independently of a metric map,
in an independent file with:

.. code-block:: bash

      # georeference it:
      mola-sm-georeferencing -i datasetWithGPS.simplemap --output myMap.georef



.. dropdown:: Full CLI reference
   :icon: code-review

   .. code-block:: bash

      USAGE:

         mola-sm-georeferencing  [-v <INFO>] [-l <foobar.so>]
                                 [--horizontality-sigma <1.0>] [-o <map.georef>]
                                 [--write-into <map.mm>] -i <map.simplemap> [--]
                                 [--version] [-h]


      Where: 

         -v <INFO>,  --verbosity <INFO>
         Verbosity level: ERROR|WARN|INFO|DEBUG (Default: INFO)

         -l <foobar.so>,  --load-plugins <foobar.so>
         One or more (comma separated) *.so files to load as plugins, e.g.
         defining new CMetricMap classes

         --horizontality-sigma <1.0>
         For short trajectories (not >10x the GPS uncertainty), this helps to
         avoid degeneracy.

         -o <map.georef>,  --output <map.georef>
         Write the obtained georeferencing metadata to a .georef file

         --write-into <map.mm>
         An existing .mm file in which to write the georeferencing metadata

         -i <map.simplemap>,  --input <map.simplemap>
         (required)  Input .simplemap file

         --,  --ignore_rest
         Ignores the rest of the labeled arguments following this flag.

         --version
         Displays version information and exits.

         -h,  --help
         Displays usage information and exits.


|

3. Georeferenced trajectories
--------------------------------------
Once you already have a **trajectory** file in the **local map frame of reference**,
for example, as generated by MOLA-LO in TUM format,
and after georeferencing the generated map as shown above,
you can use the CLI tool ``mola-trajectory-georef`` to convert it into geodetic coordinates,
for example in KML format suitable for visualization in Google Earth.

.. dropdown:: Full CLI reference
   :icon: code-review

   .. code-block:: bash

      USAGE:

         mola-trajectory-georef  -o <path.kml> -t <traj.tum> [-g <map.georef>]
                                 [-m <map.mm>] [--] [--version] [-h]


      Where: 

         -o <path.kml>,  --output <path.kml>
         (required)  The name of the google earth kml file to write to

         -t <traj.tum>,  --trajectory <traj.tum>
         (required)  Input .tum trajectory, in map local coordinates

         -g <map.georef>,  --geo-ref <map.georef>
         Input .georef file with georef info

         -m <map.mm>,  --map <map.mm>
         Input .mm map with georef info

         --,  --ignore_rest
         Ignores the rest of the labeled arguments following this flag.

         --version
         Displays version information and exits.

         -h,  --help
         Displays usage information and exits.


|

4. Georeferenced maps in mm-viewer
----------------------------------------
Write me!

