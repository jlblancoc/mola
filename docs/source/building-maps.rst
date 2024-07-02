.. _building-maps:

====================
How to build a map
====================

This page describes the steps for building metric maps using MOLA.


.. contents::
   :depth: 1
   :local:
   :backlinks: none


1. Prepare the input data
---------------------------------
- Think what sensors do you want to use:
  - At least, one **2D or 3D LiDAR**. It is possible to have multiple LiDARs.
  - Optional: Encoder-based odometry, for wheeled robots.
  - Optional: Low-cost GNNS (GPS) receiver, for georeferencing the final metric maps. 
    You can use `mrpt_sensor_gnns_nmea <https://github.com/mrpt-ros-pkg/mrpt_sensors?tab=readme-ov-file#mrpt_sensor_gnns_nmea>`_ for 
    standard GNNS USB devices providing NMEA messages.

- Decide whether SLAM will run **online** (live) or **ofline** (in postprocessing).

- If it will be run **offline** (which is normally **recommended**): Grab the data using `ros2 bag record <https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html>`_
  if you use ROS 2. Alternative options include using :ref:`MOLA-based dataset sources <supported_sensors>`.

- If using ROS 2...
  - ...and you only have **one LiDAR** as the **unique sensor**,
  then setting up `ROS tf frames <https://www.google.com/search?q=ROS+tf+frames+tutorials>`_
  is not mandatory, if you are happy with tracking the **sensor** pose (vs the vehicle pose).
  - ...and you have odometry or more sensors on a real robot, then having correct `/tf` messages
  published is important to let the mapping algorithm what is the relative pose of the sensor within
  the vehicle.


2. Run MOLA-LO
---------------------------------
The output of running LiDAR odometry on your dataset is a **simple-map**:

.. figure:: imgs/odometry_inputs_outputs.png
   :width: 400

   Role of an "odometry" module (Figure adapted from :cite:`blanco2024mola_lo`).


To process an offline dataset, use any of the available options in :ref:`MOLA-LO applications <_launching_mola_lo>`:
  - :ref:`mola-lo-gui-rosbag2 <mola_lo_gui_rosbag2>` for a version with GUI, or
  - :ref:`mola-lidar-odometry-cli <mola_lidar_odometry_cli>` for a terminal (CLI) version.

To launch SLAM live on a robot, read :ref:`mola_lo_ros`.

In any case above, make sure to enable the option of **generating and saving the output simple-map** and
take note of where is the generated file.

.. hint::

    To help you getting familiar with the whole process, feel free of downloading any of these example simple-maps
    so you can use the following steps before building your own maps:
    
    - warehouse.simplemap : A map of a (simulated) warehouse, built from a wheeled robot with a 3D LiDAR.


3. Inspect the resulting simple-map
----------------------------------------
To verify that the generated simple-map is correct, you can use :ref:`sm-cli <app_sm-cli>`.
For example: 


.. code-block:: bash

    sm-cli info YOUR_FILE.simplemap


4. Build metric maps
----------------------------------------
Generating metric maps from a simple-maps is done with mp2p_icp filtering pipelines.
It can be done directly from C++ if so desired, or easily from the command 
line with :ref:`sm2mm <app_sm2mm>`.


5. Visualize the maps
----------------------------------------

Visualizing metric map files (``*.mm``) can be done with :ref:`mm-viewer <app_mm-viewer>`.


6. What's next?
----------------------------------------

Write me: 
- georeferencing
- loop closure
- Use for localization

