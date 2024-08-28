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


2. Localization with LiDAR odometry
--------------------------------------
Write me!


|


3. Localization with particle filtering
----------------------------------------
Write me!
