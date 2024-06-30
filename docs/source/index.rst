.. MOLA documentation master file, created by
   sphinx-quickstart on Sat May  4 17:03:47 2019.

.. _index:

============
MOLA
============

.. toctree::
  :maxdepth: 2
  :hidden:
  :caption: Quickstart

  Home <index.html#http://>
  installing
  solutions
  use-cases

.. toctree::
  :maxdepth: 2
  :hidden:
  :caption: 3D LiDAR

  mola_lidar_odometry
  mola_lidar_odometry_cli
  mola_lo_apps
  mola_lo_pipelines
  mola_lo_demos
  mola_lo_ros

.. toctree::
  :maxdepth: 2
  :hidden:
  :caption: mp2p_icp

  module-mp2p-icp
  mp2p_icp_installing
  mp2p_icp_basics
  mp2p_icp_optimal-transformations
  mp2p_icp_applications
  mp2p_icp_demos

.. toctree::
  :maxdepth: 2
  :hidden:
  :caption: Details

  mola_architecture
  tutorials
  supported-sensors
  modules
  doxygen-index
  bibliography


:octicon:`mark-github` `MOLA`_ is a Modular system for Localization and Mapping.

Get started:
 - See :octicon:`rocket` :ref:`mola_lidar_odometry` for documentation of the LiDAR odometry module.
 - See :ref:`use-cases` for examples of use.
 - See :ref:`concepts` for an overview of the MOLA modular system.
 - See `videos`_ on YouTube.

.. image:: https://mrpt.github.io/imgs/mola-slam-kitti-demo.gif


.. _MOLA: https://github.com/MOLAorg/mola
.. _videos: https://www.youtube.com/playlist?list=PLOJ3GF0x2_eVaujK78PoVOvxJGrl_Z7fV


How to cite us
==============

The ``mola_lidar_odometry`` system was presented in :cite:`blanco2024mola_lo`:

  J.L. Blanco,
  `A flexible framework for accurate LiDAR odometry, map manipulation, and localization`_, in
  ArXiV, 2024.

.. _A flexible framework for accurate LiDAR odometry, map manipulation, and localization: https://TBD

The basics of the MOLA framework were introduced in :cite:`blanco2019modular`.

  J.L. Blanco,
  `A Modular Optimization Framework for Localization and Mapping`_, in
  *Robotics: Science and Systems (RSS)*, 2019.

.. _A Modular Optimization Framework for Localization and Mapping: https://ingmec.ual.es/~jlblanco/papers/blanco2019mola_rss2019.pdf


Indices and tables
==================

* :ref:`genindex`
