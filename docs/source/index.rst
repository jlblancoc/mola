.. MOLA documentation master file, created by
   sphinx-quickstart on Sat May  4 17:03:47 2019.

.. _index:

MOLA
============

.. toctree::
  :maxdepth: 2
  :hidden:
  :caption: Quickstart

  Home <index.html#http://>
  building-maps
  localization
  geo-referencing
  use-cases
  ros2api
  solutions

.. toctree::
  :maxdepth: 2
  :hidden:
  :caption: 3D LiDAR

  mola_lidar_odometry
  mola_lo_apps
  mola_lo_pipelines
  wrappers_3rd_party

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
  :caption: Learn more

  mola_architecture
  tutorials
  supported-sensors
  dataset-conversions
  modules
  doxygen-index
  bibliography


:octicon:`mark-github` `MOLA`_ is a Modular system for Localization and Mapping.

Get started:
 - Read: :ref:`building-maps`.
 - Read :octicon:`rocket` :ref:`mola_lidar_odometry` documentation.
 - See :ref:`use-cases` for examples of use.
 - See :ref:`installing` and :ref:`how to cite it <citing_mola>`.
 - See `videos`_ on YouTube.

.. image:: https://mrpt.github.io/imgs/mola-slam-kitti-demo.gif


.. _MOLA: https://github.com/MOLAorg/mola
.. _videos: https://www.youtube.com/playlist?list=PLOJ3GF0x2_eVaujK78PoVOvxJGrl_Z7fV

|

.. humble badges ------

.. |badgeHrel| image:: https://img.shields.io/ros/v/humble/mola
   :scale: 100%
   :align: middle
   :target: https://index.ros.org/search/?term=mola

.. |badgeHrel_LO| image:: https://img.shields.io/ros/v/humble/mola_lidar_odometry
   :scale: 100%
   :align: middle
   :target: https://index.ros.org/search/?term=mola_lidar_odometry

.. |badgeHrel_MP| image:: https://img.shields.io/ros/v/humble/mp2p_icp
   :scale: 100%
   :align: middle
   :target: https://index.ros.org/search/?term=mp2p_icp

.. iron badges ------

.. |badgeIrel| image:: https://img.shields.io/ros/v/iron/mola
   :scale: 100%
   :align: middle
   :target: https://index.ros.org/search/?term=mola

.. |badgeIrel_LO| image:: https://img.shields.io/ros/v/iron/mola_lidar_odometry
   :scale: 100%
   :align: middle
   :target: https://index.ros.org/search/?term=mola_lidar_odometry

.. |badgeIrel_MP| image:: https://img.shields.io/ros/v/iron/mp2p_icp
   :scale: 100%
   :align: middle
   :target: https://index.ros.org/search/?term=mp2p_icp

.. jazzy badges ------

.. |badgeJrel| image:: https://img.shields.io/ros/v/jazzy/mola
   :scale: 100%
   :align: middle
   :target: https://index.ros.org/search/?term=mola

.. |badgeJrel_LO| image:: https://img.shields.io/ros/v/jazzy/mola_lidar_odometry
   :scale: 100%
   :align: middle
   :target: https://index.ros.org/search/?term=mola_lidar_odometry

.. |badgeJrel_MP| image:: https://img.shields.io/ros/v/jazzy/mp2p_icp
   :scale: 100%
   :align: middle
   :target: https://index.ros.org/search/?term=mp2p_icp


.. rolling badges ------

.. |badgeRrel| image:: https://img.shields.io/ros/v/rolling/mola
   :scale: 100%
   :align: middle
   :target: https://index.ros.org/search/?term=mola

.. |badgeRrel_LO| image:: https://img.shields.io/ros/v/rolling/mola_lidar_odometry
   :scale: 100%
   :align: middle
   :target: https://index.ros.org/search/?term=mola_lidar_odometry

.. |badgeRrel_MP| image:: https://img.shields.io/ros/v/rolling/mp2p_icp
   :scale: 100%
   :align: middle
   :target: https://index.ros.org/search/?term=mp2p_icp


.. _installing:

Installing
======================

How to install all MOLA modules:

.. dropdown:: From ROS 2 repositories
    :open:
    :icon: download

    **Recommended**: This is the easiest way to install MOLA.

    In Debian/Ubuntu systems, activate your ROS environment (``setup.bash``) if not done automatically 
    in your ``~./bashrc`` file, then just run:

    .. code-block:: bash

        # Install core MOLA modules and 3D LiDAR odometry:
        sudo apt install \
         ros-$ROS_DISTRO-mola \
         ros-$ROS_DISTRO-mola-lidar-odometry

        # (OPTIONAL) Install example small datasets to run demos/unit tests:
        sudo apt install ros-$ROS_DISTRO-mola-test-datasets

    Check if all new nodes and apps are visible:

    .. code-block:: bash

        # For example, let's launch the mm map viewer. 
        # If a GUI app is opened, it means installation was successful.
        mm-viewer

        # You can also test the mola LO cli interface:
        mola-lidar-odometry-cli --help


    These are the **versions available** from ROS build farms for each main MOLA component:

    +-------------------------+---------------+---------------------+---------------------+
    | ROS distribution        |      MOLA     | mola_lidar_odometry |   mp2p_icp          |
    +=========================+===============+=====================+=====================+
    | ROS 2 Humble (u22.04)   |  |badgeHrel|  |   |badgeHrel_LO|    |   |badgeHrel_MP|    |
    +-------------------------+---------------+---------------------+---------------------+
    | ROS 2 Iron (u22.04)     |  |badgeIrel|  |   |badgeIrel_LO|    |   |badgeIrel_MP|    |
    +-------------------------+---------------+---------------------+---------------------+
    | ROS 2 Jazzy (u24.04)    |  |badgeJrel|  |   |badgeJrel_LO|    |   |badgeJrel_MP|    |
    +-------------------------+---------------+---------------------+---------------------+
    | ROS 2 Rolling (u24.04)  |  |badgeRrel|  |   |badgeRrel_LO|    |   |badgeRrel_MP|    |
    +-------------------------+---------------+---------------------+---------------------+


.. dropdown:: Build from sources
    :icon: code-square

    Building tools
    ~~~~~~~~~~~~~~~~~
    MOLA uses ``colcon`` so you need to `install it first <https://colcon.readthedocs.io/en/released/user/installation.html>`_.

    Note that despite ROS 2 integration, a full ROS 2 installation is actually not **required** for MOLA, only ``colcon`` and ``ament``.


    Get the sources
    ~~~~~~~~~~~~~~~~~

    Clone the git repositories, including the submodules:

    .. code-block:: bash

        mkdir -p ~/ros2_mola_ws/src/ 
        cd ~/ros2_mola_ws/src/

        # Main MOLA modules:
        git clone https://github.com/MOLAorg/mola_common.git
        git clone https://github.com/MOLAorg/mp2p_icp.git --recursive
        git clone https://github.com/MOLAorg/mola.git --recursive
        git clone https://github.com/MOLAorg/mola_test_datasets.git

        # MOLA lidar odometry package:
        git clone https://github.com/MOLAorg/mola_lidar_odometry.git --recursive

    Dependencies
    ~~~~~~~~~~~~~~~~~

    Make sure you have all dependencies installed (make sure of having `rosdep already installed <https://wiki.ros.org/rosdep>`_):

    .. code-block:: bash

        cd ~/ros2_mola_ws/
        rosdep install --from-paths src --ignore-src -r -y


    Build and test
    ~~~~~~~~~~~~~~~~~

    Now, compile as usual with colcon:

    .. code-block:: bash

        cd ~/ros2_mola_ws/
        colcon  build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo


    Next, activate the new environment and check if all new modules are visible:

    .. code-block:: bash

        cd ~/ros2_mola_ws/
        . install/setup.bash

        # For example, let's launch the mm map viewer:
        mm-viewer

|

.. _citing_mola:

How to cite MOLA
==================

The ``mola_lidar_odometry`` system was presented in :cite:`blanco2024mola_lo`:

  J.L. Blanco,
  `A flexible framework for accurate LiDAR odometry, map manipulation, and localization`_, in
  ArXiV, 2024.

.. _A flexible framework for accurate LiDAR odometry, map manipulation, and localization: https://arxiv.org/abs/2407.20465

The basics of the MOLA framework were introduced in :cite:`blanco2019modular`.

  J.L. Blanco,
  `A Modular Optimization Framework for Localization and Mapping`_, in
  *Robotics: Science and Systems (RSS)*, 2019.

.. _A Modular Optimization Framework for Localization and Mapping: https://ingmec.ual.es/~jlblanco/papers/blanco2019mola_rss2019.pdf

