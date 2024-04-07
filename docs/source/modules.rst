===================
Core MOLA modules
===================

MOLA modular design means that SLAM or localization problems must be
split into their smallest possible reusable pieces.
We call these *core MOLA modules* and they are kept as colcon packages
in the `MOLAorg/mola <https://github.com/MOLAorg/mola>`_ repository.

Actual SLAM solutions, which make use of these core modules,
are listed in the :ref:`solutions` page.

Recall that each module has its own License, please refer to its
source code or the `<license>` tag of its `package.xml` file.

.. toctree::
  :maxdepth: 2

  module_mola_bridge_ros2
  module_mola_demos
  module_mola_imu_preintegration
  module_mola_input_euroc_dataset
  module_mola_input_kitti360_dataset
  module_mola_input_kitti_dataset
  module_mola_input_mulran_dataset
  module_mola_input_paris_luco_dataset
  module_mola_input_rawlog
  module_mola_input_rosbag2
  module_mola_kernel
  module_mola_launcher
  module_mola_metric_maps
  module_mola_navstate_fuse
  module_mola_pose_list
  module_mola_relocalization
  module_mola_traj_tools
  module_mola_viz
  module_mola_yaml
