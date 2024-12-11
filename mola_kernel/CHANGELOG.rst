^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mola_kernel
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.0 (2024-12-11)
------------------
* NavStateFilter interface: add API for merging GNSS observations
* Contributors: Jose Luis Blanco-Claraco

1.2.1 (2024-09-29)
------------------

1.2.0 (2024-09-16)
------------------
* Update RTTI macros for upcoming MRPT 2.14.0
* Contributors: Jose Luis Blanco-Claraco

1.1.3 (2024-08-28)
------------------
* Depend on new mrpt_lib packages (deprecate mrpt2)
* Contributors: Jose Luis Blanco-Claraco

1.1.2 (2024-08-26)
------------------

1.1.1 (2024-08-23)
------------------

1.1.0 (2024-08-18)
------------------
* add <mola_kernel/version.h> with a version-checking macro
* Merge pull request `#65 <https://github.com/MOLAorg/mola/issues/65>`_ from MOLAorg/add-more-srvs
  Add more Services
* Avoid cmake file glob expressions
* mola_kernel: add MapServer interface
* mola_kernel: add public symbols MOLA\_{MAJOR,MINOR,PATCH}_VERSION
* Update clang-format style; add reformat bash script
* Merge pull request `#62 <https://github.com/MOLAorg/mola/issues/62>`_ from MOLAorg/docs-fixes
  Docs fixes
* Fix ament_xmllint warnings in package.xml
* Contributors: Jose Luis Blanco-Claraco

1.0.8 (2024-07-29)
------------------
* mola_kernel: add C++ virtual interface for relocalization methods
* ament_lint_cmake: clean warnings
* Contributors: Jose Luis Blanco-Claraco

1.0.7 (2024-07-24)
------------------
* Viz interface: add API for rotate camera
* Contributors: Jose Luis Blanco-Claraco

1.0.6 (2024-06-21)
------------------
* Create new NavStateFilter interface and separate the simple fuser and the factor-graph approach in two packages
* mola_kernel: renamed factor FactorConstVelKinematics
* Contributors: Jose Luis Blanco-Claraco

1.0.5 (2024-05-28)
------------------
* viz: fix mismatched free/delete inside nanogui layout
* Contributors: Jose Luis Blanco-Claraco

1.0.4 (2024-05-14)
------------------
* bump cmake_minimum_required to 3.5
* Avoid global static objects
* remove useless #include's
* Define Dataset_UI dtor/ctor in a separate translation unit
* Contributors: Jose Luis Blanco-Claraco

1.0.3 (2024-04-22)
------------------
* Fix package.xml website URL
* Contributors: Jose Luis Blanco-Claraco

1.0.2 (2024-04-04)
------------------

1.0.1 (2024-03-28)
------------------
* Remove now-useless build dependencies and includes for mola-kernel
* Contributors: Jose Luis Blanco-Claraco

1.0.0 (2024-03-19)
------------------
* add methods to query for subscribers
* New interfaces
* Refactor initialize()
* mola_kernel: new UI interface for datasets
* New option to shutdown automatically mola-cli after dataset ends
* viz API: add enqueue_custom_nanogui_code()
* mola_viz: show console messages
* Correct usage of mola:: namespace in cmake targets
* copyright update
* mola_viz: support visualizing velodyne observations
* Add look_at() viz interface
* Fewer mutex locking()
* dont force by default load() lazy-load observations
* FrontEndBase: attach to VizInterface too
* Fix loss of yaml key/values when using import-from-file feature
* kitti eval cli moves to its own package
* port to mrpt::lockHelper()
* reorganize as monorepo
* Contributors: Jose Luis Blanco-Claraco

0.2.2 (2023-09-08)
------------------
* Correct references to the license.
* viz interface: new service update_3d_object()
* Fix const-correctness of observations
* FIX missing dependency on mrpt::gui for public header
* Contributors: Jose Luis Blanco-Claraco

0.2.1 (2023-09-02)
------------------

* Add virtual interface for dataset groundtruth
* Update copyright date
* Update to new colcon ROS2 build system
* Contributors: Jose Luis Blanco-Claraco
