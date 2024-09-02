.. _wrappers_3rd_party:

======================
3rd-party wrappers
======================
For the sake of scientific comparison between different methods, we provide wrappers of
other LiDAR odometry methods that mimic the interface of :ref:`mola-lidar-odometry-cli <mola_lidar_odometry_cli>`,
so exactly the same input datasets can be processed by different methods.

____________________________________________

|

KISS-ICP
--------------------------------------
Wrapper for the work :cite:`vizzo2023kiss`.

Repository: https://github.com/MOLAorg/mola_kiss_icp_wrapper

.. dropdown:: Compile instructions
   :icon: code-square

   Clone in your ROS 2 workspace:

   .. code-block:: bash

      mkdir -p ~/ros2_mola_ws/src/
      cd ~/ros2_mola_ws/src/

      git clone https://github.com/MOLAorg/mola_kiss_icp_wrapper.git --recursive

   Install dependencies:

   .. code-block:: bash

      cd ~/ros2_mola_ws/
      rosdep install --from-paths src --ignore-src -r -y

   Compile:

   .. code-block:: bash

      cd ~/ros2_mola_ws/
      colcon  build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

.. dropdown:: CLI reference
   :icon: checklist

   .. code-block:: bash

      USAGE:

         mola-lidar-odometry-cli-kiss  [--input-mulran-seq <KAIST01>]
                                       [--kitti-correction-angle-deg <0.205
                                       [degrees]>] [--input-kitti-seq <00>]
                                       [--lidar-sensor-label <>] [--input-rosbag2
                                       <dataset.mcap>] [--input-rawlog
                                       <dataset.rawlog>] [--only-first-n <Number
                                       of dataset entries to run>] [--no-deskew]
                                       [--output-tum-path
                                       <output-trajectory.txt>] [--max-range
                                       <max-range>] [--min-range <min-range>]
                                       [--] [--version] [-h]


      Where: 

         --input-mulran-seq <KAIST01>
         INPUT DATASET: Use Mulran dataset sequence KAIST01|KAIST01|...

         --kitti-correction-angle-deg <0.205 [degrees]>
         Correction vertical angle offset (see Deschaud,2018)

         --input-kitti-seq <00>
         INPUT DATASET: Use KITTI dataset sequence number 00|01|...

         --lidar-sensor-label <>
         If provided, this supersedes the values in the 'lidar_sensor_labels'
         entry of the odometry pipeline, defining the sensorLabel/topic name to
         read LIDAR data from. It can be a regular expression (std::regex)

         --input-rosbag2 <dataset.mcap>
         INPUT DATASET: rosbag2. Input dataset in rosbag2 format (*.mcap)

         --input-rawlog <dataset.rawlog>
         INPUT DATASET: rawlog. Input dataset in rawlog format (*.rawlog)

         --only-first-n <Number of dataset entries to run>
         Run for the first N steps only (0=default, not used)

         --no-deskew
         Skip scan de-skew

         --output-tum-path <output-trajectory.txt>
         Save the estimated path as a TXT file using the TUM file format (see
         evo docs)

         --max-range <max-range>
         max-range parameter

         --min-range <min-range>
         min-range parameter

         --,  --ignore_rest
         Ignores the rest of the labeled arguments following this flag.

         --version
         Displays version information and exits.

         -h,  --help
         Displays usage information and exits.


         mola-lidar-odometry-cli-kiss


|


SiMpLE
--------------------------------------
Wrapper for the work :cite:`bhandari2024minimal`.

Repository: https://github.com/MOLAorg/mola_simple_wrapper

.. dropdown:: Compile instructions
   :icon: code-square

   Clone in your ROS 2 workspace:

   .. code-block:: bash

      mkdir -p ~/ros2_mola_ws/src/
      cd ~/ros2_mola_ws/src/

      git clone https://github.com/MOLAorg/mola_simple_wrapper.git --recursive

   Install dependencies:

   .. code-block:: bash

      cd ~/ros2_mola_ws/
      rosdep install --from-paths src --ignore-src -r -y

   Compile:

   .. code-block:: bash

      cd ~/ros2_mola_ws/
      colcon  build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

.. dropdown:: CLI reference
   :icon: checklist

   .. code-block:: bash

      USAGE: 

         mola-lidar-odometry-cli-simple  [--input-paris-luco] [--input-mulran-seq
                                       <KAIST01>] [--input-kitti360-seq <00>]
                                       [--kitti-correction-angle-deg <0.205
                                       [degrees]>] [--input-kitti-seq <00>]
                                       [--lidar-sensor-label <lidar1>]
                                       [--input-rosbag2 <dataset.mcap>]
                                       [--input-rawlog <dataset.rawlog>]
                                       [--only-first-n <Number of dataset
                                       entries to run>] [--output-tum-path
                                       <output-trajectory.txt>] -c
                                       <config.yaml> [--] [--version] [-h]


      Where: 

         --input-paris-luco
         INPUT DATASET: Use Paris Luco dataset (unique sequence=00)

         --input-mulran-seq <KAIST01>
         INPUT DATASET: Use Mulran dataset sequence KAIST01|KAIST01|...

         --input-kitti360-seq <00>
         INPUT DATASET: Use KITTI360 dataset sequence number 00|01|...

         --kitti-correction-angle-deg <0.205 [degrees]>
         Correction vertical angle offset (see Deschaud,2018)

         --input-kitti-seq <00>
         INPUT DATASET: Use KITTI dataset sequence number 00|01|...

         --lidar-sensor-label <lidar1>
         If provided, this supersedes the values in the 'lidar_sensor_labels'
         entry of the odometry pipeline, defining the sensorLabel/topic name to
         read LIDAR data from. It can be a regular expression (std::regex)

         --input-rosbag2 <dataset.mcap>
         INPUT DATASET: rosbag2. Input dataset in rosbag2 format (*.mcap)

         --input-rawlog <dataset.rawlog>
         INPUT DATASET: rawlog. Input dataset in rawlog format (*.rawlog)

         --only-first-n <Number of dataset entries to run>
         Run for the first N steps only (0=default, not used)

         --output-tum-path <output-trajectory.txt>
         Save the estimated path as a TXT file using the TUM file format (see
         evo docs)

         -c <config.yaml>,  --config-file <config.yaml>
         (required)  Simple config file

         --,  --ignore_rest
         Ignores the rest of the labeled arguments following this flag.

         --version
         Displays version information and exits.

         -h,  --help
         Displays usage information and exits.


         mola-lidar-odometry-cli-simple

