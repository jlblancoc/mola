set -x
set -e

# Better to hot-link?
# if [ ! -d html/videos/ ]; then
#     mkdir html/videos
#     (cd html/videos/ && wget https://mrpt.github.io/videos/mp2p_icp-log-viewer-demo.mp4)
#     (cd html/videos/ && wget https://mrpt.github.io/videos/mola_main_page_video.mp4);
#     (cd html/videos/ && wget https://mrpt.github.io/videos/mola-lo-gui-kitti_demo_00.mp4);
# fi

cp -v ../../mola_lidar_odometry/docs/imgs/* html/imgs/
cp -v ../../mola_state_estimation/docs/imgs/* html/imgs/
