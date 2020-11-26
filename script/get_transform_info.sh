roslaunch realsense2_camera rs_camera.launch camera:=cam_1 serial_no:=036222070486 filters:=spatial,temporal,pointcloud &
sleep 5
roslaunch realsense2_camera rs_camera.launch camera:=cam_2 serial_no:=034422073314 filters:=spatial,temporal,pointcloud &
sleep 5
# rosrun rviz rviz -d /home/xujing/catkin_ws/src/pcl_filter/config/camera_transform_config.rviz &
sleep 1
# python ~/catkin_ws/src/realsense-ros/realsense2_camera/scripts/set_cams_transforms.py cam_1_link cam_2_link 0.1 0.0 0 0 0 0