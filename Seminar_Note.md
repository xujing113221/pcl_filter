### Setup

1. roscore
2. roslaunch realsense2_camera rs_camera.launch filters:=pointcloud
3. rviz
4. roslaunch pcl_filter pclfilter.launch

    realsense-viewer

run plugin:
    rosrun nodelet nodelet load pcl_filter/PCLFilterNodelet /manager/nodelet_manager __name:=PCLFilterNodelet /camera/depth/color/points:=/PCLNodelets/PCLJointNodelet/jointedPointCloud


### Realsense Camera 

    Realsense SR300

[GitHub: IntelRealSense/realsense-ros](https://github.com/IntelRealSense/realsense-ros)

### Task 1: Filter

    Topic: sensor_msgs/PointCloud2 (/camera/depth/color/points from Camera)
    
### camera_calibration

+ [ROS下采用camera_calibration进行双目相机标定](https://blog.csdn.net/lemonxiaoxiao/article/details/109392102)

+ [Ubuntu16.04下intel RealSense D435i imu+双目标定](https://www.codenong.com/cs109369181/)
```bash
roslaunch realsense2_camera rs_camera.launch

rosrun camera_calibration cameracalibrator.py  --size 7x6 --square 0.0178 --approximate 0.01 left:=/left/color/image_raw right:=/right/color/image_raw right_camera:=/right left_camera:=/left --no-service-check

rosrun camera_calibration cameracalibrator.py --size 7x6 --square 0.0178 --approximate 0.01 left:=/left/infra1/image_rect_raw right:=/right/infra1/image_rect_raw right_camera:=/right left_camera:=/left --no-service-check

rosrun camera_calibration cameracalibrator.py --size 7x6 --square 0.0178 image:=/usb_cam/image_raw camera:=/usb_cam

rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.0178 image:=/left/color/image_raw camera:=/left
```

+ [How-to: Multiple camera setup with ROS](https://www.intelrealsense.com/how-to-multiple-camera-setup-with-ros/)
**serial number**:
    camera right : 618206002406
    camera left: 617205002698

```bash
Terminal 1:
    roslaunch realsense2_camera rs_camera.launch camera:=cam_1 serial_no:=036222070486 filters:=spatial,temporal,pointcloud
Terminal 2:
    roslaunch realsense2_camera rs_camera.launch camera:=cam_2 serial_no:=034422073314 filters:=spatial,temporal,pointcloud
Terminal 3:
    rviz
Terminal 4:
    cd catkin_ws
    python src/realsense-ros/realsense2_camera/scripts/set_cams_transforms.py cam_1_link cam_2_link 0.1 0.0 0 0 0 0
Output:
    ~/catkin_ws/src/realsense-ros/realsense2_camera/scripts/_set_cams_info_file.txt
```
or directly use script:
```bash
roslaunch pcl_filter set_transform.launch
sh ~/catkin_ws/src/pcl_filter/script/get_transform_info.sh
```

### Joint Pointcloud

[ROS下多雷达融合算法](https://www.cnblogs.com/kuangxionghui/p/12059973.html)


### fix decimation and temporal of the realsense
    
[Intel Realsense后处理过滤器](https://blog.csdn.net/dontla/article/details/103574458)
    rosrun rqt_reconfigure rqt_reconfigure



### PointCloud filter

    limit pointcloud: from[基于ROS系统的3D点云单目标行人跟踪](https://www.jianshu.com/p/acb6b85cd0be)
```cpp
    pcl::PassThrough<pcl::PointXYZ> pass;

    pass.setInputCloud (cloud);            //设置输入点云
    pass.setFilterFieldName ("x");         //设置过滤时所需要点云类型的Z字段
    pass.setFilterLimits (-7.0, 7.0);        //设置在过滤字段的范围
    pass.filter (*cloud);
    
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("y");         //设置过滤时所需要点云类型的Z字段
    pass.setFilterLimits (-7.0, 7.0);        //设置在过滤字段的范围
    pass.filter (*cloud);
    
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");         //设置过滤时所需要点云类型的Z字段
    pass.setFilterLimits (-1.5, 0.5);        //设置在过滤字段的范围
    pass.filter (*cloud);
```

[Filtering 点云滤波](https://zhuanlan.zhihu.com/p/95983353)


### Pointcloud tracking 

Plan A:
    track the center of point cluster
Plan B:
    optical flow
Plan C:
    karlmann filter



### others

install ros:
    sudo apt install -y ros-melodic-desktop-full


##calibration

x:0.71875 + 0.50625
y:-0.1375 + 0.60625
z:0.5406250
a:-20.625 - 84.0
p:108.18750
r:-209.5