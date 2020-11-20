### Setup

1. roscore
2. roslaunch realsense2_camera rs_camera.launch filters:=pointcloud
3. rviz
4. roslaunch pcl_filter pclfilter.launch

    realsense-viewer



### Realsense Camera 

    Realsense SR300

[GitHub: IntelRealSense/realsense-ros](https://github.com/IntelRealSense/realsense-ros)


### Task 1: Filter

    Topic: sensor_msgs/PointCloud2 (/camera/depth/color/points from Camera)
    
### camera_calibration

+ [ROS下采用camera_calibration进行双目相机标定](https://blog.csdn.net/lemonxiaoxiao/article/details/109392102)

+ [Ubuntu16.04下intel RealSense D435i imu+双目标定](https://www.codenong.com/cs109369181/)

    roslaunch realsense2_camera rs_camera.launch

    rosrun camera_calibration cameracalibrator.py  --size 7x6 --square 0.0178 --approximate 0.01 left:=/left/color/image_raw right:=/right/color/image_raw right_camera:=/right left_camera:=/left --no-service-check

    rosrun camera_calibration cameracalibrator.py --size 7x6 --square 0.0178 --approximate 0.01 left:=/left/infra1/image_rect_raw right:=/right/infra1/image_rect_raw right_camera:=/right left_camera:=/left --no-service-check

    rosrun camera_calibration cameracalibrator.py --size 7x6 --square 0.0178 image:=/usb_cam/image_raw camera:=/usb_cam

    rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.0178 image:=/left/color/image_raw camera:=/left

+ [How-to: Multiple camera setup with ROS](https://www.intelrealsense.com/how-to-multiple-camera-setup-with-ros/)
**serial number**:
    camera right : 618206002406
    camera left: 617205002698


    Terminal 1:
        roslaunch realsense2_camera rs_camera.launch camera:=cam_1 serial_no:=617205002698 filters:=spatial,temporal,pointcloud
    Terminal 2:
        roslaunch realsense2_camera rs_camera.launch camera:=cam_2 serial_no:=618206002406 filters:=spatial,temporal,pointcloud
    Terminal 3:
        rviz
    Terminal 4:
        cd catkin_ws
        python src/realsense-ros/realsense2_camera/scripts/set_cams_transforms.py cam_1_link cam_2_link 0.1 0.0 0 0 0 0

### Joint Pointcloud

[ROS下多雷达融合算法](https://www.cnblogs.com/kuangxionghui/p/12059973.html)


### fix decimation and temporal of the realsense
    
[Intel Realsense后处理过滤器](https://blog.csdn.net/dontla/article/details/103574458)
    rosrun rqt_reconfigure rqt_reconfigure

### others

install ros:
    sudo apt install -y ros-melodic-desktop-full






**camera calibration exsample:**

Left:
D = [0.12400851736650999, -0.26370453697067747, 0.00031186952404282985, 0.001740457631538854, 0.0]
K = [602.8786091516981, 0.0, 329.4897924779055, 0.0, 602.2668954077808, 236.97478719057725, 0.0, 0.0, 1.0]
R = [0.99913015092875, -0.013416612565871766, -0.039483363741587535, 0.011921904261969296, 0.9992122509151734, -0.03785162902439986, 0.0399601013995417, 0.037347986937579965, 0.9985030386372635]
P = [702.9001952215381, 0.0, 340.8315124511719, 0.0, 0.0, 702.9001952215381, 237.12061309814453, 0.0, 0.0, 0.0, 1.0, 0.0]

Right:
D = [0.1294161637580652, -0.28028693539898486, 0.0005459685418843547, -0.0022221146187633074, 0.0]
K = [603.885013954261, 0.0, 318.7130427349341, 0.0, 604.4369022754347, 236.6438915390346, 0.0, 0.0, 1.0]
R = [0.9998369870109509, -0.01774762798102017, -0.0033198051013400764, 0.017859928024862408, 0.9991340130491063, 0.03757987412611886, 0.0026499765678788846, -0.037633039598675984, 0.9992881125855316]
P = [702.9001952215381, 0.0, 340.8315124511719, 102.21417365635853, 0.0, 702.9001952215381, 237.12061309814453, 0.0, 0.0, 0.0, 1.0, 0.0]
self.T = [0.14539405752501933, -0.0025808203508445784, -0.00048275862980317724]
self.R = [0.9992860974207949, 0.004530404689393452, -0.03750694515510294, -0.00732443025319179, 0.9971795409219394, -0.0746948183489074, 0.037062760595828616, 0.0749162105292337, 0.9965008345089109]
# oST version 5.0 parameters

[image]

width
640

height
480

[narrow_stereo/left]

camera matrix
602.878609 0.000000 329.489792
0.000000 602.266895 236.974787
0.000000 0.000000 1.000000

distortion
0.124009 -0.263705 0.000312 0.001740 0.000000

rectification
0.999130 -0.013417 -0.039483
0.011922 0.999212 -0.037852
0.039960 0.037348 0.998503

projection
702.900195 0.000000 340.831512 0.000000
0.000000 702.900195 237.120613 0.000000
0.000000 0.000000 1.000000 0.000000
# oST version 5.0 parameters


[image]

width
640

height
480

[narrow_stereo/right]

camera matrix
603.885014 0.000000 318.713043
0.000000 604.436902 236.643892
0.000000 0.000000 1.000000

distortion
0.129416 -0.280287 0.000546 -0.002222 0.000000

rectification
0.999837 -0.017748 -0.003320
0.017860 0.999134 0.037580
0.002650 -0.037633 0.999288

projection
702.900195 0.000000 340.831512 102.214174
0.000000 702.900195 237.120613 0.000000
0.000000 0.000000 1.000000 0.000000
