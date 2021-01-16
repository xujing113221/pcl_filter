#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/io/pcd_io.h>//which contains the required definitions to load and store point clouds to PCD and other file formats.

int main(int argc,char **argv){
    ros::init(argc,argv,"UandBdetect");
    ros::NodeHandle nh;
    ros::Publisher pcl_pub_left=nh.advertise<sensor_msgs::PointCloud2> ("/cam_1/depth/color/points",1);
    ros::Publisher pcl_pub_right=nh.advertise<sensor_msgs::PointCloud2> ("/cam_2/depth/color/points",1);
    pcl::PointCloud<pcl::PointXYZ> cloud_left, cloud_right;
    sensor_msgs::PointCloud2 output_left;
    sensor_msgs::PointCloud2 output_right;
    
    PCL_INFO("Initializing Read Pointcloud nodelet...\n");
    ros::Rate loop_rate(30);
    int frame_cnt = 0;
    while (ros::ok())
    {
        /* code for loop body */
        std::string filename_left("/home/ning/catkin_ws/pointcloud_files_left/pcd_left_"+ std::to_string(frame_cnt) +".pcd");
        std::string filename_right("/home/ning/catkin_ws/pointcloud_files_right/pcd_right_"+ std::to_string(frame_cnt) +".pcd");

        pcl::io::loadPCDFile(filename_left,cloud_left);
        pcl::toROSMsg(cloud_left,output_left);
        output_left.header.frame_id="cam_1_link";

        pcl::io::loadPCDFile(filename_right,cloud_right);
        pcl::toROSMsg(cloud_right,output_right);
        output_right.header.frame_id="cam_2_link";

        PCL_INFO("Publish new point cloud!\n");

        if (frame_cnt++ >2699) frame_cnt = 0;
        pcl_pub_left.publish(output_left);
        pcl_pub_right.publish(output_right);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}