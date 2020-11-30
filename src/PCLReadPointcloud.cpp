#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/io/pcd_io.h>//which contains the required definitions to load and store point clouds to PCD and other file formats.

int main(int argc,char **argv){
    ros::init(argc,argv,"UandBdetect");
    ros::NodeHandle nh;
    ros::Publisher pcl_pub=nh.advertise<sensor_msgs::PointCloud2> ("pcl_output",1);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::PointCloud2 output;

    ros::Rate loop_rate(30);
    int frame_cnt = 0;
    while (ros::ok())
    {
        /* code for loop body */
        std::string filename("/home/xujing/Desktop/pointcloud_files/pcd_"+ std::to_string(frame_cnt) +".pcd");
        pcl::io::loadPCDFile(filename,cloud);
        pcl::toROSMsg(cloud,output);
        output.header.frame_id="odom";

        PCL_INFO("Publish new point cloud!\n");

        if (frame_cnt++ >7530) frame_cnt = 0;
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}