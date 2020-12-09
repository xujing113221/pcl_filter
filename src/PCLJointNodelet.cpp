#include <pluginlib/class_list_macros.h>

#include "pcl_filter/PCLJointNodelet.h"

#include <pcl/io/pcd_io.h>

PLUGINLIB_EXPORT_CLASS(pcl_filter::PCLJointNodelet, nodelet::Nodelet)

namespace pcl_filter
{
/***************************************************************************//**
 * Prepare the ROS subscriber and publishers
 * The "pub_Cluster" publisher will publish the clusters in the "CloudClusters" type for the next nodelet
 * The "pub_Cloud" publisher will publish the whole point cloud as "PointCloud2" for rviz visualization
 *
 ******************************************************************************/
void PCLJointNodelet::onInit(){
  NODELET_INFO("Initializing PCL Joint nodelet...");
  private_nh = getPrivateNodeHandle();
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cam1(private_nh, "/cam_1/depth/color/points", 10);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cam2(private_nh, "/cam_2/depth/color/points", 10);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_cam1, sub_cam2);
  sync.registerCallback(boost::bind(&PCLJointNodelet::pointCloudCallback, this,_1, _2));

  pub_Cloud = private_nh.advertise<sensor_msgs::PointCloud2>("jointedPointCloud",10);
  
  ros::spin();
}

/***************************************************************************//**
 * The callback function called when something was published to the subscribed topic
 *
 * The function handles the Jointing of the received point cloud
 *
 * @param left_input The message received from ROS as ConstPtr aka boost::shared_pointer
 * @param right_input The message received from ROS as ConstPtr aka boost::shared_pointer
 *
 ******************************************************************************/
static int32_t frame_cnt = 0;
void PCLJointNodelet::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& left_input, const sensor_msgs::PointCloud2::ConstPtr& right_input){

  pcl::PointCloud<pcl::PointXYZ>::Ptr left_pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*left_input, *left_pcl_pointcloud);

  // pcl::PointCloud<pcl::PointXYZI>::Ptr left_calibration_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  // pcl::transformPointCloud(*left_pcl_pointcloud, *left_calibration_cloud, g_left_calibration_matrix);

  // for(std::size_t i = 0; i < left_calibration_cloud->size(); ++i)
  // {
  // left_calibration_cloud->points[i].intensity = 64;
  // }

  pcl::PointCloud<pcl::PointXYZ>::Ptr right_pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*right_input, *right_pcl_pointcloud);

  // std::string filename_left("/home/xujing/catkin_ws/pointcloud_files_left/pcd_left_"+ std::to_string(frame_cnt) +".pcd");
  // std::string filename_right("/home/xujing/catkin_ws/pointcloud_files_right/pcd_right_"+ std::to_string(frame_cnt) +".pcd");
  // pcl::PCDWriter writer;
  // writer.write(filename_left,*left_pcl_pointcloud);
  // writer.write(filename_right,*right_pcl_pointcloud);
  // frame_cnt ++;

  // Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  // transform_2.translation() << 0.71875, -0.1375, 0.540625;
  // transform_2.rotate (Eigen::AngleAxisf (-20.625/180.0f*M_PI, Eigen::Vector3f::UnitZ()));
  // transform_2.rotate (Eigen::AngleAxisf (108.18750/180.0f*M_PI, Eigen::Vector3f::UnitY()));
  // transform_2.rotate (Eigen::AngleAxisf (-209.5/180.0f*M_PI, Eigen::Vector3f::UnitX()));
  // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  // pcl::transformPointCloud(*right_pcl_pointcloud, *transformed_cloud, transform_2);

  // joint pointclouds from left and right realsense camera 
  pcl::PointCloud<pcl::PointXYZ>::Ptr joint_pointcloud(new pcl::PointCloud<pcl::PointXYZ>());
  *joint_pointcloud = *left_pcl_pointcloud + *right_pcl_pointcloud;

  // pass through filter
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (joint_pointcloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.1, 1.0);  // 0.1~1.2m 
  // pass.setFilterLimitsNegative (true);//设置保留范围内 还是 过滤掉范围内
  pass.filter (*cloud_pass_filtered); 

  // downsampling
  float leaf_size = 0.002f;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> sor;    
  sor.setInputCloud(cloud_pass_filtered);            
  sor.setLeafSize(leaf_size,leaf_size,leaf_size);   
  // sor.setLeafSize(0.015f, 0.015f, 0.015f);
  sor.filter(*cloud_filtered);          


  // publish pointcloud
  sensor_msgs::PointCloud2 pub_pointcloud;
  pcl::toROSMsg(*cloud_filtered, pub_pointcloud);

  if(pub_Cloud.getNumSubscribers() > 0)
    pub_Cloud.publish(pub_pointcloud);

}
}