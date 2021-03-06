#include <pluginlib/class_list_macros.h>

#include "pcl_filter/PCLJointNodelet.h"

#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

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

  float trans_info[6] = {0.6531,0.4375, 0, -103.5,3.0,0.0};  //x,y,z,a,p,r :degrees
  tr_matrix = get_transform_matrix(trans_info);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_cam1, sub_cam2);
  sync.registerCallback(boost::bind(&PCLJointNodelet::pointCloudCallback, this,_1, _2));

  pub_Cloud = private_nh.advertise<sensor_msgs::PointCloud2>("jointedPointCloud",10);
  
  ros::spin();
}

/***************************************************************************//**
 * Get transform matrix from the information(x,y,z,yaw,pitch,roll), convert 
 * the point cloud of camera2 into point cloud of camera1
 * @param tr_info (x,y,z,yaw,pitch,roll) unit: meter and degrees
 * @return the transform matrix
 ******************************************************************************/
Eigen::Matrix4f PCLJointNodelet::get_transform_matrix(const float tr_info[]){
  float x = tr_info[0];
  float y = tr_info[1];
  float z = tr_info[2];
  float r_z = tr_info[3]/180.0f*M_PI;   //yaw
  float r_y = tr_info[4]/180.0f*M_PI;   //pitch
  float r_x = tr_info[5]/180.0f*M_PI;   //roll

  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  transform_2.translation() << x, y, z;
  transform_2.rotate (Eigen::AngleAxisf (r_z, Eigen::Vector3f::UnitZ()));
  transform_2.rotate (Eigen::AngleAxisf (r_y, Eigen::Vector3f::UnitY()));
  transform_2.rotate (Eigen::AngleAxisf (r_x, Eigen::Vector3f::UnitX()));
  
  Eigen::Matrix4f T_c1w_c2w = transform_2.matrix();
  Eigen::Matrix4f T_c_cw;
  T_c_cw << 0,-1,0,0, 0,0,-1,0, 1,0,0,0, 0,0,0,1;
  Eigen::Matrix4f T_c1_c2;
  T_c1_c2 = T_c_cw * T_c1w_c2w * T_c_cw.inverse();
  return T_c1_c2;
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

  pcl::PointCloud<pcl::PointXYZ>::Ptr right_pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*right_input, *right_pcl_pointcloud);

  // std::string filename_left("/home/xujing/catkin_ws/pointcloud_files_left/pcd_left_"+ std::to_string(frame_cnt) +".pcd");
  // std::string filename_right("/home/xujing/catkin_ws/pointcloud_files_right/pcd_right_"+ std::to_string(frame_cnt) +".pcd");
  // pcl::PCDWriter writer;
  // writer.write(filename_left,*left_pcl_pointcloud);
  // writer.write(filename_right,*right_pcl_pointcloud);
  // frame_cnt ++;

  // convert pointcloud on cam2 to cam1
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::transformPointCloud(*right_pcl_pointcloud, *transformed_cloud, tr_matrix);
  // std::cout<< transform_2.matrix() << std::endl;

  // joint pointclouds from left and right realsense camera 
  pcl::PointCloud<pcl::PointXYZ>::Ptr joint_pointcloud(new pcl::PointCloud<pcl::PointXYZ>());
  // *joint_pointcloud = *transformed_cloud;
  *joint_pointcloud = *left_pcl_pointcloud + *transformed_cloud;

  // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  // icp.setInputSource(transformed_cloud);
  // icp.setInputTarget(left_pcl_pointcloud);
  // icp.align(*joint_pointcloud);

  // pass through filter
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass_filtered_x (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (joint_pointcloud);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-0.5, 1.5);  // 0.1~1.5m 
  pass.filter (*cloud_pass_filtered_x); 

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass_filtered_y (new pcl::PointCloud<pcl::PointXYZ>);
  pass.setInputCloud (cloud_pass_filtered_x);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-1.0, 1.0);  // -1.0~1.0m 
  pass.filter (*cloud_pass_filtered_y); 

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pass.setInputCloud (cloud_pass_filtered_y);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-1.0,1.5);  // -1.0~1.0m 
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

  pub_pointcloud.header = left_input->header;

  if(pub_Cloud.getNumSubscribers() > 0)
    pub_Cloud.publish(pub_pointcloud);

}
}