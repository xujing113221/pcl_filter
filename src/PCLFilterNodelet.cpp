#include <pluginlib/class_list_macros.h>

#include "pcl_filter/PCLFilterNodelet.h"

#include <pcl/io/pcd_io.h>

PLUGINLIB_EXPORT_CLASS(pcl_filter::PCLFilterNodelet, nodelet::Nodelet)

namespace pcl_filter{

/***************************************************************************//**
 * Prepare the ROS subscriber and publishers
 * The "pub_Cluster" publisher will publish the clusters in the "CloudClusters" type for the next nodelet
 * The "pub_Cloud" publisher will publish the whole point cloud as "PointCloud2" for rviz visualization
 *
 ******************************************************************************/
void PCLFilterNodelet::onInit(){
  NODELET_INFO("Initializing PCL Filter nodelet...");
  private_nh = getPrivateNodeHandle();
  sub_ = private_nh.subscribe("/camera/depth/color/points", 10, &PCLFilterNodelet::pointCloudCallback, this);
  // sub_ = private_nh.subscribe("/PCLNodelets/PCLJointNodelet/jointedPointCloud", 1, &PCLFilterNodelet::pointCloudCallback, this);
  pub_Cluster = private_nh.advertise<pcl_filter::CloudClusters>("filteredPointClusters",10);
  pub_Cloud = private_nh.advertise<sensor_msgs::PointCloud2>("filterdPointCloud",10);

  private_nh.param("useDownsample", useDownsample, false);
  NODELET_INFO(useDownsample ? "Filter nodelet is using downsampling!" : "Filter nodelet is not using downsampling!");
}

/***************************************************************************//**
 * The callback function called when something was published to the subscribed topic
 *
 * creates a handler thread that will do the work the caller of the callback function can return
 *
 * @param message The message received from ROS as ConstPtr aka boost::shared_pointer
 *
 ******************************************************************************/
void PCLFilterNodelet::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr message){

  // NODELET_INFO("Got new point cloud");

  // std::thread handler(&PCLFilterNodelet::handler, this, message);

  handler(message);

  // handler.detach();
}

/***************************************************************************//**
 * The handler thread will handles the filtering of the received point cloud
 *
 * @param message The message received from ROS as ConstPtr aka boost::shared_pointer
 *
 ******************************************************************************/
static int frame_cnt =0;
void PCLFilterNodelet::handler(const sensor_msgs::PointCloud2ConstPtr message){
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>),cloud_m (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*message, *cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  //Creating the filtering object: downsample the dataset using a leaf size of 1,5cm
  // Optional just has to be done when using the gazebo simulation
  if (useDownsample) {
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.015f, 0.015f, 0.015f);
    vg.filter(*cloud_filtered);
  } else {
    cloud_filtered = cloud;
  }

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud_filtered);
  sor.setMeanK(20);
  sor.setStddevMulThresh(0.7);
  sor.filter(*cloud_filtered);



  //Create the segmentation object for the planer model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients); // will contain the plane parameters in "values" (in ax+by+cz+d=0 // form)
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ>);
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.02); // 2cm

  int i=0, nr_points = static_cast<int>(cloud_filtered->points.size());
  while(cloud_filtered->points.size() > 0.3 * nr_points){
    //Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
      NODELET_WARN("Cloud not estimate a planar model for the given database.");
      break;
    }

    // Remove the planar inliers, extract the rest
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_f);
    *cloud_filtered = *cloud_f;

  }
  
  // Upsampling
  // pcl::MovingLeastSquares<pcl::PointXYZ,pcl::PointXYZ> mls;
  // mls.setInputCloud(cloud_filtered);
  // pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
  // mls.setSearchMethod(kdtree);
  // mls.setSearchRadius(0.03);
  // mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ,pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
  // mls.setUpsamplingRadius(0.03);
  // mls.setUpsamplingStepSize(0.02);
  // mls.process(*cloud_m);
  // *cloud_filtered = *cloud_m;


  // Create outlier filter
  // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud_filtered);
  sor.setMeanK(20);
  sor.setStddevMulThresh(1);
  sor.filter(*cloud_filtered);

  // Creating the KdTree object for the search method of the extraction
  // this will detected the clusters, which represent the objects
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.05); // 5cm
  ec.setMinClusterSize(35);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered);
  ec.extract(cluster_indices);

  // Publish the filterd clusters
  pcl_filter::CloudClusters msg;
  msg.num_cluster = cluster_indices.size();
  pcl::toROSMsg(*cloud_filtered, msg.pointCloud);

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
    pcl_msgs::PointIndices msgIndices;
    pcl_conversions::fromPCL(*it, msgIndices);
    msg.cluster_indices.push_back(msgIndices);
    }

 if(pub_Cluster.getNumSubscribers()>0)
  pub_Cluster.publish(msg);

  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

  NODELET_INFO("PointClusters published! Filtering took %3.3f [ms]", std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count() /1000000.0f);

  // write time into a file
  //std::ofstream file;
  //file.open("/home/hiwi/Desktop/time.csv",std::ofstream::app);
  //file << (std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count() / 1000000.0f) << "\n";
  //file.close();
  //   std::string filename("/home/kamera/catkin_ws/pointcloud_files/pcd_"+ std::to_string(frame_cnt) +".pcd");
  //   frame_cnt++;
  //   pcl::PCDWriter writer;
  //   writer.write(filename,*cloud);

  if(pub_Cloud.getNumSubscribers()>0)
    pub_Cloud.publish(msg.pointCloud);
}

}
