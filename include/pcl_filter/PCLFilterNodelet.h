#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/surface/mls.h>

#include <chrono>
#include <thread>

#include "pcl_filter/CloudClusters.h"

namespace pcl_filter
{
/** This nodelet receives point clouds from a depth camera or a gazebo simulation and prepares them for the next nodelets of the ERGA. */
class PCLFilterNodelet : public nodelet::Nodelet
{
 public:
  virtual void onInit();
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr message);

  void handler(const sensor_msgs::PointCloud2ConstPtr message);

 private:
  ros::NodeHandle private_nh;
  ros::Subscriber sub_;
  ros::Publisher pub_Cluster;
  ros::Publisher pub_Cloud;

  bool useDownsample = false;
};
}
