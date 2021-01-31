#include <nodelet/nodelet.h>
#include <ros/ros.h>
#define _USE_MATH_DEFINES
#include <sensor_msgs/PointCloud2.h>
#include <math.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>


#include <chrono>
#include <thread>

using namespace message_filters;

namespace pcl_filter
{
/** This nodelet receives point clouds from two depth cameras and joint them for the next nodelets . */
class PCLJointNodelet : public nodelet::Nodelet
{
 public:
  virtual void onInit();
  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& left_input, const sensor_msgs::PointCloud2::ConstPtr& right_input);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  ros::NodeHandle private_nh;
  ros::Publisher pub_Cloud;
  Eigen::Matrix4f tr_matrix;
  Eigen::Matrix4f get_transform_matrix(const float tr_info[]);
};

}