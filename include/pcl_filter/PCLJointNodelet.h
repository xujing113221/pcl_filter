#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <chrono>
#include <thread>

using namespace message_filters;


namespace pcl_filter
{
/** This nodelet receives point clouds from a depth camera or a gazebo simulation and prepares them for the next nodelets of the ERGA. */
class PCLJointNodelet : public nodelet::Nodelet
{
 public:
  virtual void onInit();
  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& left_input, const sensor_msgs::PointCloud2::ConstPtr& right_input);
  void handler(const sensor_msgs::PointCloud2::ConstPtr& left_input, const sensor_msgs::PointCloud2::ConstPtr& right_input);

 private:
  ros::NodeHandle private_nh;
//   ros::Subscriber sub_cam1;
//   ros::Subscriber sub_cam2;
  ros::Publisher pub_Cloud;

  bool useDownsample = true;
};

}