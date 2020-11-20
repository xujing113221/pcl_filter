#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <visualization_msgs/Marker.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/transforms.h>

#include <chrono>
#include <thread>
#include <mutex>
#include <math.h>

#include "pcl_filter/CloudClusters.h"

namespace pcl_filter
{
/** This nodelet creates point cloud completions based on the received point cloud and the camera position */
class PCLCloudMirrorNodelet : public nodelet::Nodelet
{
 public:
  virtual void onInit();
  void cloudClusterCallback(pcl_filter::CloudClustersPtr message);

  void threadHandler(pcl_filter::CloudClustersPtr message);
  void mirrorThread(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud,
                    boost::shared_ptr<pcl::PointIndices> pClusterIndices,
                    pcl::ModelCoefficients::Ptr pSupportPlane,
                    boost::shared_ptr<Eigen::Vector3f> pViewPoint,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr pMirrorCloud);

  pcl::PointXYZ mirrorPoint(pcl::PointXYZ pPoint, pcl::ModelCoefficients::Ptr pMirrorPlane);
  void calculateScore(pcl::PointCloud<pcl::PointXYZ>::Ptr pSampleCloud,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr pMirrorCloud,
                       pcl::ProjectInliers<pcl::PointXYZ>::Ptr pProj,
                       pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr pSamplekdtree,
                       pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr pProjkdtree,
                       float *pCurrentBestScore,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr pCurrentBestCloud);

  void publish_rviz(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pClouds);
  void publish_result(pcl_filter::CloudClustersPtr message, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pClouds);

 private:
  ros::NodeHandle private_nh;
  ros::Subscriber sub_;
  ros::Publisher pub_rviz;
  ros::Publisher pub_result;

  tf::TransformListener listener;

  std::mutex proj_mutex;
  std::mutex score_mutex;

};
}
