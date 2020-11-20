#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <thread>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <chrono>
#include <mutex>

#include "pcl_filter/CloudClusters.h"

//bounding Mesh
#include "pcl_filter/boundingmesh/Decimator.h"

namespace pcl_filter
{

/** This nodelet creates meshes to the given point cloud based on the shrink-wrapping algorithm and optional on the bounding mesh algorithm */
class PCLShrinkWrappingNodelet : public nodelet::Nodelet
{
 public:
  PCLShrinkWrappingNodelet();
  virtual void onInit();
  void cloudClusterCallback(const pcl_filter::CloudClustersConstPtr  message);

  void updateEdgesPolygonAdd(pcl::Vertices pPolygon, boost::shared_ptr<std::vector<pcl::Vertices>> pEdges);
  void updateEdgesPolygonRemove(pcl::Vertices pPolygon, boost::shared_ptr<std::vector<pcl::Vertices>> pEdges);

  void createInitialBoundingBoxMesh(boost::shared_ptr<std::vector<pcl::Vertices>> pPolygons,
                                    boost::shared_ptr<std::vector<pcl::Vertices>> pEdges,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud);

  void subDivideBoundingBox(boost::shared_ptr<std::vector<pcl::Vertices>> pPolygons,
                            boost::shared_ptr<std::vector<pcl::Vertices>> pEdges,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud,
                            unsigned int pIterations);

  void threadHandler(const pcl_filter::CloudClustersConstPtr message);

  void generateMeshThread(pcl::PointCloud<pcl::PointXYZ>::Ptr pClusterCloud,
                          boost::shared_ptr<pcl::PointIndices> pCluster,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr pMeshCloud,
                          boost::shared_ptr<std::vector<pcl::Vertices>> pPolygons,
                          boost::shared_ptr<std::vector<pcl::Vertices>> pEdges);

  void generateMeshThread_Mirror(pcl::PointCloud<pcl::PointXYZ>::Ptr pClusterCloud,
                                 boost::shared_ptr<pcl::PointIndices> pCluster,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr pMeshCloud,
                                 boost::shared_ptr<std::vector<pcl::Vertices>> pPolygons,
                                 boost::shared_ptr<std::vector<pcl::Vertices>> pEdges,
                                 sensor_msgs::PointCloud2 mirrorClouds);

  void generateMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr pSampleCloud,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr pMeshCloud,
                    boost::shared_ptr<std::vector<pcl::Vertices>> pPolygons,
                    boost::shared_ptr<std::vector<pcl::Vertices>> pEdges);

  void transformationToGivenOBB(pcl::PointXYZ *pMinOBB,
                                pcl::PointXYZ *pMaxOBB,
                                Eigen::Matrix3f *pRot,
                                pcl::PointXYZ *pTrans,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud);

  void projectionOperation(pcl::PointCloud<pcl::PointXYZ>::Ptr pSampleCloud,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr pMeshCloud,
                           boost::shared_ptr<std::vector<pcl::PointXYZ>> pNormals);

  void smoothingOperation(pcl::PointCloud<pcl::PointXYZ>::Ptr pMeshCloud,
                          boost::shared_ptr<std::vector<pcl::Vertices>> pEdges,
                          boost::shared_ptr<std::vector<pcl::PointXYZ>> pNormals);

  void publish(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pMeshClouds, std::vector<boost::shared_ptr<std::vector<pcl::Vertices>>> pPolygonsVector);

 private:
  ros::NodeHandle private_nh;
  ros::Subscriber sub_;
  ros::Publisher pub_;

  boost::shared_ptr<std::vector<pcl::Vertices>> initialPolygons;
  boost::shared_ptr<std::vector<pcl::Vertices>> initialEdges;
  pcl::PointCloud<pcl::PointXYZ>::Ptr initialCloud;

  bool useBoundingMesh;
};

}
