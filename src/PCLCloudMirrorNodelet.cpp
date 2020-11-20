#include <pluginlib/class_list_macros.h>

#include "pcl_filter/PCLCloudMirrorNodelet.h"

PLUGINLIB_EXPORT_CLASS(pcl_filter::PCLCloudMirrorNodelet, nodelet::Nodelet)

namespace pcl_filter{

// Settings for the additional mirror clouds
// The initial mirror cloud will be first rotated in ROTATION_STEPS times in negative and positive direction to a maximum angle of ROTATION_RANGE
// Then these rotated clouds will be translated TRANSLATION_STEPS times away from the camera to maximum distance if TRANSLATION_RANGE
// A translation towards the camera does not create reasonable mirror clouds, because the mirror clouds move in front of the sample cloud
#define ROTATION_STEPS 3
#define ROTATION_RANGE 5.0f*(M_PI)/180.0f
#define TRANSLATION_STEPS 3
#define TRANSLATION_RANGE 0.15f

/***************************************************************************//**
 * Prepare the ROS subscriber and publisher
 *
 ******************************************************************************/
void PCLCloudMirrorNodelet::onInit(){
  NODELET_INFO("Initializing PCL CloudMirror nodelet...");

  private_nh = getPrivateNodeHandle();
  sub_ = private_nh.subscribe("/PCLNodelets/PCLFilterNodelet/filteredPointClusters", 1, &PCLCloudMirrorNodelet::cloudClusterCallback, this);
  pub_rviz = private_nh.advertise<sensor_msgs::PointCloud2>("completedCluster", 1);
  pub_result = private_nh.advertise<pcl_filter::CloudClusters>("mirroredClouds", 1);
}

/***************************************************************************//**
 * The callback function called when something was published to the subscribed topic
 *
 * creates a handler thread that will do the work the caller of the callback function can return
 *
 * @param message The message received from ROS as ConstPtr aka boost::shared_pointer
 *
 ******************************************************************************/
void PCLCloudMirrorNodelet::cloudClusterCallback(pcl_filter::CloudClustersPtr message){

  NODELET_INFO("Got Clusters");

  std::thread handler(&PCLCloudMirrorNodelet::threadHandler, this, message);

  handler.detach();
}

/***************************************************************************//**
 * The thread that will do the mirroring
 *
 * @param message The message received from ROS as ConstPtr aka boost::shared_pointer
 *
 ******************************************************************************/
void PCLCloudMirrorNodelet::threadHandler(pcl_filter::CloudClustersPtr message){

  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

  // Read the ROS-Message
  // convert point-cloud in pcl-format
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(message->pointCloud, *cloud);

  // convert cluster_indices in pcl-format
  std::vector<boost::shared_ptr<pcl::PointIndices>> cluster_indices;
  for (int i = 0; i < message->num_cluster; i++) {
    boost::shared_ptr<pcl::PointIndices> tmp_indices (new pcl::PointIndices);
    pcl_conversions::toPCL(message->cluster_indices[i], *tmp_indices);
    cluster_indices.push_back(tmp_indices);
  }

  // calculate support plane in the camera_depth_optical_frame

  tf::StampedTransform transform;
  tf::Stamped<tf::Vector3> world_normal;
  tf::Stamped<tf::Vector3> optical_frame_normal;

  world_normal.setValue(0,0,1);
  world_normal.frame_id_ = "/world";
  optical_frame_normal.setValue(0,0,1);
  optical_frame_normal.frame_id_ = "/camera_depth_optical_frame";

  try{
    // listener.transformVector("/camera_depth_optical_frame", message->header.stamp, world_normal, "/world", optical_frame_normal);
    listener.lookupTransform("/camera_depth_optical_frame", "/camera_depth_optical_frame", message->header.stamp, transform);
  }catch(tf::TransformException &ex){
    NODELET_ERROR("%s", ex.what());
  }

  // create plane in coordinate-form: ax + by + cz + d=0
  // with the coefficients [a, b, c, d]
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
  coefficients->values.resize(4);
  coefficients->values[0] = optical_frame_normal.getX();
  coefficients->values[1] = optical_frame_normal.getY();
  coefficients->values[2] = optical_frame_normal.getZ();
  coefficients->values[3] = -transform.getOrigin().getX() * optical_frame_normal.getX()
                            -transform.getOrigin().getY() * optical_frame_normal.getY()
                            -transform.getOrigin().getZ() * optical_frame_normal.getZ();

  // calculate the view-point projection / origin to the support-plane
  // lamda = scalar-factor for the normal vector of the plane
  // the normal-vector fits lamda-times between the origin (view point) and the plane
  float lamda = (-coefficients->values[3]) / (pow(coefficients->values[0], 2) + pow(coefficients->values[1], 2) + pow(coefficients->values[2], 2));
  boost::shared_ptr<Eigen::Vector3f> view_point (new Eigen::Vector3f(lamda * coefficients->values[0], lamda * coefficients->values[1], lamda * coefficients->values[2]));

  // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
  // coefficients->values[0] = 1;
  // coefficients->values[1] = 1;
  // coefficients->values[2] = 1.0f;
  // coefficients->values[3] = 0.8f;
  // float lamda = (-coefficients->values[3]) / (pow(coefficients->values[0], 2) + pow(coefficients->values[1], 2) + pow(coefficients->values[2], 2));
  // boost::shared_ptr<Eigen::Vector3f> view_point (new Eigen::Vector3f(lamda * coefficients->values[0], lamda * coefficients->values[1], lamda * coefficients->values[2]));

  // boost::shared_ptr<Eigen::Vector3f> view_point (new Eigen::Vector3f(0,0,0.8f));
  // prepare and start workers
  std::vector<std::thread> workers;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> mirrorClouds;

  for(int i = 0; i < message->num_cluster; i++){

    mirrorClouds.push_back(pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>));

    workers.push_back(std::thread(&PCLCloudMirrorNodelet::mirrorThread,
                                  this,
                                  cloud,
                                  cluster_indices[i],
                                  coefficients,
                                  view_point,
                                  mirrorClouds[mirrorClouds.size()-1]
                                  ));
  }

  // wait for all threads to finish
  std::for_each(workers.begin(), workers.end(),[](std::thread &t){t.join(); });

  publish_result(message, mirrorClouds);

  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  // NODELET_INFO("Published mirrored Clusters. Calculation took %3.3f [ms]", std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count() / 1000000.0f );

  // write time into a file
  //std::ofstream file;
  //file.open("/home/hiwi/Desktop/time.csv",std::ofstream::app);
  //file << (std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count() / 1000000.0f) << "\n";
  //file.close();

  // only for visualization does not increase the calculation time because the results for the next nodelet were already published
  publish_rviz(mirrorClouds);
}

/***************************************************************************//**
 * The thread that will calculate the mirror cloud for one object
 *
 * @param pCloud The whole cloud received inside the message
 * @param pClusterIndices One entry from the indices vector inside the message. Defines one object in the point cloud
 * @param pSupportPlane The previously calculated parameters of the support plane.
 * @param pViewPoint The previously calculated projection of the view point onto the support plane.
 * @param pMirrorCloud The output parameter for the calculated mirror cloud.
 *
 ******************************************************************************/
void PCLCloudMirrorNodelet::mirrorThread(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud,
                                         boost::shared_ptr<pcl::PointIndices> pClusterIndices,
                                         pcl::ModelCoefficients::Ptr pSupportPlane,
                                         boost::shared_ptr<Eigen::Vector3f> pViewPoint,
                                         pcl::PointCloud<pcl::PointXYZ>::Ptr pMirrorCloud){

  pcl::PointCloud<pcl::PointXYZ>::Ptr sampleCloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*pCloud, *pClusterIndices, *sampleCloud);

  // projection of the sample cloud to the support plane
  pcl::PointCloud<pcl::PointXYZ>::Ptr projCloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ProjectInliers<pcl::PointXYZ>::Ptr proj (new pcl::ProjectInliers<pcl::PointXYZ>);
  proj->setModelType(pcl::SACMODEL_PLANE);
  proj->setInputCloud(sampleCloud);
  proj->setModelCoefficients(pSupportPlane);
  proj->filter(*projCloud);

  // calculate the eigenvectors
  pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud(projCloud);
  feature_extractor.compute();

  Eigen::Vector3f major_vector, middle_vector, minor_vector;
  Eigen::Vector3f mass_center;

  feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
  feature_extractor.getMassCenter(mass_center);

  // find the eigenvector most perpendicular to the viewing direction
  // middle_vector and major_vector are used. The minor_vector is the normal vector of the support plane
  // one of the both that is not more perpendicular is the normal vector of the mirror plane
  // the normal vector is inverted if it shows in the direction if the view point
  Eigen::Vector3f viewing_direction = mass_center - *pViewPoint;
  viewing_direction.normalize();
  Eigen::Vector3f initial_mirror_normal;

  float majorDotProdukt = viewing_direction.dot(major_vector);
  float middleDotProdukt = viewing_direction.dot(middle_vector);

  if( abs(majorDotProdukt) <= abs(middleDotProdukt))
    initial_mirror_normal = middleDotProdukt > 0 ? middle_vector : -middle_vector;
  else
    initial_mirror_normal = majorDotProdukt > 0 ? major_vector : -major_vector;


  initial_mirror_normal.normalize();

  // create inital mirror-plane
  pcl::ModelCoefficients::Ptr initial_mirror (new pcl::ModelCoefficients);
  initial_mirror->values.resize(4);

  //Eigen::Vector3f initial_mirror_normal = initial_mirror_vector.cross(Eigen::Vector3f(pSupportPlane->values[0], pSupportPlane->values[1], pSupportPlane->values[2]));
  initial_mirror->values[0] = initial_mirror_normal(0);
  initial_mirror->values[1] = initial_mirror_normal(1);
  initial_mirror->values[2] = initial_mirror_normal(2);
  initial_mirror->values[3] = -(mass_center.dot(initial_mirror_normal));

  // create inital mirror-cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr initialMirrorCloud (new pcl::PointCloud<pcl::PointXYZ>);
  for(std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>>::iterator i = sampleCloud->points.begin(); i!=sampleCloud->points.end(); i++){
    initialMirrorCloud->points.push_back(mirrorPoint(*i, initial_mirror));
  }

  /**************************************************************************/
  // create additional Hypothesis by transforming the initial mirror-cloud
  /**************************************************************************/
  std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>> hypothesisClouds;
  std::vector<Eigen::Matrix3f> usedRotMatrix;
  hypothesisClouds.push_back(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>());

  /**************************************************************************/
  /* rotate the initial mirror cloud                                        */
  /**************************************************************************/
  for(int rotationStep = -ROTATION_STEPS; rotationStep <= ROTATION_STEPS; rotationStep++){

    Eigen::Matrix4f transMatrix = Eigen::Matrix4f::Identity();
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud (new pcl::PointCloud<pcl::PointXYZ>);
    float angle = 1.0f/ROTATION_STEPS * rotationStep * ROTATION_RANGE;

    // translate to origin to make the rotation easy the cloud will be translated back in the next step
    transMatrix.block<3,1>(0,3) = -mass_center;
    pcl::transformPointCloud(*initialMirrorCloud, *transformedCloud, transMatrix);

    // create the Rotation-Matrix with the Rodrigues Rotation Formular
    Eigen::Matrix3f w = Eigen::Matrix3f::Zero();
    w(0,1) = - pSupportPlane->values[2];
    w(0,2) = + pSupportPlane->values[1];
    w(1,0) = + pSupportPlane->values[2];
    w(1,2) = - pSupportPlane->values[0];
    w(2,0) = - pSupportPlane->values[1];
    w(2,1) = + pSupportPlane->values[0];

    Eigen::Matrix3f rot;
    rot = Eigen::Matrix3f::Identity() + w * sin(angle) + w*w * (1 - cos(angle));

    transMatrix = Eigen::Matrix4f::Identity();
    transMatrix.block<3,3>(0,0) = rot;
    pcl::transformPointCloud(*transformedCloud, *transformedCloud, transMatrix);

    hypothesisClouds.at(0).push_back(transformedCloud);
    usedRotMatrix.push_back(rot);
  }

  /**************************************************************************/
  /* rotate the initial mirror cloud                                        */
  /**************************************************************************/
  for(int transStep = 0; transStep <= TRANSLATION_STEPS; transStep++){
    Eigen::Matrix4f transMatrix = Eigen::Matrix4f::Identity();
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud (new pcl::PointCloud<pcl::PointXYZ>);

    // write the translation matrix if it is not the not translated first row of the cloud matrix (hypothesisClouds)
    // than just move the cloud back to the original mass center
    // the other clouds will be copied and translated from this first row of the cloud matrix (hypothesisClouds)
    Eigen::Vector3f trans = (transStep == 0 ? mass_center : (1.0f/TRANSLATION_STEPS * transStep * TRANSLATION_RANGE * initial_mirror_normal));

    transMatrix.block<3,1>(0,3) = trans;

    for(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator i = hypothesisClouds.at(0).begin(); i < hypothesisClouds.at(0).end(); i++){
      if(transStep == 0){
        transMatrix.block<3, 1>(0, 3) = trans;
        pcl::transformPointCloud(**i, **i, transMatrix);
      } else {
        transMatrix.block<3, 1>(0, 3) =  (usedRotMatrix.at(i - hypothesisClouds.at(0).begin())) * trans;
        pcl::transformPointCloud(**i, *transformedCloud, transMatrix);
        if (i == hypothesisClouds.at(0).begin()) hypothesisClouds.push_back(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>());
        hypothesisClouds[hypothesisClouds.size() - 1].push_back(transformedCloud);
      }
    }
  }

  /**************************************************************************/
  // find best MirrorCloud
  /**************************************************************************/
  // calculate center of sample cloud for view plane
  pcl::MomentOfInertiaEstimation<pcl::PointXYZ> center_extractor;
  center_extractor.setInputCloud(sampleCloud);
  center_extractor.compute();
  Eigen::Vector3f center;
  center_extractor.getMassCenter(center);

  Eigen::Vector3f center_normalized = center;
  center_normalized.normalize();
  pcl::PointCloud<pcl::PointXYZ>::Ptr projSampleCloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ModelCoefficients::Ptr viewPlane (new pcl::ModelCoefficients);
  viewPlane->values.resize(4);
  viewPlane->values[0] = 0; //center_normalized.x();
  viewPlane->values[1] = 0; //center_normalized.y();
  viewPlane->values[2] = 1; //center_normalized.z();
  viewPlane->values[3] = 0;

  proj->setModelCoefficients(viewPlane);
  proj->setInputCloud(sampleCloud);
  proj->filter(*projSampleCloud);

  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr samplekdtree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
  samplekdtree->setInputCloud(sampleCloud);

  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr projkdtree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
  projkdtree->setInputCloud(projSampleCloud);

  float bestScore = -1;
  pcl::PointCloud<pcl::PointXYZ>::Ptr best_Cloud (new pcl::PointCloud<pcl::PointXYZ>);

  std::vector<std::thread> scoreWorker;

  for(std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>>::iterator i = hypothesisClouds.begin(); i < hypothesisClouds.end(); i++){
    for(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator j = i->begin(); j < i->end(); j++){
      scoreWorker.push_back(std::thread(&PCLCloudMirrorNodelet::calculateScore,
                                         this,
                                         sampleCloud,
                                         *j,
                                         proj,
                                         samplekdtree,
                                         projkdtree,
                                         &bestScore,
                                         best_Cloud
                                         ));
      //calculateScore(sampleCloud, *j, proj, samplekdtree,projkdtree,&bestScore,best_Cloud);
    }
  }

  std::for_each(scoreWorker.begin(), scoreWorker.end(), [](std::thread &t){t.join(); });

  *pMirrorCloud = *best_Cloud;
}

/***************************************************************************//**
 * Function to calculate the mirror point to a given point and a mirror plane
 *
 \verbatim
   Mirror point about the given Mirror-Plane


                         |
        Mirror-Line g    |  Intersection-Point
                     \  _| /
                      \|o|/
        P X -------------X------------- X P'
                         |
                         |
                         |
                         |
                   Mirror-Plane E

            --   --                                           -- --
        ->  | p_1 |                                  ->       | a |
        P = | p_2 |    E: ax + by + cz + d = 0    g: P + k *  | b |
            | p_3 |                                           | c |
            --   --                                           -- --

        calculate the factor k' to get the the intersection between g and E
        => put g into E

                -(a * p_1 + b * p_2 + c * p_3 + d)
        => k' = ----------------------------------
                         (a² + b² + c²)

        in order to get the point P' the factor k' has to be multiplied by 2
                             -- --
          ->   ->            | a |
        => P' = P + 2 * k' + | b |
                             | c |
                             -- --

 \endverbatim
 *
 * @param pPoint The point to mirror
 * @param pMirrorPlane The mirror plane the point should be mirrored
 * @return The mirrored point
 *
 ******************************************************************************/
pcl::PointXYZ PCLCloudMirrorNodelet::mirrorPoint(pcl::PointXYZ pPoint, pcl::ModelCoefficients::Ptr pMirrorPlane){

  float k = -(pMirrorPlane->values[0] * pPoint.x +
              pMirrorPlane->values[1] * pPoint.y +
              pMirrorPlane->values[2] * pPoint.z +
              pMirrorPlane->values[3])
            / (pMirrorPlane->values[0] * pMirrorPlane->values[0] +
               pMirrorPlane->values[1] * pMirrorPlane->values[1] +
               pMirrorPlane->values[2] * pMirrorPlane->values[2]);

  pcl::PointXYZ result(pPoint.x + 2 * k * pMirrorPlane->values[0],
                       pPoint.y + 2 * k * pMirrorPlane->values[1],
                       pPoint.z + 2 * k * pMirrorPlane->values[2]);

  return result;
}

/***************************************************************************//**
 * Function to calculate the error for a given mirror cloud
 *
 * @param pSampleCloud The original captured point cloud of the object.
 * @param pMirrorCloud The mirrored cloud to evaluate.
 * @param pProj The projection object for the view plane
 * @param pSamplekdtree The KdTree for neighborhood search prepared with the sample cloud.
 * @param pProjkdtree The KdTree for neighborhood search prepared with the sample cloud projection on the view plane
 * @param pCurrentBestScore The currently smallest error calculated for other mirror clouds. -1 if no other score was calculated yet.
 * @param pCurrentBestCloud shared pointer to mirror cloud with the currently smallest error
 *
 ******************************************************************************/
void PCLCloudMirrorNodelet::calculateScore(pcl::PointCloud<pcl::PointXYZ>::Ptr pSampleCloud,
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr pMirrorCloud,
                                            pcl::ProjectInliers<pcl::PointXYZ>::Ptr pProj,
                                            pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr pSamplekdtree,
                                            pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr pProjkdtree,
                                            float *pCurrentBestScore,
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr pCurrentBestCloud){

  //transform the PointCloud into the view-plane (plane with the view-direction as normal)
   pcl::PointCloud<pcl::PointXYZ>::Ptr projMirrorCloud (new pcl::PointCloud<pcl::PointXYZ>);

  //{
  //  std::lock_guard<std::mutex> lock(proj_mutex);
  //  pProj->setInputCloud(pMirrorCloud);
  //  pProj->filter(*projMirrorCloud);
  // }

  pcl::ModelCoefficients::Ptr viewPlane (new pcl::ModelCoefficients);
  viewPlane->values.resize(4);
  viewPlane->values[0] = 0; //center_normalized.x();
  viewPlane->values[1] = 0; //center_normalized.y();
  viewPlane->values[2] = 1; //center_normalized.z();
  viewPlane->values[3] = 0;

  pcl::ProjectInliers<pcl::PointXYZ>::Ptr proj (new pcl::ProjectInliers<pcl::PointXYZ>);
  proj->setModelType(pcl::SACMODEL_PLANE);
  proj->setInputCloud(pMirrorCloud);
  proj->setModelCoefficients(viewPlane);
  proj->filter(*projMirrorCloud);

  //calculate score for every point in the MirrorCloud
  float result = 0.0f;

  for(int pointIndex = 0; pointIndex < pMirrorCloud->points.size(); pointIndex++ ){
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquareDistance(1);

    pProjkdtree->nearestKSearch(projMirrorCloud->points.at(pointIndex), 1, pointIdxNKNSearch, pointNKNSquareDistance);

    if(sqrt(pointNKNSquareDistance.at(0)) * 1000.0f > 5.0f){
      // point outside the segmentation mask
      // calculate distance to segmentation mask
      result += pow(sqrt(pointNKNSquareDistance.at(0)) * 1000.0f, 2);
    }else{
      // point inside the segmentation mask
      // if point closer to camera than existing points calculate distance to nearest point
      //pSamplekdtree->nearestKSearch(pMirrorCloud->points.at(pointIndex), 1, pointIdxNKNSearch, pointNKNSquareDistance);

      float sampleDistance = sqrt(pow(pSampleCloud->points.at(pointIdxNKNSearch.at(0)).x * 1000.0f, 2)+
                                  pow(pSampleCloud->points.at(pointIdxNKNSearch.at(0)).y * 1000.0f, 2)+
                                  pow(pSampleCloud->points.at(pointIdxNKNSearch.at(0)).z * 1000.0f, 2));

      float mirrorDistance = sqrt(pow(pMirrorCloud->at(pointIndex).x * 1000.0f, 2)+
                                  pow(pMirrorCloud->at(pointIndex).y * 1000.0f, 2)+
                                  pow(pMirrorCloud->at(pointIndex).z * 1000.0f, 2));

      if(sampleDistance > mirrorDistance){
        float tmp = result;
        result += pow(sampleDistance - mirrorDistance,2) * 2;
        //NODELET_WARN("%.3f | %.3f",tmp,result);
      }
    }
  }

  {
    std::lock_guard<std::mutex> lock(score_mutex);
    if((*pCurrentBestScore > result) || *pCurrentBestScore == -1){
      *pCurrentBestScore = result;
      *pCurrentBestCloud = *pMirrorCloud;
    }
  }
}

/***************************************************************************//**
 * Function to publish the results for rviz visualization
 *
 * @param message The received message to which the mirror clouds will be added and then published
 * @param pClouds The mirror clouds which should be published
 *
 ******************************************************************************/
void PCLCloudMirrorNodelet::publish_result(pcl_filter::CloudClustersPtr message, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pClouds){

  message->header.stamp = ros::Time();

  message->mirror_exists = true;

  for(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator i = pClouds.begin(); i < pClouds.end(); i++){
    sensor_msgs::PointCloud2 msg_cloud;
    pcl::toROSMsg(**i, msg_cloud);
    message->mirrorCloud.push_back(msg_cloud);
  }

  pub_result.publish(message);
}


/***************************************************************************//**
 * Function to publish the results for rviz visualization
 *
 * @param pClouds Vector of a mirror cloud for every object
 *
 ******************************************************************************/
void PCLCloudMirrorNodelet::publish_rviz(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pClouds){

  pcl::PointCloud<pcl::PointXYZ>::Ptr result (new pcl::PointCloud<pcl::PointXYZ>());

  for(int i=0; i < pClouds.size(); i++){
    *result += *pClouds[i];
  }

  sensor_msgs::PointCloud2 msg_cloud;
  pcl::toROSMsg(*result, msg_cloud);

  msg_cloud.header.frame_id = "/camera_depth_optical_frame";

  pub_rviz.publish(msg_cloud);

}
   
}
