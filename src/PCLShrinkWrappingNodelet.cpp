#include <pluginlib/class_list_macros.h>

#include "pcl_filter/PCLShrinkWrappingNodelet.h"

PLUGINLIB_EXPORT_CLASS(pcl_filter::PCLShrinkWrappingNodelet, nodelet::Nodelet)

namespace pcl_filter{

  /***************************************************************************//**
   * The constructor for the shrink-wrapping nodelet.
   * Initialising the shared pointer to the initial mesh.
   * The nodelet crashes when using not initialised shared pointers
   *
   ******************************************************************************/
  PCLShrinkWrappingNodelet::PCLShrinkWrappingNodelet() : initialPolygons(new std::vector<pcl::Vertices>),
                                                         initialEdges(new std::vector<pcl::Vertices>),
                                                         initialCloud(new pcl::PointCloud<pcl::PointXYZ>){}

  /***************************************************************************//**
   * Prepare the initial bounding box mesh and subdivide it. Also prepare ROS subscriber and publisher
   *
   ******************************************************************************/
  void PCLShrinkWrappingNodelet::onInit(){
    NODELET_INFO("Initializing PCL Shrink Wrapping nodelet...");

    createInitialBoundingBoxMesh(initialPolygons, initialEdges, initialCloud);
    subDivideBoundingBox(initialPolygons, initialEdges, initialCloud, 2);

    private_nh = getPrivateNodeHandle();
    sub_ = private_nh.subscribe("/PCLNodelets/PCLCloudMirrorNodelet/mirroredClouds", 1, &PCLShrinkWrappingNodelet::cloudClusterCallback, this);
    pub_ = private_nh.advertise<visualization_msgs::Marker>("meshes", 2);

    private_nh.param("useBoundingMesh", useBoundingMesh,false);
    NODELET_INFO(useBoundingMesh ? "Shrink-Wrapping is using bounding mesh simplification" : "ShrinkWrapping is not using simplification");
  }

  /***************************************************************************//**
   * ROS callback function. Starts a handler thread for the meshing
   *
   * @param message The message received from ROS as ConstPtr aka boost::shared_pointer
   *
   ******************************************************************************/
  void PCLShrinkWrappingNodelet::cloudClusterCallback(const pcl_filter::CloudClustersConstPtr message){

    NODELET_INFO("Got Clusters");

    // call thread to prepare and start ShrinkWrapping-Threads and detach it in order to keep the callback short

    std::thread handler(&PCLShrinkWrappingNodelet::threadHandler, this, message);

    handler.detach();
  }

  /***************************************************************************//**
   * Maintenance for the mesh data structures.
   * Updating the edges vector when adding a new polygon to the polygon vector
   *
   * @param pPolygon The polygon added to the polygon vector
   * @param pEdges The edges vector to be update
   *
   ******************************************************************************/
  void PCLShrinkWrappingNodelet::updateEdgesPolygonAdd(pcl::Vertices pPolygon, boost::shared_ptr<std::vector<pcl::Vertices>> pEdges){

    // Updates the Edges-Vector with the connections contained in the added polygon

    for(int VertextoUpdateID  = 0; VertextoUpdateID < pPolygon.vertices.size(); VertextoUpdateID++){
      for(int potentialVertextoAddID = 0; potentialVertextoAddID < pPolygon.vertices.size(); potentialVertextoAddID++){
        if(VertextoUpdateID != potentialVertextoAddID){
          // Check if the edge is already in pEdges (the vertices list for the VertextoUpdate already contains the potentialVertextoAdd)
          int EdgeVertexID = 0;
          for(EdgeVertexID = 0; EdgeVertexID < pEdges->at(pPolygon.vertices.at(VertextoUpdateID)).vertices.size(); EdgeVertexID++){
            if(pEdges->at(pPolygon.vertices.at(VertextoUpdateID)).vertices.at(EdgeVertexID) == pPolygon.vertices.at(potentialVertextoAddID)) break;
          }
          // Add Vertex if not
          if(EdgeVertexID == pEdges->at(pPolygon.vertices.at(VertextoUpdateID)).vertices.size())
            pEdges->at(pPolygon.vertices.at(VertextoUpdateID)).vertices.push_back(pPolygon.vertices.at(potentialVertextoAddID));
        }
      }
    }

  }

  /***************************************************************************//**
   * Maintenance for the mesh data structures.
   * Updating the edges vector when removing a polygon from the polygon vector
   *
   * @param pPolygon The polygon removed from the polygon vector
   * @param pEdges The edges vector to be update
   *
   ******************************************************************************/
  void PCLShrinkWrappingNodelet::updateEdgesPolygonRemove(pcl::Vertices pPolygon, boost::shared_ptr<std::vector<pcl::Vertices>> pEdges){

    // Updates the Edges-Vector with the connections contained in the removed polygon

    for(int VertextoUpdateID = 0; VertextoUpdateID < pPolygon.vertices.size(); VertextoUpdateID++){
      for(int VertextoRemoveID = 0; VertextoRemoveID < pPolygon.vertices.size(); VertextoRemoveID++){
        if(VertextoUpdateID != VertextoRemoveID){
          for(int EdgeVertexID = 0; EdgeVertexID < pEdges->at(pPolygon.vertices.at(VertextoUpdateID)).vertices.size(); EdgeVertexID++){
            if(pEdges->at(pPolygon.vertices.at(VertextoUpdateID)).vertices.at(EdgeVertexID) == pPolygon.vertices.at(VertextoRemoveID))
              pEdges->at(pPolygon.vertices.at(VertextoUpdateID)).vertices.erase(pEdges->at(pPolygon.vertices.at(VertextoUpdateID)).vertices.begin() + EdgeVertexID);
          }
        }
      }
    }

  }

  /***************************************************************************//**
   * Creates the initial bounding box mesh without subdivisions
   *
   \verbatim

            7 X------------------X 6      Creates a initial bounding box mesh which will be
             /|                 /|        transformed later to fit the given sample cloud
            / |                / |
           /  |               /  |
        4 X------------------X 5 |        Initial Coordinates:     Added Triangles:
          |   |              |   |        0: (-0.5, -0.5, -0.5)    Bottom:    Front:     Back:
          |   |              |   |        1: ( 0.5, -0.5, -0.5)    0-2-1      0-1-5      2-3-7
          | 3 X--------------|---X 2      2: ( 0.5,  0.5, -0.5)    2-0-3      0-5-4      2-7-6
          |  /               |  /         3: (-0.5,  0.5, -0.5)    Top:       Right:     Left:
          | /                | /          4: (-0.5, -0.5,  0.5)    4-5-6      1-2-6      3-0-4
          |/                 |/           5: ( 0.5, -0.5,  0.5)    6-7-4      1-6-5      3-4-7
          X------------------X            6: ( 0.5,  0.5,  0.5)
          0                  1            7: (-0.5,  0.5,  0.5)

          z                               The numbers represent the index of the point in the
          ^ y                             the PointCloud-object pCloud.
          |/
           -->x

   \endverbatim
   *
   * @param pPolygons shared pointer to the polygon vector to store the polygons
   * @param pEdges shared pointer to the edges vector to store the edges
   * @param pCloud share pointer to the point cloud to store the coordinates of the vertices
   *
   ******************************************************************************/
  void PCLShrinkWrappingNodelet::createInitialBoundingBoxMesh(boost::shared_ptr<std::vector<pcl::Vertices>> pPolygons,
                                                              boost::shared_ptr<std::vector<pcl::Vertices>> pEdges,
                                                              pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud){

    // Insert vertices
    pcl::PointXYZ minOBB(-0.5, -0.5, -0.5);
    pcl::PointXYZ maxOBB(0.5, 0.5, 0.5);
    pCloud->push_back(minOBB);                                    pEdges->push_back(pcl::Vertices());
    pCloud->push_back(pcl::PointXYZ(maxOBB.x,minOBB.y,minOBB.z)); pEdges->push_back(pcl::Vertices());
    pCloud->push_back(pcl::PointXYZ(maxOBB.x,maxOBB.y,minOBB.z)); pEdges->push_back(pcl::Vertices());
    pCloud->push_back(pcl::PointXYZ(minOBB.x,maxOBB.y,minOBB.z)); pEdges->push_back(pcl::Vertices());

    pCloud->push_back(pcl::PointXYZ(minOBB.x,minOBB.y,maxOBB.z)); pEdges->push_back(pcl::Vertices());
    pCloud->push_back(pcl::PointXYZ(maxOBB.x,minOBB.y,maxOBB.z)); pEdges->push_back(pcl::Vertices());
    pCloud->push_back(maxOBB);                                    pEdges->push_back(pcl::Vertices());
    pCloud->push_back(pcl::PointXYZ(minOBB.x,maxOBB.y,maxOBB.z)); pEdges->push_back(pcl::Vertices());

    // Insert Polygons
    pcl::Vertices tmp;

    // Bottom
    //First Triangle
    tmp.vertices.push_back(0);
    tmp.vertices.push_back(2);
    tmp.vertices.push_back(1);
    pPolygons->push_back(tmp);
    updateEdgesPolygonAdd(tmp, pEdges);
    tmp.vertices.clear();

    //Second Triangle
    tmp.vertices.push_back(2);
    tmp.vertices.push_back(0);
    tmp.vertices.push_back(3);
    pPolygons->push_back(tmp);
    updateEdgesPolygonAdd(tmp, pEdges);
    tmp.vertices.clear();

    // Top
    // First Triangle
    tmp.vertices.push_back(0+4);
    tmp.vertices.push_back(1+4);
    tmp.vertices.push_back(2+4);
    pPolygons->push_back(tmp);
    updateEdgesPolygonAdd(tmp, pEdges);
    tmp.vertices.clear();

    //Second Triangle
    tmp.vertices.push_back(2+4);
    tmp.vertices.push_back(3+4);
    tmp.vertices.push_back(0+4);
    pPolygons->push_back(tmp);
    updateEdgesPolygonAdd(tmp, pEdges);
    tmp.vertices.clear();

    //Sides
    // Two Triangles per Side
    for (uint8_t i = 0; i < 4; i++) {
      tmp.vertices.push_back((0+i)%4);
      tmp.vertices.push_back((1+i)%4);
      tmp.vertices.push_back(((1+i)%4)+4);
      pPolygons->push_back(tmp);
      updateEdgesPolygonAdd(tmp, pEdges);
      tmp.vertices.clear();

      tmp.vertices.push_back((0+i)%4);
      tmp.vertices.push_back(((1+i)%4)+4);
      tmp.vertices.push_back(((0+i)%4)+4);
      pPolygons->push_back(tmp);
      updateEdgesPolygonAdd(tmp, pEdges);
      tmp.vertices.clear();
    }
  }

  /***************************************************************************//**
   * Subdivide the bounding box mesh
   \verbatim

                 C       Delete existing Polygon ABC                 C
                 x       and replace with 4 new:                     x
                /|       A AB AC                                    /|
               / |       AB B BC                      \            / |
              /  |       AC BC C                     ==\          /  |
          AC x   x BC    AB BC AC             Result ==/      AC x---x BC
            /    |                                    /         /|  /|
           /     |                                             / | / |
          /      |                                            /  |/  |
       A x---x---x B                                       A x---x---x B
             AB                                                  AB
       New Edges:        Initial triangle count: 12
       AC <-> AB         triangle count after n iterations: 12 * 4^n
       AC <-> BC
       AB <-> BC
   \endverbatim
   *
   * @param pPolygons the polygons vector of the mesh
   * @param pEdges The edges vector of the mesh
   * @param pCloud The coordinates of the vertices of the mesh
   * @param pIterations The number of subdivisions to calculate
   *
   ******************************************************************************/
  void PCLShrinkWrappingNodelet::subDivideBoundingBox(boost::shared_ptr<std::vector<pcl::Vertices>> pPolygons,
                                                      boost::shared_ptr<std::vector<pcl::Vertices>> pEdges,
                                                      pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud,
                                                      unsigned int pIterations){

    // struct to keep track which middle-points were already calculated and added to the point-cloud
    struct PointRelations{
      bool operator==(const PointRelations& compare){
        return ((Point_1 == compare.Point_1) && (Point_2 == compare.Point_2))||((Point_1 == compare.Point_2)&&(Point_2 == compare.Point_1));
      }

      int32_t Point_1 = -1;
      int32_t Point_2 = -1;
      int32_t Middle = -1;
    };


    for(uint32_t Iterations = 0; Iterations < pIterations; Iterations++){
      std::vector<PointRelations> calculatedMiddlePoints;

      // Replace all Polygons (Triangles) with four sub-polygons
      for(int32_t Polygon_ID = pPolygons->size()-1; Polygon_ID >= 0; Polygon_ID--){

        int32_t A = pPolygons->at(Polygon_ID).vertices.at(0);
        int32_t B = pPolygons->at(Polygon_ID).vertices.at(1);
        int32_t C = pPolygons->at(Polygon_ID).vertices.at(2);

        //Delete old polygon
        updateEdgesPolygonRemove(pPolygons->at(Polygon_ID), pEdges);
        pPolygons->erase(pPolygons->begin() + Polygon_ID);

        PointRelations AB;
        AB.Point_1 = A;
        AB.Point_2 = B;

        PointRelations AC;
        AC.Point_1 = A;
        AC.Point_2 = C;

        PointRelations BC;
        BC.Point_1 = B;
        BC.Point_2 = C;

        // Check for already calculated middle-points
        // Probably not as efficient as recalculate the middle-point
        // but this would lead to duplicated points in the point cloud
        // Because this function is only called once to initialize the
        // nodelet, this do not have to be super efficient
        for(int32_t i = calculatedMiddlePoints.size()-1; i >= 0; i--){
          if ((AB.Middle == -1) && (calculatedMiddlePoints[i] == AB)) {
            AB.Middle = calculatedMiddlePoints[i].Middle;
            calculatedMiddlePoints.erase(calculatedMiddlePoints.begin() + i);
          }

          if((AC.Middle == -1) && (calculatedMiddlePoints[i] == AC)) {
            AC.Middle = calculatedMiddlePoints[i].Middle;
            calculatedMiddlePoints.erase(calculatedMiddlePoints.begin() + i);
          }

          if((BC.Middle == -1) && (calculatedMiddlePoints[i] == BC)) {
            BC.Middle = calculatedMiddlePoints[i].Middle;
            calculatedMiddlePoints.erase(calculatedMiddlePoints.begin() + i);
          }
        }

        // Calculate middle-points if needed
        if(AB.Middle == -1){
          pCloud->push_back(pcl::PointXYZ((pCloud->points[A].x+pCloud->points[B].x) / 2,
                                          (pCloud->points[A].y+pCloud->points[B].y) / 2,
                                          (pCloud->points[A].z+pCloud->points[B].z) / 2));
          pEdges->push_back(pcl::Vertices());
          AB.Middle = pCloud->points.size() - 1;
          calculatedMiddlePoints.push_back(AB);
        }

        if(AC.Middle == -1){
          pCloud->push_back(pcl::PointXYZ((pCloud->points[A].x+pCloud->points[C].x) / 2,
                                          (pCloud->points[A].y+pCloud->points[C].y) / 2,
                                          (pCloud->points[A].z+pCloud->points[C].z) / 2));
          pEdges->push_back(pcl::Vertices());
          AC.Middle = pCloud->points.size() - 1;
          calculatedMiddlePoints.push_back(AC);
        }

        if (BC.Middle == -1) {
          pCloud->push_back(pcl::PointXYZ((pCloud->points[B].x + pCloud->points[C].x) / 2,
                                          (pCloud->points[B].y + pCloud->points[C].y) / 2,
                                          (pCloud->points[B].z + pCloud->points[C].z) / 2));
          pEdges->push_back(pcl::Vertices());
          BC.Middle = pCloud->points.size() - 1;
          calculatedMiddlePoints.push_back(BC);
        }

        // Add new polygons
        pcl::Vertices tmp;

        // A AB AC
        tmp.vertices.push_back(A);
        tmp.vertices.push_back(AB.Middle);
        tmp.vertices.push_back(AC.Middle);
        pPolygons->push_back(tmp);
        updateEdgesPolygonAdd(tmp, pEdges);
        tmp.vertices.clear();

        // AB B BC
        tmp.vertices.push_back(AB.Middle);
        tmp.vertices.push_back(B);
        tmp.vertices.push_back(BC.Middle);
        pPolygons->push_back(tmp);
        updateEdgesPolygonAdd(tmp, pEdges);
        tmp.vertices.clear();

        // AC BC C
        tmp.vertices.push_back(AC.Middle);
        tmp.vertices.push_back(BC.Middle);
        tmp.vertices.push_back(C);
        pPolygons->push_back(tmp);
        updateEdgesPolygonAdd(tmp, pEdges);
        tmp.vertices.clear();

        // AB BC AC
        tmp.vertices.push_back(AB.Middle);
        tmp.vertices.push_back(BC.Middle);
        tmp.vertices.push_back(AC.Middle);
        pPolygons->push_back(tmp);
        updateEdgesPolygonAdd(tmp, pEdges);
        tmp.vertices.clear();
      }
    }
  }

  /***************************************************************************//**
   * Transformation of a point cloud (mesh coordinates) to fit to a given PCL orientated bounding box
   *
   * @param pMinOBB pointer to the point with the smallest coordinate values of the bounding box (without transformations)
   * @param pMaxOBB pointer to the point with the highest coordinate values of the bounding box (without transformation)
   * @param pRot pointer to the rotation matrix of the bounding box
   * @param pTrans pointer to the transformation matrix of the bounding box
   * @param pCloud shared pointer to the point cloud to be transformed
   *
   ******************************************************************************/
  void PCLShrinkWrappingNodelet::transformationToGivenOBB(pcl::PointXYZ *pMinOBB,
                                                          pcl::PointXYZ *pMaxOBB,
                                                          Eigen::Matrix3f *pRot,
                                                          pcl::PointXYZ *pTrans,
                                                          pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud){

    // transform the copy of the initial bounding-box-mesh to fit the given sample cloud

    //Scaling
    Eigen::Matrix4f transMatrix = Eigen::Matrix4f::Identity();
    transMatrix(0,0) = transMatrix(0,0) * (pMaxOBB->x - pMinOBB->x);
    transMatrix(1,1) = transMatrix(1,1) * (pMaxOBB->y - pMinOBB->y);
    transMatrix(2,2) = transMatrix(2,2) * (pMaxOBB->z - pMinOBB->z);

    pcl::transformPointCloud(*pCloud, *pCloud, transMatrix);

    // Transformation
    transMatrix = Eigen::Matrix4f::Identity();
    transMatrix.block<3,3>(0,0) = *pRot;
    Eigen::Vector3f position(pTrans->x, pTrans->y, pTrans->z);
    transMatrix.block<3,1>(0,3) = position;

    pcl::transformPointCloud(*pCloud, *pCloud, transMatrix);

    transMatrix.Identity();
  }

  /***************************************************************************//**
   * Thread created by the callback function.
   * Creates a meshing thread for every object
   *
   * @param message The message received from ROS as ConstPtr aka boost::shared_pointer
   *
   ******************************************************************************/
  void PCLShrinkWrappingNodelet::threadHandler(const pcl_filter::CloudClustersConstPtr message){

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    // Read the ROS-Message
    // convert point-cloud in pcl-format
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(message->pointCloud, *cloud);

    //convert cluster_indices in pcl-format
    std::vector<boost::shared_ptr<pcl::PointIndices>> cluster_indices;
    for (int i = 0; i < message->num_cluster; i++) {
      boost::shared_ptr<pcl::PointIndices> tmp_indices (new pcl::PointIndices);
      pcl_conversions::toPCL(message->cluster_indices[i], *tmp_indices);
      cluster_indices.push_back(tmp_indices);
    }

    //prepare ans start threads for every object
    std::vector<std::thread> workers;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> meshClouds;
    std::vector<boost::shared_ptr<std::vector<pcl::Vertices>>> polygonsVector;
    std::vector<boost::shared_ptr<std::vector<pcl::Vertices>>> edgesVector;

    if(message->mirror_exists){
      for (int i = 0; i < message->num_cluster; i++) {

        meshClouds.push_back(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>));
        polygonsVector.push_back(boost::shared_ptr<std::vector<pcl::Vertices>>(new std::vector<pcl::Vertices>));
        edgesVector.push_back(boost::shared_ptr<std::vector<pcl::Vertices>>(new std::vector<pcl::Vertices>));

        workers.push_back(std::thread(&PCLShrinkWrappingNodelet::generateMeshThread_Mirror,
                                      this,
                                      cloud,
                                      cluster_indices[i],
                                      meshClouds[meshClouds.size() - 1],
                                      polygonsVector[polygonsVector.size() - 1],
                                      edgesVector[edgesVector.size() - 1],
                                      message->mirrorCloud.at(i)));
      }
    }else{
      for (int i = 0; i < message->num_cluster; i++) {

        meshClouds.push_back(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>));
        polygonsVector.push_back(boost::shared_ptr<std::vector<pcl::Vertices>>(new std::vector<pcl::Vertices>));
        edgesVector.push_back(boost::shared_ptr<std::vector<pcl::Vertices>>(new std::vector<pcl::Vertices>));

        workers.push_back(std::thread(&PCLShrinkWrappingNodelet::generateMeshThread,
                                      this,
                                      cloud,
                                      cluster_indices[i],
                                      meshClouds[meshClouds.size() - 1],
                                      polygonsVector[polygonsVector.size() - 1],
                                      edgesVector[edgesVector.size() - 1]));
      }
    }
    // wait for all threads to finish
    std::for_each(workers.begin(), workers.end(), [](std::thread &t){ t.join(); });

    publish(meshClouds, polygonsVector);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    NODELET_INFO("Meshes published! Calculation took %3.3f [ms]", std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count() / 1000000.0f);

    // write time into a file
    std::ofstream file;
    file.open("/home/hiwi/Desktop/time.csv",std::ofstream::app);
    file << (std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count() / 1000000.0f) << "\n";
    file.close();
  }

  /***************************************************************************//**
   * Meshing thread for one object without a mirror cloud
   *
   * @param pClusterCloud The whole point cloud
   * @param pCluster The indices of the points of the object cloud
   * @param pMeshCloud The pointer to the output mesh coordinates
   * @param pPolygons The pointer to the output mesh polygons vector
   * @param pEdges The pointer to the output mesh edges vector
   *
   ******************************************************************************/
  void PCLShrinkWrappingNodelet::generateMeshThread(pcl::PointCloud<pcl::PointXYZ>::Ptr pClusterCloud,
                                                    boost::shared_ptr<pcl::PointIndices> pCluster,
                                                    pcl::PointCloud<pcl::PointXYZ>::Ptr pMeshCloud,
                                                    boost::shared_ptr<std::vector<pcl::Vertices> > pPolygons,
                                                    boost::shared_ptr<std::vector<pcl::Vertices> > pEdges){

    // copy the object point cloud from the whole cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr sampleCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*pClusterCloud, *pCluster, *sampleCloud);

    generateMesh(sampleCloud, pMeshCloud, pPolygons, pEdges);

  }

  /***************************************************************************//**
   * Meshing thread for one object with a mirror cloud
   *
   * @param pClusterCloud The whole point cloud
   * @param pCluster The indices of the points of the object cloud
   * @param pMeshCloud The pointer to the output mesh coordinates
   * @param pPolygons The pointer to the output mesh polygons vector
   * @param pEdges The pointer to the output mesh edges vector
   * @param mirrorClouds The points of the mirror cloud for this object
   *
   ******************************************************************************/
  void PCLShrinkWrappingNodelet::generateMeshThread_Mirror(pcl::PointCloud<pcl::PointXYZ>::Ptr pClusterCloud,
                                                           boost::shared_ptr<pcl::PointIndices> pCluster,
                                                           pcl::PointCloud<pcl::PointXYZ>::Ptr pMeshCloud,
                                                           boost::shared_ptr<std::vector<pcl::Vertices> > pPolygons,
                                                           boost::shared_ptr<std::vector<pcl::Vertices> > pEdges,
                                                           sensor_msgs::PointCloud2 mirrorClouds){

    // copy the object point cloud from the whole cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr sampleCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*pClusterCloud, *pCluster, *sampleCloud);

    // add the mirror cloud to the sample cloud
    pcl::PointCloud<pcl::PointXYZ> tmpCloud;
    pcl::fromROSMsg(mirrorClouds,tmpCloud);
    *sampleCloud += tmpCloud;

    generateMesh(sampleCloud, pMeshCloud, pPolygons, pEdges);
  }

  /***************************************************************************//**
   * Meshing function for on object
   * Including the optional bounding mesh algorithm
   *
   * @param pSampleCloud The point cloud for one object
   * @param pMeshCloud The pointer to the output mesh coordinates
   * @param pPolygons The pointer to the output mesh polygons vector
   * @param pEdges The pointer to the output mesh edges vector
   *
   ******************************************************************************/
  void PCLShrinkWrappingNodelet::generateMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr pSampleCloud,
                                              pcl::PointCloud<pcl::PointXYZ>::Ptr pMeshCloud,
                                              boost::shared_ptr<std::vector<pcl::Vertices> > pPolygons,
                                              boost::shared_ptr<std::vector<pcl::Vertices> > pEdges){
    // get copy of initial Bounding Box
    pcl::copyPointCloud(*initialCloud, *pMeshCloud);
    *pPolygons = *initialPolygons;
    *pEdges = *initialEdges;
    boost::shared_ptr<std::vector<pcl::PointXYZ>> normals (new std::vector<pcl::PointXYZ>);

    // prepare Feature extraction
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(pSampleCloud);
    feature_extractor.compute();

    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;

    // get Features of Bounding Box
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

    // transform initial Bounding Box to the sample Bounding Box features
    transformationToGivenOBB(&min_point_OBB, &max_point_OBB, &rotational_matrix_OBB, &position_OBB, pMeshCloud);

    // apply Shrink-Wrapping operations
    //for (int i = 0; i < 10; i++) {
    projectionOperation(pSampleCloud, pMeshCloud, normals);
    smoothingOperation(pMeshCloud, pEdges, normals);
    //}

    //******************************************************************************//
    //bounding Mesh
    //******************************************************************************//

    if(useBoundingMesh){
      std::shared_ptr<boundingmesh::Decimator> bmDecimator = std::make_shared<boundingmesh::Decimator>();
      bmDecimator->setDirection(boundingmesh::Outward);
      bmDecimator->setMetric(boundingmesh::Average);
      bmDecimator->setMaximumError(1.0);
      std::shared_ptr<boundingmesh::Mesh> bmMesh (new boundingmesh::Mesh);

      std::for_each(pMeshCloud->points.begin(),pMeshCloud->points.end(), [&](pcl::PointXYZ &p) {
          bmMesh->addVertex(boundingmesh::Vector3(p.x,p.y,p.z) );
        });

      std::for_each(pPolygons->begin(),pPolygons->end(), [&](pcl::Vertices &v) {
          bmMesh->addTriangle(v.vertices[0],v.vertices[1],v.vertices[2]);
        });


      //************************************//
      // Execute the algorithm
      bmDecimator->setMesh(*bmMesh);
      while((bmDecimator->nextError() < 0.005) /*|| (bmDecimator->getMesh()->nTriangles() > 10)*/){
        bmDecimator->doContractions();
      }
      //***********************************//
      bmMesh = bmDecimator->getMesh();
      NODELET_WARN("%i",bmMesh->nTriangles());
      pMeshCloud->points.clear();
      for(int i = 0; i < bmMesh->nVertices(); i++){

        pcl::PointXYZ tmpPoint(bmMesh->vertex(i).position()(0,0),
                               bmMesh->vertex(i).position()(1,0),
                               bmMesh->vertex(i).position()(2,0));

        pMeshCloud->points.push_back(tmpPoint);
      }

      pPolygons->clear();
      for (int i = 0; i < bmMesh->nTriangles(); i++) {
        pcl::Vertices tmpVert;
        tmpVert.vertices.push_back(bmMesh->triangle(i).vertex(0));
        tmpVert.vertices.push_back(bmMesh->triangle(i).vertex(1));
        tmpVert.vertices.push_back(bmMesh->triangle(i).vertex(2));
        pPolygons->push_back(tmpVert);
      }
    }
  }

  /***************************************************************************//**
   * Projection operation of the shrink-wrapping algorithm
   *
   * @param pSampleCloud The object point cloud
   * @param pMeshCloud The pointer to the output mesh coordinates
   * @param pNormals Output parameter for the moving direction of the mesh vertices used as normal approximations in the smoothing operation
   *
   ******************************************************************************/
  void PCLShrinkWrappingNodelet::projectionOperation(pcl::PointCloud<pcl::PointXYZ>::Ptr pSampleCloud,
                                                     pcl::PointCloud<pcl::PointXYZ>::Ptr pMeshCloud,
                                                     boost::shared_ptr<std::vector<pcl::PointXYZ>> pNormals){

    // create KdTree for nearest neighbor search
    pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;

    if(pSampleCloud->points.size() > 0){
      kdTree.setInputCloud(pSampleCloud);

      pNormals->resize(pMeshCloud->points.size());

      // search nearest neighbor for every point in MeshCloud
      for (uint32_t i = 0; i < pMeshCloud->points.size(); i++) {
        std::vector<int> pointIdNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);

        kdTree.nearestKSearch(pMeshCloud->points.at(i), 1, pointIdNKNSearch, pointNKNSquaredDistance);

        // calculate connection Vector between MeshPoint and nearest SamplePoint
        pNormals->at(i).x = pMeshCloud->points.at(i).x -
          pSampleCloud->points.at(pointIdNKNSearch[0]).x;
        pNormals->at(i).y = pMeshCloud->points.at(i).y -
          pSampleCloud->points.at(pointIdNKNSearch[0]).y;
        pNormals->at(i).z = pMeshCloud->points.at(i).z -
          pSampleCloud->points.at(pointIdNKNSearch[0]).z;

        // move MeshPoint
        pMeshCloud->points.at(i).x -= (pNormals->at(i).x) * 0.85f;// * 0.1f;
        pMeshCloud->points.at(i).y -= (pNormals->at(i).y) * 0.85f;// * 0.1f;
        pMeshCloud->points.at(i).z -= (pNormals->at(i).z) * 0.85f;// * 0.1f;

        float x = pNormals->at(i).x;
        float y = pNormals->at(i).y;
        float z = pNormals->at(i).z;

        if(pointNKNSquaredDistance.at(0) != 0.0f){
          // calculate normalized Normal-Vector for next operation
          pNormals->at(i).x /= sqrt(pointNKNSquaredDistance.at(0));
          pNormals->at(i).y /= sqrt(pointNKNSquaredDistance.at(0));
          pNormals->at(i).z /= sqrt(pointNKNSquaredDistance.at(0));
        }
        else
          {
            NODELET_WARN("mesh-point exactly on sample-point");
          }
      }
    }
    else
      NODELET_WARN("pSampleCloud in projectionOperation was empty. That should not happen. Investigate!");
  }

  /***************************************************************************//**
   * Smoothing operation of the shrink-wrapping algorithm
   *
   * @param pMeshCloud The pointer to the output mesh coordinates
   * @param pEdges The pointer to the output edges vector
   * @param pNormals Approximations for normals calculated in the projection operation
   *
   ******************************************************************************/
  void PCLShrinkWrappingNodelet::smoothingOperation(pcl::PointCloud<pcl::PointXYZ>::Ptr pMeshCloud,
                                                    boost::shared_ptr<std::vector<pcl::Vertices> > pEdges,
                                                    boost::shared_ptr<std::vector<pcl::PointXYZ> > pNormals){
    for(int i = 0; i < pMeshCloud->points.size(); i++){

      // calculate laplacian vector (average of all edge vectors connected to this MeshPoint)
      pcl::PointXYZ laplacian_vector(0,0,0);
      for(int j = 0; j < pEdges->at(i).vertices.size(); j++){

        laplacian_vector.x += pMeshCloud->points.at(pEdges->at(i).vertices.at(j)).x -  pMeshCloud->points.at(i).x;
        laplacian_vector.y += pMeshCloud->points.at(pEdges->at(i).vertices.at(j)).y -  pMeshCloud->points.at(i).y;
        laplacian_vector.z += pMeshCloud->points.at(pEdges->at(i).vertices.at(j)).z -  pMeshCloud->points.at(i).z;
      }

      laplacian_vector.x /= pEdges->at(i).vertices.size();
      laplacian_vector.y /= pEdges->at(i).vertices.size();
      laplacian_vector.z /= pEdges->at(i).vertices.size();

      float DotProduct = (laplacian_vector.x * pNormals->at(i).x) * (laplacian_vector.y * pNormals->at(i).y) * (laplacian_vector.z * pNormals->at(i).z);

      // calculate tangential laplacian vector
      pcl::PointXYZ tangentialLaplaceVector(0,0,0);

      tangentialLaplaceVector.x = laplacian_vector.x - (DotProduct * pNormals->at(i).x);
      tangentialLaplaceVector.y = laplacian_vector.y - (DotProduct * pNormals->at(i).y);
      tangentialLaplaceVector.z = laplacian_vector.z - (DotProduct * pNormals->at(i).z);

      // move MeshPoint
      pMeshCloud->points.at(i).x += tangentialLaplaceVector.x * 0.2;// * 0.1f;
      pMeshCloud->points.at(i).y += tangentialLaplaceVector.y * 0.2;// * 0.1f;
      pMeshCloud->points.at(i).z += tangentialLaplaceVector.z * 0.2;// * 0.1f;
    }

  }


  /***************************************************************************//**
   * Publisher function. Publishes results for visualization purposes to rviz
   *
   * @param pMeshClouds The pointer to the output mesh coordinates
   * @param pPolygonsVector The pointer to the output polygons vector
   *
   ******************************************************************************/
  void PCLShrinkWrappingNodelet::publish(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pMeshClouds,
                                         std::vector<boost::shared_ptr<std::vector<pcl::Vertices>>> pPolygonsVector){

    visualization_msgs::Marker marker;
    marker.header.frame_id = "camera_depth_optical_frame";
    marker.header.stamp = ros::Time();
    marker.ns = "Meshes";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    if(pMeshClouds.size() == pPolygonsVector.size()){

      for(int CloudIndex = 0; CloudIndex < pMeshClouds.size(); CloudIndex++){

        std::for_each(pPolygonsVector.at(CloudIndex)->begin(), pPolygonsVector.at(CloudIndex)->end(), [&](pcl::Vertices &v) {
            std::for_each(v.vertices.begin(), v.vertices.end(), [&](int index) {
                geometry_msgs::Point point;
                point.x = pMeshClouds.at(CloudIndex)->points.at(index).x;
                point.y = pMeshClouds.at(CloudIndex)->points.at(index).y;
                point.z = pMeshClouds.at(CloudIndex)->points.at(index).z;
                marker.points.push_back(point);
              });
          });
      }

      marker.pose.position.x = 0;
      marker.pose.position.y = 0;
      marker.pose.position.z = 0;

      marker.pose.orientation.x = 0;
      marker.pose.orientation.y = 0;
      marker.pose.orientation.z = 0;
      marker.pose.orientation.w = 1;

      marker.scale.x = 1;
      marker.scale.y = 1;
      marker.scale.z = 1;

      marker.color.a = 1;
      marker.color.r = 1;
      marker.color.g = 0;
      marker.color.b = 0;


      pub_.publish(marker);

    }else{
      NODELET_WARN("Oh this seems to be wrong... MeshCloud-Vector and Polygon-Vector have a different size. Could not publish meshes");
    }
  }
}
