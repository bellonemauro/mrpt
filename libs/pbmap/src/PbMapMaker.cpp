/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

/*  Plane-based Map (PbMap) library
 *  Construction of plane-based maps and localization in it from RGBD Images.
 *  Writen by Eduardo Fernandez-Moral. See docs for <a href="group__mrpt__pbmap__grp.html" >mrpt-pbmap</a>
 */

#include <mrpt/pbmap.h> // precomp. hdr

#if MRPT_HAS_PCL


#include <mrpt/base.h>
#include <mrpt/system/threads.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
//#include <pcl/segmentation/planar_region.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/filters/extract_indices.h>
//#include <pcl/surface/convex_hull.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
//#include <pcl/geometry/polygon_operations.h>
#include <pcl/common/time.h>

#include <iostream>

//#include <mrpt/pbmap/PbMapMaker.h>
//#include <mrpt/pbmap/Miscellaneous.h>

using namespace std;
using namespace Eigen;
using namespace mrpt::pbmap;

mrpt::synch::CCriticalSection CS_visualize;

/*!Some parameters to specify input/output and some thresholds.*/
struct config_pbmap
{
  // [global]
  bool input_from_rawlog;
  std::string rawlog_path;
  bool record_rawlog;
  bool use_color;
  float color_threshold;

  // [plane_segmentation]
  float dist_threshold; // Maximum distance to the plane between neighbor 3D-points
  float angle_threshold; //  = 0.017453 * 4.0 // Maximum angle between contiguous 3D-points
  float minInliersRate; // Minimum ratio of inliers/image points required

  // [map_construction]
  float proximity_neighbor_planes;  // Two planar patches are considered neighbors when the closest distance between them is under proximity_neighbor_planes
//  float max_angle_normals; // (10º) Two planar patches that represent the same surface must have similar normals // QUITAR
  float max_cos_normal;
  float max_dist_center_plane; // Two planar patches that represent the same surface must have their center in the same plane
  float proximity_threshold;  // Two planar patches that represent the same surface must overlap or be nearby
  bool  graph_rule;  // This var selects the condition to create edges in the graph, either proximity of planar patches or co-visibility in a single frame

  // [semantics]
  bool inferStructure;    // Infer if the planes correspond to the floor, ceiling or walls
  bool makeCovisibilityClusters; // Should the PbMapMaker cluster the planes according to their co-visibility

  // [localisation]
  bool detect_loopClosure;             // Run PbMapLocaliser in a different threads to detect loop closures or preloaded PbMaps
  float color_THRESHOLD;
  float area_THRESHOLD;
  float elongation_THRESHOLD;
//  std::string path_prev_pbmap;

  // [serialize]
  std::string path_save_pbmap;
  bool save_registered_cloud;
  std::string path_save_registered_cloud;

//  // [visualization]
//  bool visualizeIntensity;
//
//  // [debug]
//  bool verbose;  // Prints on screen some variables to debug the program

} configPbMap;

void readConfigFile()
{
  mrpt::utils::CConfigFile config_file("/home/edu/Projects/PbMapLocalizer/pbmap/configPbMap.ini");

  // global
//  configPbMap.input_from_rawlog = config_file.read_bool("global","input_from_rawlog",false);
//  if(configPbMap.input_from_rawlog)
//    configPbMap.rawlog_path = config_file.read_string("global","rawlog_path","");
//  else
//    configPbMap.record_rawlog = config_file.read_bool("global","record_rawlog",false);
  configPbMap.use_color = config_file.read_bool("global","use_color",false);
//std::cout << "use_color " << configPbMap.use_color << std::endl;

  // Plane segmentation
  configPbMap.dist_threshold = config_file.read_float("plane_segmentation","dist_threshold",0.04,true);
  configPbMap.angle_threshold = config_file.read_float("plane_segmentation","angle_threshold",0.069812,true);
  configPbMap.minInliersRate = config_file.read_float("plane_segmentation","minInliersRate",0.01,true);

  // map_construction
//  configPbMap.max_angle_normals = config_file.read_float("map_construction","max_angle_normals",0.17453,true);
  configPbMap.max_cos_normal = config_file.read_float("map_construction","max_cos_normal",0.9848,true);
  configPbMap.max_dist_center_plane = config_file.read_float("map_construction","max_dist_center_plane",0.1,true);
  configPbMap.proximity_threshold = config_file.read_float("map_construction","proximity_threshold",0.15);
  configPbMap.proximity_neighbor_planes = config_file.read_float("map_construction","proximity_neighbor_planes",1.0);
  configPbMap.graph_rule = config_file.read_bool("map_construction","graph_rule",false);

  // [semantics]
  configPbMap.inferStructure = config_file.read_bool("semantics","inferStructure",true);
  configPbMap.makeCovisibilityClusters = config_file.read_bool("semantics","makeCovisibilityClusters",true);
//  configPbMap.path_prev_pbmap = config_file.read_string("localisation","path_prev_pbmap","",true);

  // [localisation]
  configPbMap.detect_loopClosure = config_file.read_bool("localisation","detect_loopClosure",true);
  configPbMap.color_THRESHOLD = config_file.read_float("localisation","color_THRESHOLD",0.2);
  configPbMap.area_THRESHOLD = config_file.read_float("localisation","area_THRESHOLD",4.0);
  configPbMap.elongation_THRESHOLD = config_file.read_float("localisation","elongation_THRESHOLD",3.0);

  // serialize
  configPbMap.path_save_pbmap = config_file.read_string("serialize","path_save_pbmap","map");
  configPbMap.save_registered_cloud = config_file.read_bool("serialize","save_registered_cloud",true);
  configPbMap.path_save_registered_cloud = config_file.read_string("serialize","path_save_registered_cloud","/home/edu/Projects/PbMaps/PbMaps.txt");
//cout << "path_save_registered_cloud " << configPbMap.path_save_registered_cloud << endl;

//  // [visualization]
//  configPbMap.visualizeIntensity = config_file.read_bool("visualization","visualizeIntensity",false);
//
//  // [debug]
//  configPbMap.verbose = config_file.read_bool("debug","verbose",false);

  #ifdef _VERBOSE
    cout << "readConfigFile configPbMap.ini dist_threshold " << configPbMap.dist_threshold << endl;
  #endif
}

PbMapMaker::PbMapMaker() :
    cloudViewer("Cloud Viewer"),
    mpPbMapLocaliser(NULL),
    m_pbmaker_must_stop(false),
    m_pbmaker_finished(false)
{
//  GVars3::GUI.RegisterCommand("SavePbMap", GUICommandCallBack, this);

  // Load parameters
  readConfigFile();

  mpPlaneInferInfo = new PlaneInferredInfo(mPbMap);

  if(configPbMap.detect_loopClosure)
    mpPbMapLocaliser = new PbMapLocaliser(mPbMap);

  if(configPbMap.makeCovisibilityClusters)
    clusterize = new SemanticClustering(mPbMap);

//  mrpt::system::createThread (&run, T param);
  pbmaker_hd = mrpt::system::createThreadFromObjectMethod(this,&PbMapMaker::run);
}

bool PbMapMaker::arePlanesNearby(Plane &plane1, Plane &plane2, const float distThreshold)
{
  float distThres2 = distThreshold * distThreshold;

  // First we check distances between centroids and vertex to accelerate this check
  if( (plane1.v3center - plane2.v3center).squaredNorm() < distThres2 )
    return true;

  for(unsigned i=1; i < plane1.polygonContourPtr->size(); i++)
    if( (getVector3fromPointXYZ(plane1.polygonContourPtr->points[i]) - plane2.v3center).squaredNorm() < distThres2 )
      return true;

  for(unsigned j=1; j < plane2.polygonContourPtr->size(); j++)
    if( (plane1.v3center - getVector3fromPointXYZ(plane2.polygonContourPtr->points[j]) ).squaredNorm() < distThres2 )
      return true;

  for(unsigned i=1; i < plane1.polygonContourPtr->size(); i++)
    for(unsigned j=1; j < plane2.polygonContourPtr->size(); j++)
      if( (diffPoints(plane1.polygonContourPtr->points[i], plane2.polygonContourPtr->points[j]) ).squaredNorm() < distThres2 )
        return true;

  //If not found yet, search properly by checking distances:
  // a) Between an edge and a vertex
  // b) Between two edges (imagine two polygons on perpendicular planes)
  // c) Between a vertex and the inside of the polygon
  // d) Or the polygons intersect

  // a) & b)
  for(unsigned i=1; i < plane1.polygonContourPtr->size(); i++)
    for(unsigned j=1; j < plane2.polygonContourPtr->size(); j++)
      if(dist3D_Segment_to_Segment2(Segment(plane1.polygonContourPtr->points[i],plane1.polygonContourPtr->points[i-1]), Segment(plane2.polygonContourPtr->points[j],plane2.polygonContourPtr->points[j-1])) < distThres2)
        return true;

  // c)
  for(unsigned i=1; i < plane1.polygonContourPtr->size(); i++)
    if( plane2.v3normal .dot (getVector3fromPointXYZ(plane1.polygonContourPtr->points[i]) - plane2.v3center) < distThreshold )
      if(isInHull(plane1.polygonContourPtr->points[i], plane2.polygonContourPtr) )
        return true;

  for(unsigned j=1; j < plane2.polygonContourPtr->size(); j++)
    if( plane1.v3normal .dot (getVector3fromPointXYZ(plane2.polygonContourPtr->points[j]) - plane1.v3center) < distThreshold )
      if(isInHull(plane2.polygonContourPtr->points[j], plane1.polygonContourPtr) )
        return true;

  // d)
  for(unsigned i=1; i < plane1.polygonContourPtr->size(); i++)
    if( plane2.v3normal .dot (getVector3fromPointXYZ(plane1.polygonContourPtr->points[i]) - plane2.v3center) < distThreshold )
    {
//    cout << "Elements\n" << plane2.v3normal << "\n xyz " << plane1.polygonContourPtr->points[i].x << " " << plane1.polygonContourPtr->points[i].y << " " << plane1.polygonContourPtr->points[i].z
//          << " xyz2 " << plane1.polygonContourPtr->points[i-1].x << " " << plane1.polygonContourPtr->points[i-1].y << " " << plane1.polygonContourPtr->points[i-1].z << endl;
      assert( plane2.v3normal .dot (diffPoints(plane1.polygonContourPtr->points[i], plane1.polygonContourPtr->points[i-1]) ) != 0 );
      float d = (plane2.v3normal .dot (plane2.v3center - getVector3fromPointXYZ(plane1.polygonContourPtr->points[i-1]) ) ) / (plane2.v3normal .dot (diffPoints(plane1.polygonContourPtr->points[i], plane1.polygonContourPtr->points[i-1]) ) );
      PointT intersection;
      intersection.x = d * plane1.polygonContourPtr->points[i].x + (1-d) * plane1.polygonContourPtr->points[i-1].x;
      intersection.y = d * plane1.polygonContourPtr->points[i].y + (1-d) * plane1.polygonContourPtr->points[i-1].y;
      intersection.z = d * plane1.polygonContourPtr->points[i].z + (1-d) * plane1.polygonContourPtr->points[i-1].z;
      if(isInHull(intersection, plane2.polygonContourPtr) )
        return true;
    }

  return false;
}

void PbMapMaker::checkProximity(Plane &plane, float proximity)
{
  for(unsigned i=0; i < mPbMap.vPlanes.size(); i++ )
  {
    if(plane.id == mPbMap.vPlanes[i].id)
      continue;

    if(plane.nearbyPlanes.count(mPbMap.vPlanes[i].id))
      continue;

    if(arePlanesNearby(plane, mPbMap.vPlanes[i], proximity) ) // If the planes are closer than proximity (in meters), then mark them as neighbors
    {
      plane.nearbyPlanes.insert(mPbMap.vPlanes[i].id);
      mPbMap.vPlanes[i].nearbyPlanes.insert(plane.id);
    }
  }
}


void PbMapMaker::detectPlanesCloud( pcl::PointCloud<PointT>::Ptr &pointCloudPtr_arg,
                                    Eigen::Matrix4f &poseKF,
                                    double distThreshold, double angleThreshold, double minInliersF)
{
  unsigned minInliers = minInliersF * pointCloudPtr_arg->size();

  #ifdef _VERBOSE
    cout << "detectPlanes in a cloud with " << pointCloudPtr_arg->size() << " points " << minInliers << " minInliers\n";
//    cout << "Number of pts of the clouds: ";
//    for(unsigned i=0; i < frameQueue.size(); i++)
//      cout << frameQueue[i].cloudPtr->size() << " ";
//    cout << endl;
  #endif

  pcl::PointCloud<PointT>::Ptr pointCloudPtr_arg2(new pcl::PointCloud<PointT>);
  pcl::copyPointCloud(*pointCloudPtr_arg,*pointCloudPtr_arg2);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr alignedCloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::transformPointCloud(*pointCloudPtr_arg,*alignedCloudPtr,poseKF);

  { mrpt::synch::CCriticalSectionLocker csl(&CS_visualize);
    *mPbMap.globalMapPtr += *alignedCloudPtr;
  } // End CS

  // Downsample voxel map's point cloud
  static pcl::VoxelGrid<pcl::PointXYZRGBA> grid;
  grid.setLeafSize(0.02,0.02,0.02);
  pcl::PointCloud<pcl::PointXYZRGBA> globalMap;
  grid.setInputCloud (mPbMap.globalMapPtr);
  grid.filter (globalMap);
  mPbMap.globalMapPtr->clear();
  *mPbMap.globalMapPtr = globalMap;

  pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
  ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
  ne.setMaxDepthChangeFactor (0.02f); // For VGA: 0.02f, 10.0f
  ne.setNormalSmoothingSize (5.0f);
  ne.setDepthDependentSmoothing (true);

  pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
  mps.setMinInliers (minInliers);
  mps.setAngularThreshold (angleThreshold); // (0.017453 * 2.0) // 3 degrees
  mps.setDistanceThreshold (distThreshold); //2cm

  pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
  ne.setInputCloud (pointCloudPtr_arg2);
  ne.compute (*normal_cloud);

#ifdef _VERBOSE
  double plane_extract_start = pcl::getTime ();
#endif
  mps.setInputNormals (normal_cloud);
//    mps.setInputCloud (alignedCloudPtr);
  mps.setInputCloud (pointCloudPtr_arg2);

  std::vector<pcl::PlanarRegion<PointT>, aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
  std::vector<pcl::ModelCoefficients> model_coefficients;
  std::vector<pcl::PointIndices> inlier_indices;
  pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
  std::vector<pcl::PointIndices> label_indices;
  std::vector<pcl::PointIndices> boundary_indices;
  mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);


  #ifdef _VERBOSE
    double plane_extract_end = pcl::getTime();
    std::cout << "Plane extraction took " << double (plane_extract_end - plane_extract_start) << std::endl;
//    std::cout << "Frame took " << double (plane_extract_end - normal_start) << std::endl;
    cout << regions.size() << " planes detected\n";
  #endif

  // Create a vector with the planes detected in this keyframe, and calculate their parameters (normal, center, pointclouds, etc.)
  // in the global reference
  vector<Plane> detectedPlanes;
  for (size_t i = 0; i < regions.size (); i++)
  {
    Plane plane;

    Vector3f centroid = regions[i].getCentroid ();
    plane.v3center = compose(poseKF, centroid);
    plane.v3normal = poseKF.block(0,0,3,3) * Vector3f(model_coefficients[i].values[0], model_coefficients[i].values[1], model_coefficients[i].values[2]);

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
    extract.setInputCloud (pointCloudPtr_arg2);
    extract.setIndices ( boost::make_shared<const pcl::PointIndices> (inlier_indices[i]) );
    extract.setNegative (false);
    extract.filter (*plane.planePointCloudPtr);    // Write the planar point cloud

    static pcl::VoxelGrid<pcl::PointXYZRGBA> plane_grid;
    plane_grid.setLeafSize(0.05,0.05,0.05);
    pcl::PointCloud<pcl::PointXYZRGBA> planeCloud;
    plane_grid.setInputCloud (plane.planePointCloudPtr);
    plane_grid.filter (planeCloud);
    plane.planePointCloudPtr->clear();
    pcl::transformPointCloud(planeCloud,*plane.planePointCloudPtr,poseKF);

    plane.contourPtr->points = regions[i].getContour();
    pcl::transformPointCloud(*plane.contourPtr,*plane.polygonContourPtr,poseKF);
    plane_grid.setInputCloud (plane.polygonContourPtr);
    plane_grid.filter (*plane.contourPtr);

    plane.calcConvexHull(plane.contourPtr);
    plane.computeMassCenterAndArea();
    plane.areaVoxels= plane.planePointCloudPtr->size() * 0.0025;

    #ifdef _VERBOSE
      cout << "Area plane region " << plane.areaVoxels<< " of Chull " << plane.areaHull << " of polygon " << plane.compute2DPolygonalArea() << endl;
    #endif

    // Check whether this region correspond to the same plane as a previous one (this situation may happen when there exists a small discontinuity in the observation)
    bool isSamePlane = false;
    for (size_t j = 0; j < detectedPlanes.size(); j++)
      if( areSamePlane(detectedPlanes[j], plane, configPbMap.max_cos_normal, configPbMap.max_dist_center_plane, configPbMap.proximity_threshold) ) // The planes are merged if they are the same
      {
        isSamePlane = true;

        mergePlanes(detectedPlanes[j], plane);

        #ifdef _VERBOSE
          cout << "\tTwo regions support the same plane in the same KeyFrame\n";
        #endif

        break;
      }
    if(!isSamePlane)
      detectedPlanes.push_back(plane);
  }

  #ifdef _VERBOSE
    cout << detectedPlanes.size () << " Planes detected\n";
  #endif

  // Merge detected planes with previous ones if they are the same
  size_t numPrevPlanes = mPbMap.vPlanes.size();
//  set<unsigned> observedPlanes;
  observedPlanes.clear();
  for (size_t i = 0; i < detectedPlanes.size (); i++)
  {
    // Check similarity with previous planes detected
    bool isSamePlane = false;
    for(size_t j = 0; j < numPrevPlanes; j++) // numPrevPlanes
    {
      if( areSamePlane(mPbMap.vPlanes[j], detectedPlanes[i], configPbMap.max_cos_normal, configPbMap.max_dist_center_plane, configPbMap.proximity_threshold) ) // The planes are merged if they are the same
      {
        isSamePlane = true;

        mergePlanes(mPbMap.vPlanes[j], detectedPlanes[i]);

        #ifdef _VERBOSE
          cout << "Previous plane " << mPbMap.vPlanes[j].id << " area " << mPbMap.vPlanes[j].areaVoxels<< " of polygon " << mPbMap.vPlanes[j].compute2DPolygonalArea() << endl;
        #endif

        if( observedPlanes.count(mPbMap.vPlanes[j].id) ) // If this plane has already been observed through a previous partial plane in this same keyframe, then we must not account twice in the observation count
          break;

        mPbMap.vPlanes[j].numObservations++;

        // Update proximity graph
        checkProximity(mPbMap.vPlanes[j], configPbMap.proximity_neighbor_planes); // Detect neighbors

        // Update co-visibility graph
        for(set<unsigned>::iterator it = observedPlanes.begin(); it != observedPlanes.end(); it++)
          if(mPbMap.vPlanes[j].neighborPlanes.count(*it))
          {
            mPbMap.vPlanes[j].neighborPlanes[*it]++;
            mPbMap.vPlanes[*it].neighborPlanes[mPbMap.vPlanes[j].id]++; // j = mPbMap.vPlanes[j]
          }
          else
          {
            mPbMap.vPlanes[j].neighborPlanes[*it] = 1;
            mPbMap.vPlanes[*it].neighborPlanes[mPbMap.vPlanes[j].id] = 1;
          }

        observedPlanes.insert(mPbMap.vPlanes[j].id);

        #ifdef _VERBOSE
          cout << "Same plane\n";
        #endif

        break;
      }
    }
    if(!isSamePlane)
    {
      detectedPlanes[i].id = mPbMap.vPlanes.size();
      detectedPlanes[i].numObservations = 1;
      detectedPlanes[i].bFullExtent = false;
      detectedPlanes[i].nFramesAreaIsStable = 0;
      detectedPlanes[i].semanticGroup = clusterize->currentSemanticGroup;
      clusterize->groups[clusterize->currentSemanticGroup].push_back(detectedPlanes[i].id);

      #ifdef _VERBOSE
        cout << "New plane " << detectedPlanes[i].id << " area " << detectedPlanes[i].areaVoxels<< " of polygon " << detectedPlanes[i].areaHull << endl;
      #endif

      // Update proximity graph
      checkProximity(detectedPlanes[i], configPbMap.proximity_neighbor_planes);  // Detect neighbors with max separation of 1.0 meters

      // Update co-visibility graph
      for(set<unsigned>::iterator it = observedPlanes.begin(); it != observedPlanes.end(); it++)
      {
        detectedPlanes[i].neighborPlanes[*it] = 1;
        mPbMap.vPlanes[*it].neighborPlanes[detectedPlanes[i].id] = 1;
      }

      observedPlanes.insert(detectedPlanes[i].id);

      mPbMap.vPlanes.push_back(detectedPlanes[i]);
    }
  }

  #ifdef _VERBOSE
    cout << "\n\tobservedPlanes: ";
    cout << observedPlanes.size () << " Planes observed\n";
    for(set<unsigned>::iterator it = observedPlanes.begin(); it != observedPlanes.end(); it++)
      cout << *it << " ";
    cout << endl;
  #endif

    // For all observed planes
    for(set<unsigned>::iterator it = observedPlanes.begin(); it != observedPlanes.end(); it++)
    {
      Plane &observedPlane = mPbMap.vPlanes[*it];

      // Calculate principal direction
      observedPlane.calcElongationAndPpalDir();

      // Infer knowledge from the planes (e.g. do these planes represent the floor, walls, etc.)
      if(configPbMap.inferStructure)
        mpPlaneInferInfo->searchTheFloor(poseKF, observedPlane);

    } // End for obsevedPlanes

    // Search the floor plane
    if(mPbMap.FloorPlane != -1) // Verify that the observed planes centers are above the floor
    {
      #ifdef _VERBOSE
        cout << "Verify that the observed planes centers are above the floor\n";
      #endif

      for(set<unsigned>::reverse_iterator it = observedPlanes.rbegin(); it != observedPlanes.rend(); it++)
      {
        if(static_cast<int>(*it) == mPbMap.FloorPlane)
          continue;
        if( mPbMap.vPlanes[mPbMap.FloorPlane].v3normal .dot (mPbMap.vPlanes[*it].v3center - mPbMap.vPlanes[mPbMap.FloorPlane].v3center) < -0.1 )
        {
          if(mPbMap.vPlanes[mPbMap.FloorPlane].v3normal .dot (mPbMap.vPlanes[*it].v3normal) > 0.99) //(cos 8.1º = 0.99)
          {
            mPbMap.vPlanes[*it].label = "Floor";
            mPbMap.vPlanes[mPbMap.FloorPlane].label = "";
            mPbMap.FloorPlane = *it;
          }
          else
          {
            assert(0);
            mPbMap.vPlanes[mPbMap.FloorPlane].label = "";
            mPbMap.FloorPlane = -1;
            break;
          }
        }
      }
    }

  if(configPbMap.detect_loopClosure)
    for(set<unsigned>::iterator it = observedPlanes.begin(); it != observedPlanes.end(); it++)
    {
      if(mpPbMapLocaliser->vQueueObservedPlanes.size() < 10)
        mpPbMapLocaliser->vQueueObservedPlanes.push_back(*it);
    }

    #ifdef _VERBOSE
      cout << "DetectedPlanesCloud finished\n";
    #endif
}


bool graphRepresentation = false;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
  if ( (event.getKeySym () == "r" || event.getKeySym () == "R") && event.keyDown ())
  {
    graphRepresentation = !graphRepresentation;
  }
}

/*!Check if the the input plane is the same than this plane for some given angle and distance thresholds.
 * If the planes are the same they are merged in this and the function returns true. Otherwise it returns false.*/
bool PbMapMaker::areSamePlane(Plane &plane1, Plane &plane2, const float &cosAngleThreshold, const float &distThreshold, const float &proxThreshold)
{
//cout << "areSamePlane...\n";
  // Check that both planes have similar orientation
//cout << "Angle between planes centers (cos) " << plane1.v3normal * plane2.v3normal << " " << cosAngleThreshold << endl;
  if( plane1.v3normal .dot (plane2.v3normal) < cosAngleThreshold )
    return false;

  // Check the normal distance of the planes centers using their average normal
//  double sumAreas = plane1.areaVoxels+ plane2.areaVoxels;
//  Vector<3> avNormal = (plane1.areaVoxels*plane1.v3normal + plane2.areaVoxels*plane2.v3normal) / sumAreas;
//  float dist_normal = avNormal * (plane1.v3center - plane2.v3center);
  float dist_normal = plane1.v3normal .dot (plane2.v3center - plane1.v3center);
//cout << "Normal Dist between planes centers " << dist_normal << " " << distThreshold << endl;
  if(fabs(dist_normal) > distThreshold ) // Then merge the planes
    return false;

  // Check that the distance between the planes centers is not too big
  if( !arePlanesNearby(plane1, plane2, proxThreshold) )
    return false;

  return true;
}

void PbMapMaker::mergePlanes(Plane &updatePlane, Plane &discardPlane)
{
  // Update normal and center
  double sumAreas = updatePlane.areaVoxels + discardPlane.areaVoxels;
  updatePlane.v3normal = (updatePlane.areaVoxels*updatePlane.v3normal + discardPlane.areaVoxels*discardPlane.v3normal) / sumAreas;

  // Update point inliers
//  *updatePlane.polygonContourPtr += *discardPlane.polygonContourPtr; // Merge polygon points
  *updatePlane.planePointCloudPtr += *discardPlane.planePointCloudPtr; // Add the points of the new detection and perform a voxel grid

  // Filter the points of the patch with a voxel-grid. This points are used only for visualization
  static pcl::VoxelGrid<pcl::PointXYZRGBA> merge_grid;
  merge_grid.setLeafSize(0.05,0.05,0.05);
  pcl::PointCloud<pcl::PointXYZRGBA> mergeCloud;
  merge_grid.setInputCloud (updatePlane.planePointCloudPtr);
  merge_grid.filter (mergeCloud);
  updatePlane.planePointCloudPtr->clear();
  *updatePlane.planePointCloudPtr = mergeCloud;

  if(configPbMap.use_color)
  {
    updatePlane.calcMainColor();
  }

  *discardPlane.polygonContourPtr += *updatePlane.planePointCloudPtr;
  updatePlane.calcConvexHull(discardPlane.polygonContourPtr);
  updatePlane.computeMassCenterAndArea();

  // Move the points to fulfill the plane equation
  updatePlane.forcePtsLayOnPlane();

  // Update area
  double area_recalc = updatePlane.planePointCloudPtr->size() * 0.0025;
  mpPlaneInferInfo->isFullExtent(updatePlane, area_recalc);
  updatePlane.areaVoxels= updatePlane.planePointCloudPtr->size() * 0.0025;

//cout << "Previous plane " << updatePlane.id << " area " << updatePlane.areaVoxels<< " of polygon " << updatePlane.compute2DPolygonalArea() << endl;
}

// Color = (red[i], grn[i], blu[i])
// The color order is: red, green, blue, yellow, pink, turquoise, orange, purple, dark green, beige
unsigned char red [10] = {255,   0,   0, 255, 255,   0, 255, 204,   0, 255};
unsigned char grn [10] = {  0, 255,   0, 255,   0, 255, 160,  51, 128, 222};
unsigned char blu [10] = {  0,   0, 255,   0, 255, 255, 0  , 204,   0, 173};

double ared [10] = {1.0,   0,   0, 1.0, 1.0,   0, 1.0, 0.8,   0, 1.0};
double agrn [10] = {  0, 1.0,   0, 1.0,   0, 1.0, 0.6, 0.2, 0.5, 0.9};
double ablu [10] = {  0,   0, 1.0,   0, 1.0, 1.0,   0, 0.8,   0, 0.7};

void PbMapMaker::viz_cb (pcl::visualization::PCLVisualizer& viz)
{
  if (mPbMap.globalMapPtr->empty())
  {
    mrpt::system::sleep(10);
    return;
  }

  { mrpt::synch::CCriticalSectionLocker csl(&CS_visualize);

    // Render the data
    {
//      viz.spinOnce(100);
//      viz.setUseVbos (true);
//      viz.setSize (840, 740);
      viz.removeAllShapes();
      viz.removeAllPointClouds();

      char name[1024];

      if(graphRepresentation)
      {
//      cout << "show graphRepresentation\n";
        for(size_t i=0; i<mPbMap.vPlanes.size(); i++)
        {
          pcl::PointXYZ center(2*mPbMap.vPlanes[i].v3center[0], 2*mPbMap.vPlanes[i].v3center[1], 2*mPbMap.vPlanes[i].v3center[2]);
          double radius = 0.1 * sqrt(mPbMap.vPlanes[i].areaVoxels);
//        cout << "radius " << radius << endl;
          sprintf (name, "sphere%u", static_cast<unsigned>(i));
          viz.addSphere (center, radius, ared[i%10], agrn[i%10], ablu[i%10], name);

          if( !mPbMap.vPlanes[i].label.empty() )
              viz.addText3D (mPbMap.vPlanes[i].label, center, 0.1, ared[i%10], agrn[i%10], ablu[i%10], mPbMap.vPlanes[i].label);
          else
          {
            sprintf (name, "P%u", static_cast<unsigned>(i));
            viz.addText3D (name, center, 0.1, ared[i%10], agrn[i%10], ablu[i%10], name);
          }

          // Draw edges
          if(!configPbMap.graph_rule) // Nearby neighbors
            for(set<unsigned>::iterator it = mPbMap.vPlanes[i].nearbyPlanes.begin(); it != mPbMap.vPlanes[i].nearbyPlanes.end(); it++)
            {
              if(*it > mPbMap.vPlanes[i].id)
                break;

              sprintf (name, "commonObs%u_%u", static_cast<unsigned>(i), static_cast<unsigned>(*it));
              pcl::PointXYZ center_it(2*mPbMap.vPlanes[*it].v3center[0], 2*mPbMap.vPlanes[*it].v3center[1], 2*mPbMap.vPlanes[*it].v3center[2]);
              viz.addLine (center, center_it, ared[i%10], agrn[i%10], ablu[i%10], name);
            }
          else
            for(map<unsigned,unsigned>::iterator it = mPbMap.vPlanes[i].neighborPlanes.begin(); it != mPbMap.vPlanes[i].neighborPlanes.end(); it++)
            {
              if(it->first > mPbMap.vPlanes[i].id)
                break;

              sprintf (name, "commonObs%u_%u", static_cast<unsigned>(i), static_cast<unsigned>(it->first));
              pcl::PointXYZ center_it(2*mPbMap.vPlanes[it->first].v3center[0], 2*mPbMap.vPlanes[it->first].v3center[1], 2*mPbMap.vPlanes[it->first].v3center[2]);
              viz.addLine (center, center_it, ared[i%10], agrn[i%10], ablu[i%10], name);

              sprintf (name, "edge%u_%u", static_cast<unsigned>(i), static_cast<unsigned>(it->first));
              char commonObs[8];
              sprintf (commonObs, "%u", it->second);
              pcl::PointXYZ half_edge( (center_it.x+center.x)/2, (center_it.y+center.y)/2, (center_it.z+center.z)/2 );
              viz.addText3D (commonObs, half_edge, 0.05, 1.0, 1.0, 1.0, name);
            }

        }
      }
      else
      { // Regular representation

        if (!viz.updatePointCloud (mPbMap.globalMapPtr, "cloud"))
          viz.addPointCloud (mPbMap.globalMapPtr, "cloud");

        if(mpPbMapLocaliser != NULL)
          if(mpPbMapLocaliser->alignedModelPtr){
            if (!viz.updatePointCloud (mpPbMapLocaliser->alignedModelPtr, "model"))
              viz.addPointCloud (mpPbMapLocaliser->alignedModelPtr, "model");}

        sprintf (name, "PointCloud size %u", static_cast<unsigned>( mPbMap.globalMapPtr->size() ) );
        viz.addText(name, 10, 20);

        for(size_t i=0; i<mPbMap.vPlanes.size(); i++)
        {
          Plane &plane_i = mPbMap.vPlanes[i];
          sprintf (name, "normal_%u", static_cast<unsigned>(i));
          pcl::PointXYZ pt1, pt2; // Begin and end points of normal's arrow for visualization
          pt1 = pcl::PointXYZ(plane_i.v3center[0], plane_i.v3center[1], plane_i.v3center[2]);
          pt2 = pcl::PointXYZ(plane_i.v3center[0] + (0.5f * plane_i.v3normal[0]),
                              plane_i.v3center[1] + (0.5f * plane_i.v3normal[1]),
                              plane_i.v3center[2] + (0.5f * plane_i.v3normal[2]));
          viz.addArrow (pt2, pt1, ared[i%10], agrn[i%10], ablu[i%10], false, name);

          // Draw Ppal diretion
//          if( plane_i.elongation > 1.3 )
//          {
//            sprintf (name, "ppalComp_%u", static_cast<unsigned>(i));
//            pcl::PointXYZ pt3 = pcl::PointXYZ ( plane_i.v3center[0] + (0.2f * plane_i.v3PpalDir[0]),
//                                                plane_i.v3center[1] + (0.2f * plane_i.v3PpalDir[1]),
//                                                plane_i.v3center[2] + (0.2f * plane_i.v3PpalDir[2]));
//            viz.addArrow (pt3, plane_i.pt1, ared[i%10], agrn[i%10], ablu[i%10], false, name);
//          }

          if( !plane_i.label.empty() )
            viz.addText3D (plane_i.label, pt2, 0.1, ared[i%10], agrn[i%10], ablu[i%10], plane_i.label);
          else
          {
            sprintf (name, "n%u", static_cast<unsigned>(i));
            viz.addText3D (name, pt2, 0.1, ared[i%10], agrn[i%10], ablu[i%10], name);
          }

//          sprintf (name, "planeRaw_%02u", static_cast<unsigned>(i));
//          viz.addPointCloud (plane_i.planeRawPointCloudPtr, name);// contourPtr, planePointCloudPtr, polygonContourPtr

          sprintf (name, "plane_%02u", static_cast<unsigned>(i));
          pcl::visualization::PointCloudColorHandlerCustom <PointT> color (plane_i.planePointCloudPtr, red[i%10], grn[i%10], blu[i%10]);
          viz.addPointCloud (plane_i.planePointCloudPtr, color, name);// contourPtr, planePointCloudPtr, polygonContourPtr
          viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);

          sprintf (name, "planeBorder_%02u", static_cast<unsigned>(i));
          pcl::visualization::PointCloudColorHandlerCustom <PointT> color2 (plane_i.contourPtr, 255, 255, 255);
          viz.addPointCloud (plane_i.contourPtr, color2, name);// contourPtr, planePointCloudPtr, polygonContourPtr

//          //Edges
//          if(mPbMap.edgeCloudPtr->size() > 0)
//          {
//            sprintf (name, "planeEdge_%02u", static_cast<unsigned>(i));
//            pcl::visualization::PointCloudColorHandlerCustom <PointT> color4 (mPbMap.edgeCloudPtr, 255, 255, 0);
//            viz.addPointCloud (mPbMap.edgeCloudPtr, color4, name);// contourPtr, planePointCloudPtr, polygonContourPtr
//            viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
//
//            sprintf (name, "edge%u", static_cast<unsigned>(i));
//            viz.addLine (mPbMap.edgeCloudPtr->points.front(), mPbMap.edgeCloudPtr->points.back(), ared[3], agrn[3], ablu[3], name);
//          }

//
//          sprintf (name, "planeEdgeOut_%02u", static_cast<unsigned>(i));
//          pcl::visualization::PointCloudColorHandlerCustom <PointT> color3 (mPbMap.outEdgeCloudPtr, 0, 255, 255);
//          viz.addPointCloud (mPbMap.outEdgeCloudPtr, color3, name);// contourPtr, planePointCloudPtr, polygonContourPtr
//          viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
//
////          // Draw plane info (descriptor)
////          sprintf (name, "B%u F%u P%u", PbMap.background, PbMap.foreground, PbMap.groundplane);
////            viz.addText3D (name, pt2, 0.05, ared[3], agrn[3], ablu[3], name);
//          //Edges Fin

//          sprintf (name, "outerPoly_%u", static_cast<unsigned>(i));
//          pcl::visualization::PointCloudColorHandlerCustom <PointT> colorOuterP (plane_i.planePointCloudPtr, red[i%10], grn[i%10], blu[i%10]);
//          viz.addPointCloud (plane_i.outerPolygonPtr, colorOuterP, name);// contourPtr, planePointCloudPtr, polygonContourPtr
//          viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);

          sprintf (name, "approx_plane_%02d", int (i));
          viz.addPolygon<PointT> (plane_i.polygonContourPtr, 0.5 * red[i%10], 0.5 * grn[i%10], 0.5 * blu[i%10], name);
        }

        // Draw recognized plane labels
        if(mpPbMapLocaliser != NULL)
          for(map<string, pcl::PointXYZ>::iterator it = mpPbMapLocaliser->foundPlaces.begin(); it != mpPbMapLocaliser->foundPlaces.end(); it++)
            viz.addText3D (it->first, it->second, 0.3, 0.9, 0.9, 0.9, it->first);

      }
    }
  }
}

//void PbMapMaker::saveInfoFiles()
//{
//  string results_file;
//  ofstream file;
//
//  results_file = "areaRestriction.txt";
//  file.open(results_file.c_str(), ios::app);
//    file << configPbMap.area_THRESHOLD << " " << rejectAreaT << " " << acceptAreaT << " " << rejectAreaF << " " << acceptAreaF << endl;
//  file.close();
//}

void PbMapMaker::run()
{
  cloudViewer.runOnVisualizationThread (boost::bind(&PbMapMaker::viz_cb, this, _1), "viz_cb");
  cloudViewer.registerKeyboardCallback ( keyboardEventOccurred );

  size_t numPrevKFs = 0;
  size_t minGrowPlanes = 10;
  while(!m_pbmaker_must_stop)  // Stop loop if PbMapMaker
  {
    if( numPrevKFs == frameQueue.size() )
    {
      mrpt::system::sleep(10);
    }
    else
    {
//    // Assign pointCloud of last KF to the global map
    detectPlanesCloud( frameQueue.back().cloudPtr, frameQueue.back().pose,
                      configPbMap.dist_threshold, configPbMap.angle_threshold, configPbMap.minInliersRate);

    if(configPbMap.makeCovisibilityClusters)
      if(mPbMap.vPlanes.size() > minGrowPlanes)
      {
        // Select current group
        unsigned current_group, current_group_votes = 0;
        map<unsigned, unsigned> observed_group;
        for(set<unsigned>::iterator it = observedPlanes.begin(); it != observedPlanes.end(); it++)
          if(observed_group.count(mPbMap.vPlanes[*it].id))
            observed_group[mPbMap.vPlanes[*it].id ]++;
          else
            observed_group[mPbMap.vPlanes[*it].id ] = 1;
        for(map<unsigned, unsigned>::iterator it = observed_group.begin(); it != observed_group.end(); it++)
          if(it->second > current_group_votes)
          {
            current_group = it->first;
            current_group_votes = it->second;
          }

        // Evaluate the partition of the current groups with minNcut
        int size_partition = clusterize->evalPartition(current_group);
      cout << "PARTITION SIZE " << size_partition << endl;
        assert(size_partition < 2);

        minGrowPlanes += 5;
      }

//  cout << "PbMapMaker iteration\n";

      ++numPrevKFs;
    }
  }
  m_pbmaker_finished = true;
}


///**
// * Set up the map serialization thread for saving/loading and the start the thread
// * @param sCommand the function that was called (eg. SaveMap)
// * @param sParams the params string, which may contain a filename and/or a map number
// */
//void PbMapMaker::PbMapSerialization(std::string sCommand, std::string sParams)
//{
//cout << "\n\nPbMapSerialization()\n";
//  if( mpPbMapSerializer->Init( sCommand, sParams) )
//    mpPbMapSerializer->start();
//
//  // Save the global map as a PCD file
//  pcl::io::savePCDFile("pointCloud_PbMap.pcd",*mPbMap.globalMapPtr);
//}

//void PbMapMaker::GUICommandCallBack(void *ptr, string sCommand, string sParams)
//{
//  if( sCommand == "SavePbMap" /*|| sCommand == "LoadPbMap" */)
//  {
//    static_cast<PbMapMaker*>(ptr)->PbMapSerialization(sCommand, "PbMap.xml");
//  }
//}

bool PbMapMaker::stop_pbMapMaker()
{
  m_pbmaker_must_stop = true;
  while(!m_pbmaker_finished)
    mrpt::system::sleep(1);
  cout << "Waiting for PbMapMaker thread to die.." << endl;

  mrpt::system::joinThread(pbmaker_hd);
	pbmaker_hd.clear();

	return true;
}

PbMapMaker::~PbMapMaker()
{
  cout << "PbMapMaker destructor called -> Save color information to file\n";

//  saveInfoFiles();

//  PbMapSerialization("SavePbMap","PbMap_die.xml");

  // Delete pointers
//  delete mpPbMapSerializer;
  delete mpPlaneInferInfo;
  delete mpPbMapLocaliser;

  stop_pbMapMaker();

  cout << " .. PbMapMaker has died." << endl;
}

#endif