#include <rail_segmentation/segmentation.h>

using namespace pcl;
using namespace std;

RailSegmentation::RailSegmentation()
{
  cameraPitch = 0;

  PointCloud<PointXYZRGB>::Ptr tempCloudPtr(new PointCloud<PointXYZRGB>);
  cloudPtr = tempCloudPtr;

  segmentedClouds.clear();

  segmentedObjectsPublisher = n.advertise<rail_segmentation::SegmentedObjectList>("rail_segmentation/segmented_objects",
                                                                                  1);
  segmentedObjectsVisPublisher = n.advertise<rail_segmentation::SegmentedObjectList>(
      "rail_segmentation/segmented_objects_visualization", 1);
  pointCloudSubscriber = n.subscribe("/camera/depth_registered/points", 1, &RailSegmentation::pointCloudCallback, this);
  cameraPitchSubscriber = n.subscribe("/dynamixel_back", 1, &RailSegmentation::cameraPitchCallback, this);

  objectList.header.stamp = ros::Time::now();
  objectListVis.header.stamp = ros::Time::now();
  objectList.objects.clear();
  objectListVis.objects.clear();
  
  recognizeClient = n.serviceClient<rail_segmentation::Recognize>("rail_recognition/recognize");

  clearObjectsServer = n.advertiseService("rail_segmentation/clear_objects", &RailSegmentation::clearObjectsCallback, this);
  removeObjectServer = n.advertiseService("rail_segmentation/remove_object", &RailSegmentation::removeObject, this);
  autoSegmentServer = n.advertiseService("rail_segmentation/segment_auto", &RailSegmentation::segmentAuto, this);
  segmentServer = n.advertiseService("rail_segmentation/segment", &RailSegmentation::segment, this);
  recognizeServer = n.advertiseService("rail_segmentation/recognize", &RailSegmentation::recognize, this);
}

void RailSegmentation::pointCloudCallback(const sensor_msgs::PointCloud2& pointCloud)
{
  PCLPointCloud2 tempCloud;
  pcl_conversions::toPCL(pointCloud, tempCloud);
  fromPCLPointCloud2(tempCloud, *cloudPtr);
}

void RailSegmentation::cameraPitchCallback(const sensor_msgs::JointState& msg)
{
  cameraPitch = msg.position[0];
}

bool RailSegmentation::segmentAuto(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  clearObjects();

  //determine the segmentation area - 0 for front of robot, 1 for on robot, 2 for back of robot
  int segmentationArea;
  if (cameraPitch < .08)
    segmentationArea = 0;
  else if (cameraPitch < 1.0)
    segmentationArea = 1;
  else
    segmentationArea = 2;

  PointCloud<PointXYZRGB>::Ptr filteredCloudPtr(new PointCloud<PointXYZRGB>);
  pcl::PointCloud<pcl::Normal>::Ptr normalCloudPtr (new pcl::PointCloud<pcl::Normal>);
  preprocessPointCloud(cloudPtr, filteredCloudPtr);



  // Determine bounding volume for segmentation
  PointCloud<PointXYZRGB>::Ptr volumeBoundedCloudPtr(new PointCloud<PointXYZRGB>);
  ConditionAnd<PointXYZRGB>::Ptr boundingCondition(new ConditionAnd<PointXYZRGB>);
  pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
  std::vector<bool>* excludeLabels = (new std::vector<bool>);

  float planeHeight = removeTableSurface(filteredCloudPtr,normalCloudPtr,labels, excludeLabels);
  if (segmentationArea == 1)
  {
    // bound search area to the volume above CARL's base plate
    boundingCondition->addComparison(
        FieldComparison<PointXYZRGB>::ConstPtr(new FieldComparison<PointXYZRGB>("x", ComparisonOps::GT, -0.21)));
    boundingCondition->addComparison(
        FieldComparison<PointXYZRGB>::ConstPtr(new FieldComparison<PointXYZRGB>("x", ComparisonOps::LT, 0.22)));
    boundingCondition->addComparison(
        FieldComparison<PointXYZRGB>::ConstPtr(new FieldComparison<PointXYZRGB>("y", ComparisonOps::GT, -0.3)));
    boundingCondition->addComparison(
        FieldComparison<PointXYZRGB>::ConstPtr(new FieldComparison<PointXYZRGB>("y", ComparisonOps::LT, 0.3)));
    boundingCondition->addComparison(
        FieldComparison<PointXYZRGB>::ConstPtr(new FieldComparison<PointXYZRGB>("z", ComparisonOps::GT, 0.745)));
  }
  else
  {
    // bound search area to table area roughly within CARL's area of influence
    boundingCondition->addComparison(
        FieldComparison<PointXYZRGB>::ConstPtr(new FieldComparison<PointXYZRGB>("z", ComparisonOps::GT, planeHeight + .005)));
    if (segmentationArea == 0)
    {
      boundingCondition->addComparison(
          FieldComparison<PointXYZRGB>::ConstPtr(new FieldComparison<PointXYZRGB>("x", ComparisonOps::GT, 0.44)));
      boundingCondition->addComparison(
          FieldComparison<PointXYZRGB>::ConstPtr(new FieldComparison<PointXYZRGB>("x", ComparisonOps::LT, 1.25)));
    }
    else
    {
      boundingCondition->addComparison(
          FieldComparison<PointXYZRGB>::ConstPtr(new FieldComparison<PointXYZRGB>("x", ComparisonOps::LT, -0.36)));
      boundingCondition->addComparison(
          FieldComparison<PointXYZRGB>::ConstPtr(new FieldComparison<PointXYZRGB>("x", ComparisonOps::GT, -1.25)));
    }
  }

  vector<PointIndices> clusterIndices = boundAndExtractClusters(filteredCloudPtr, volumeBoundedCloudPtr, boundingCondition, normalCloudPtr, labels, excludeLabels);

  printf("here123\n");

  if (clusterIndices.size() > 0)
  {
    objectList.header.stamp = ros::Time::now();
    objectListVis.header.stamp = ros::Time::now();
    for (unsigned int i = 0; i < clusterIndices.size(); i++)
    {
      sensor_msgs::Image::Ptr im(new sensor_msgs::Image);
      extractOrganizedClustersImage(volumeBoundedCloudPtr, clusterIndices[i].indices, im);

      PointCloud<PointXYZRGB>::Ptr cluster(new PointCloud<PointXYZRGB>);
      for (unsigned int j = 0; j < clusterIndices[i].indices.size(); j++)
      {
        cluster->points.push_back(volumeBoundedCloudPtr->points[clusterIndices[i].indices[j]]);
      }
      cluster->width = cluster->points.size();
      cluster->height = 1;
      cluster->is_dense = true;
      cluster->header.frame_id = volumeBoundedCloudPtr->header.frame_id;

      rail_segmentation::SegmentedObject segmentedObject;
      PCLPointCloud2::Ptr tempCloudPtr(new PCLPointCloud2());
      if (segmentationArea != 1)
      {
        PointCloud<PointXYZRGB>::Ptr transformedCluster(new PointCloud<PointXYZRGB>);
        pcl_ros::transformPointCloud("map", ros::Time(0), *cluster, "base_footprint", *transformedCluster, tfListener);
        transformedCluster->header.frame_id = "map";
        toPCLPointCloud2(*transformedCluster, *tempCloudPtr);
      }
      else
      {
        toPCLPointCloud2(*cluster, *tempCloudPtr);
      }
      pcl_conversions::fromPCL(*tempCloudPtr, segmentedObject.objectCloud);
      segmentedObject.recognized = false;
      segmentedObject.objectImage = *im;
      objectList.objects.push_back(segmentedObject);

      // downsample point cloud for visualization
      rail_segmentation::SegmentedObject segmentedObjectVis;
      PCLPointCloud2::Ptr downsampledCloudPtr(new PCLPointCloud2());
      VoxelGrid<PCLPointCloud2> voxelGrid;
      voxelGrid.setInputCloud(tempCloudPtr);
      voxelGrid.setLeafSize(.01f, .01f, .01f);
      voxelGrid.filter(*downsampledCloudPtr);
      pcl_conversions::fromPCL(*downsampledCloudPtr, segmentedObjectVis.objectCloud);
      segmentedObjectVis.recognized = false;
      objectListVis.objects.push_back(segmentedObjectVis);
    }
    segmentedObjectsPublisher.publish(objectList);
    segmentedObjectsVisPublisher.publish(objectListVis);
  }
  else
  {
    ROS_INFO("Found no objects");
  }

  ROS_INFO("Segmentation complete.");
  return true;
}

bool RailSegmentation::segment(rail_segmentation::Segment::Request &req, rail_segmentation::Segment::Response &res)
{
  if (req.clear)
  {
    clearObjects();
  }

  PointCloud<PointXYZRGB>::Ptr filteredCloudPtr(new PointCloud<PointXYZRGB>);
  pcl::PointCloud<pcl::Normal>::Ptr normalCloudPtr (new pcl::PointCloud<pcl::Normal>);
  preprocessPointCloud(cloudPtr, filteredCloudPtr);

  // Determine bounding volume for segmentation
  PointCloud<PointXYZRGB>::Ptr volumeBoundedCloudPtr(new PointCloud<PointXYZRGB>);
  ConditionAnd<pcl::PointXYZRGB>::Ptr boundingCondition(new ConditionAnd<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
  std::vector<bool>* excludeLabels = (new std::vector<bool>);

  float planeHeight = removeTableSurface(filteredCloudPtr,normalCloudPtr,labels, excludeLabels);
  if (!req.segmentOnRobot)
  {
    // bound search area to table area roughly within CARL's reach
    boundingCondition->addComparison(
        FieldComparison<PointXYZRGB>::ConstPtr(new FieldComparison<PointXYZRGB>("z", ComparisonOps::GT, planeHeight + .005)));
    boundingCondition->addComparison(
        FieldComparison<PointXYZRGB>::ConstPtr(new FieldComparison<PointXYZRGB>("x", ComparisonOps::GT, 0.44)));
    boundingCondition->addComparison(
        FieldComparison<PointXYZRGB>::ConstPtr(new FieldComparison<PointXYZRGB>("x", ComparisonOps::LT, 1.25)));
  }
  else
  {
    // bound search area to the volume above CARL's base plate
    boundingCondition->addComparison(
      FieldComparison<PointXYZRGB>::ConstPtr(new FieldComparison<PointXYZRGB>("x", ComparisonOps::GT, -0.21)));
    boundingCondition->addComparison(
      FieldComparison<PointXYZRGB>::ConstPtr(new FieldComparison<PointXYZRGB>("x", ComparisonOps::LT, 0.22)));
    boundingCondition->addComparison(
      FieldComparison<PointXYZRGB>::ConstPtr(new FieldComparison<PointXYZRGB>("y", ComparisonOps::GT, -0.3)));
    boundingCondition->addComparison(
      FieldComparison<PointXYZRGB>::ConstPtr(new FieldComparison<PointXYZRGB>("y", ComparisonOps::LT, 0.3)));
    boundingCondition->addComparison(
      FieldComparison<PointXYZRGB>::ConstPtr(new FieldComparison<PointXYZRGB>("z", ComparisonOps::GT, 0.745)));
  }


  vector<PointIndices> clusterIndices = boundAndExtractClusters(filteredCloudPtr, volumeBoundedCloudPtr, boundingCondition, normalCloudPtr, labels, excludeLabels);

  res.objects.clear();
  if (clusterIndices.size() > 0)
  {
    objectList.header.stamp = ros::Time::now();
    objectListVis.header.stamp = ros::Time::now();
    for (unsigned int i = 0; i < clusterIndices.size(); i++)
    {
      sensor_msgs::Image::Ptr im(new sensor_msgs::Image);
      extractOrganizedClustersImage(volumeBoundedCloudPtr, clusterIndices[i].indices, im);

      PointCloud<PointXYZRGB>::Ptr cluster(new PointCloud<PointXYZRGB>);
      for (unsigned int j = 0; j < clusterIndices[i].indices.size(); j++)
      {
        cluster->points.push_back(volumeBoundedCloudPtr->points[clusterIndices[i].indices[j]]);
      }
      cluster->width = cluster->points.size();
      cluster->height = 1;
      cluster->is_dense = true;
      cluster->header.frame_id = volumeBoundedCloudPtr->header.frame_id;

      rail_segmentation::SegmentedObject segmentedObject;
      PCLPointCloud2::Ptr tempCloudPtr(new PCLPointCloud2());
      if (req.useMapFrame)
      {
        PointCloud<PointXYZRGB>::Ptr transformedCluster(new PointCloud<PointXYZRGB>);
        pcl_ros::transformPointCloud("map", ros::Time(0), *cluster, "base_footprint", *transformedCluster, tfListener);
        transformedCluster->header.frame_id = "map";
        toPCLPointCloud2(*transformedCluster, *tempCloudPtr);
      }
      else
      {
        toPCLPointCloud2(*cluster, *tempCloudPtr);
      }
      pcl_conversions::fromPCL(*tempCloudPtr, segmentedObject.objectCloud);
      segmentedObject.recognized = false;
      segmentedObject.objectImage = *im;
      objectList.objects.push_back(segmentedObject);
      res.objects.push_back(segmentedObject.objectCloud);

      // downsample point cloud for visualization
      rail_segmentation::SegmentedObject segmentedObjectVis;
      PCLPointCloud2::Ptr downsampledCloudPtr(new PCLPointCloud2());
      VoxelGrid<PCLPointCloud2> voxelGrid;
      voxelGrid.setInputCloud(tempCloudPtr);
      voxelGrid.setLeafSize(.01f, .01f, .01f);
      voxelGrid.filter(*downsampledCloudPtr);
      pcl_conversions::fromPCL(*downsampledCloudPtr, segmentedObjectVis.objectCloud);
      segmentedObjectVis.recognized = false;
      objectListVis.objects.push_back(segmentedObjectVis);
    }
    segmentedObjectsPublisher.publish(objectList);
    segmentedObjectsVisPublisher.publish(objectListVis);
  }
  else
  {
    ROS_INFO("Found no objects");
  }

  ROS_INFO("Segmentation complete.");
  return true;
}

void RailSegmentation::preprocessPointCloud(const pcl::PointCloud<PointXYZRGB>::Ptr cloudInPtr,
    pcl::PointCloud<PointXYZRGB>::Ptr cloudOutPtr)
{
  // convert point cloud to base_footprint frame
  pcl::PointCloud<PointXYZRGB>::Ptr transformedCloudPtr(new pcl::PointCloud<PointXYZRGB>);
  pcl_ros::transformPointCloud("base_footprint", ros::Time(0), *cloudInPtr, cloudInPtr->header.frame_id, *cloudOutPtr, tfListener);
  cloudOutPtr->header.frame_id= "base_footprint";
}

float RailSegmentation::removeTableSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudPtr,
    pcl::PointCloud<pcl::Normal>::Ptr normalCloudPtr, pcl::PointCloud<pcl::Label>::Ptr labels,
    std::vector<bool>* excludeLabels)
{
  float planeHeight = 0.0;
  size_t planeSize = 0;

  //Get surface normals from the point cloud
  pcl::IntegralImageNormalEstimation<PointXYZRGB, pcl::Normal> normalEstimator;
  normalEstimator.setNormalEstimationMethod (normalEstimator.COVARIANCE_MATRIX);
  normalEstimator.setMaxDepthChangeFactor (0.02f);
  normalEstimator.setNormalSmoothingSize (20.0f);
  normalEstimator.setInputCloud(pointCloudPtr);
  normalEstimator.compute(*normalCloudPtr);
  float* distance_map = normalEstimator.getDistanceMap();

  std::vector<pcl::PlanarRegion<PointXYZRGB>, Eigen::aligned_allocator<pcl::PlanarRegion<PointXYZRGB> > > regions;
  std::vector<pcl::ModelCoefficients> model_coefficients;
  std::vector<pcl::PointIndices> inlier_indices;
  std::vector<pcl::PointIndices> label_indices;
  std::vector<pcl::PointIndices> boundary_indices;
  pcl::OrganizedMultiPlaneSegmentation<PointXYZRGB, pcl::Normal, pcl::Label> mps;
  pcl::EdgeAwarePlaneComparator<PointXYZRGB, pcl::Normal>::Ptr edgeAwareComparator;
  edgeAwareComparator.reset (new pcl::EdgeAwarePlaneComparator<PointXYZRGB, pcl::Normal> ());
  boost::shared_ptr<EdgeAwarePlaneComparator<PointXYZRGB,Normal> > eapc = boost::dynamic_pointer_cast<EdgeAwarePlaneComparator<pcl::PointXYZRGB,Normal> >(edgeAwareComparator);
  eapc->setDistanceMap(distance_map);
  eapc->setDistanceThreshold (0.01f, false);
  mps.setMinInliers (MAX_CLUSTER_SIZE);
  mps.setAngularThreshold(pcl::deg2rad(3.0)); //3 degrees
  mps.setDistanceThreshold(0.02); //2cm
  mps.setComparator (eapc);
  mps.setInputNormals(normalCloudPtr);
  mps.setInputCloud (pointCloudPtr);
  mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);

  excludeLabels->resize (label_indices.size (), false);
  for (size_t i = 0; i < label_indices.size (); i++)
  {
    if (label_indices[i].indices.size () > MAX_CLUSTER_SIZE)
    {
      pcl::NormalEstimation<PointXYZRGB, pcl::Normal> ne;
      float nx, ny, nz, curve;
      ne.computePointNormal(*pointCloudPtr, label_indices[i].indices, nx, ny, nz, curve);

      if (nz > 0.97)
      {
        Eigen::Vector4f xyz_centroid;
        compute3DCentroid(*pointCloudPtr, label_indices[i].indices, xyz_centroid);
        if (xyz_centroid[2] > planeHeight && label_indices[i].indices.size() > planeSize)
        {
          planeHeight = xyz_centroid[2];
          planeSize = label_indices[i].indices.size();
        }
      }

      (*excludeLabels)[i] = true;
      ExtractIndices<PointXYZRGB> extract;
      boost::shared_ptr<vector<int> > indicesPtr(new vector<int> (label_indices[i].indices));
      extract.setKeepOrganized(true);
      extract.setInputCloud(pointCloudPtr);
      extract.setIndices(indicesPtr);
      extract.setNegative(true);
      extract.filterDirectly(pointCloudPtr);
    }
  }

  //assume plane height is 0.205
  ROS_INFO("Found plane at height: %f", planeHeight);
  return planeHeight;
}

std::vector<pcl::PointIndices> RailSegmentation::boundAndExtractClusters(const PointCloud<PointXYZRGB>::Ptr cloudInPtr,
    PointCloud<PointXYZRGB>::Ptr cloudOutPtr, ConditionAnd<PointXYZRGB>::Ptr boundingCondition,
    pcl::PointCloud<pcl::Normal>::Ptr normalCloudPtr, pcl::PointCloud<pcl::Label>::Ptr labels,
    std::vector<bool>* excludeLabels)
{
  ConditionalRemoval<PointXYZRGB> heightRemoval(boundingCondition);
  heightRemoval.setInputCloud(cloudInPtr);
  heightRemoval.setKeepOrganized(true);
  heightRemoval.filter(*cloudOutPtr);
  ROS_INFO("done filtering");

  //Segment Objects
  pcl::EuclideanClusterComparator<PointXYZRGB, pcl::Normal, pcl::Label>::Ptr comp (new pcl::EuclideanClusterComparator<PointXYZRGB, pcl::Normal, pcl::Label> ());
  comp->setInputCloud(cloudOutPtr);
  comp->setInputNormals(normalCloudPtr);
  comp->setLabels(labels);
  comp->setExcludeLabels(*excludeLabels);
  comp->setDistanceThreshold(0.01f, false);
  comp->setAngularThreshold(pcl::deg2rad(3.0f));

  pcl::PointCloud<pcl::Label> euclidean_labels;
  std::vector<pcl::PointIndices> euclidean_label_indices;
  pcl::OrganizedConnectedComponentSegmentation <PointXYZRGB, pcl::Label> seg(comp);
  seg.setInputCloud(cloudOutPtr);
  seg.segment(euclidean_labels, euclidean_label_indices);

  for (std::vector<pcl::PointIndices>::iterator it = euclidean_label_indices.end(); euclidean_label_indices.begin() < it; it--)
  {
    if ((*it).indices.size() < MIN_CLUSTER_SIZE || (*it).indices.size() > MAX_CLUSTER_SIZE)
      euclidean_label_indices.erase(it);
  }

  ROS_INFO("Found %lu clusters.", euclidean_label_indices.size());
  return euclidean_label_indices;
}




void RailSegmentation::extractOrganizedClustersImage(const PointCloud<PointXYZRGB>::Ptr cloudInPtr,
    const std::vector<int> &indices, sensor_msgs::Image::Ptr im)
{
  assert(cloudInPtr->isOrganized() == True);

  PointCloud<PointXYZRGB>::Ptr temPtr(new PointCloud<PointXYZRGB>);

  int inWidth = cloudInPtr->width;
  int minCol = cloudInPtr->width, maxCol = 0, minRow = cloudInPtr->height, maxRow = 0;
  for(size_t i = 0; i < indices.size(); i++)
  {
    int row = indices[i] / cloudInPtr->width;
    int col = indices[i]-(row * cloudInPtr->width);

    if(row < minRow)
      minRow = row;
    else if(row > maxRow)
      maxRow = row;

    if(col < minCol)
      minCol = col;
    else if(col > maxCol)
      maxCol = col;
  }

  //printf("size %d\n",(maxRow - minRow)*(maxCol - minCol));

  int iter = 0;
  for(size_t r = 0; r < maxRow - minRow; r++)
  {
    for(size_t c = 0; c < maxCol - minCol; c++)
    {
      temPtr->points.push_back(cloudInPtr->at(minCol+c, minRow+r));
    }
  }

  temPtr->height = maxRow - minRow;
  temPtr->width = maxCol - minCol;
  temPtr->is_dense = true;
  temPtr->header.frame_id = cloudInPtr->header.frame_id;


  cv::Mat result = cv::Mat(temPtr->height, temPtr->width, CV_8UC3);
  if (!temPtr->empty()) {
    for (int h=0; h<result.rows; h++) {
      for (int w=0; w<result.cols; w++) {
        pcl::PointXYZRGB point = temPtr->at(w, h);


        Eigen::Vector3i rgb = point.getRGBVector3i();

        result.at<cv::Vec3b>(h,w)[0] = rgb[2];
        result.at<cv::Vec3b>(h,w)[1] = rgb[1];
        result.at<cv::Vec3b>(h,w)[2] = rgb[0];
      }
    }

    cv_bridge::CvImage cvi;
    cvi.header.frame_id = "base_footprint";
    cvi.encoding = "bgr8";
    cvi.image = result;
    cvi.toImageMsg(*im);
  }
}


bool RailSegmentation::recognize(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  for (unsigned int i = 0; i < objectList.objects.size(); i ++)
  {
    if (!objectList.objects[i].recognized)
    {
      rail_segmentation::Recognize::Request recReq;
      rail_segmentation::Recognize::Response recRes;
      recReq.objectCloud = objectList.objects[i].objectCloud;
      if (!recognizeClient.call(recReq, recRes))
      {
        ROS_INFO("Failed to call object recognition client.");
        return false;
      }
      if (recRes.success)
      {
        objectList.objects[i].recognized = true;
        objectList.objects[i].name = recRes.name;
        objectList.objects[i].model = recRes.model;
        objectList.objects[i].graspPoses = recRes.graspPoses;
        objectListVis.objects[i].recognized = true;
        objectListVis.objects[i].name = recRes.name;
        objectListVis.objects[i].model = recRes.model;
        objectListVis.objects[i].graspPoses = recRes.graspPoses;
      }
    }
  }
  segmentedObjectsPublisher.publish(objectList);
  segmentedObjectsVisPublisher.publish(objectListVis);
  
  return true;
}

bool RailSegmentation::removeObject(rail_segmentation::RemoveObject::Request &req, rail_segmentation::RemoveObject::Response &res)
{
  if (req.index > objectList.objects.size())
  {
    ROS_INFO("Index for object to be removed is outside of objectList bounds.");
    return true;
  }
  objectList.objects.erase(objectList.objects.begin() + req.index);
  objectListVis.objects.erase(objectListVis.objects.begin() + req.index);
  
  segmentedObjectsPublisher.publish(objectList);
  segmentedObjectsVisPublisher.publish(objectListVis);
  
  return true;
}

bool RailSegmentation::clearObjectsCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  clearObjects();
  return true;
}

void RailSegmentation::clearObjects()
{
  objectList.objects.clear();
  objectListVis.objects.clear();
  segmentedObjectsPublisher.publish(objectList);
  segmentedObjectsVisPublisher.publish(objectListVis);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rail_segmentation");
  RailSegmentation rs;
  ros::spin();

  return EXIT_SUCCESS;
}
