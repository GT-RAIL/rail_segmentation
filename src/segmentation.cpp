#include <rail_segmentation/segmentation.h>

using namespace pcl;
using namespace std;

RailSegmentation::RailSegmentation()
{
  PointCloud<PointXYZRGB>::Ptr tempCloudPtr(new PointCloud<PointXYZRGB>);
  cloudPtr = tempCloudPtr;

  segmentedClouds.clear();

  segmentedObjectsPublisher = n.advertise<rail_segmentation::SegmentedObjectList>("rail_segmentation/segmented_objects",
                                                                                  1);
  segmentedObjectsVisPublisher = n.advertise<rail_segmentation::SegmentedObjectList>(
      "rail_segmentation/segmented_objects_visualization", 1);
  pointCloudSubscriber = n.subscribe("/camera/depth_registered/points", 1, &RailSegmentation::pointCloudCallback, this);

  objectList.header.stamp = ros::Time::now();
  objectListVis.header.stamp = ros::Time::now();
  objectList.objects.clear();
  objectListVis.objects.clear();

  recognizeClient = n.serviceClient<rail_segmentation::Recognize>("rail_recognition/recognize");

  clearObjectsServer = n.advertiseService("rail_segmentation/clear_objects", &RailSegmentation::clearObjectsCallback, this);
  removeObjectServer = n.advertiseService("rail_segmentation/remove_object", &RailSegmentation::removeObject, this);
  segmentServer = n.advertiseService("rail_segmentation/segment", &RailSegmentation::segment, this);
  recognizeServer = n.advertiseService("rail_segmentation/recognize", &RailSegmentation::recognize, this);
}

void RailSegmentation::pointCloudCallback(const sensor_msgs::PointCloud2& pointCloud)
{
  PCLPointCloud2 tempCloud;
  pcl_conversions::toPCL(pointCloud, tempCloud);
  fromPCLPointCloud2(tempCloud, *cloudPtr);
}

bool RailSegmentation::segment(rail_segmentation::Segment::Request &req, rail_segmentation::Segment::Response &res)
{
  if (req.clear)
  {
    clearObjects();
  }

  // convert point cloud to base_footprint frame
  PointCloud<PointXYZRGB>::Ptr transformedCloudPtr(new PointCloud<PointXYZRGB>);
  pcl_ros::transformPointCloud("base_footprint", *cloudPtr, *transformedCloudPtr, tfListener);

  // filter bad values;
  PointCloud<PointXYZRGB>::Ptr filteredCloudPtr(new PointCloud<PointXYZRGB>);
  vector<int> filteredIndices;
  removeNaNFromPointCloud(*transformedCloudPtr, *filteredCloudPtr, filteredIndices);
  Eigen::Vector3f(0, 0, 1);
    
  // Determine bounding volume for segmentation
  PointCloud<PointXYZRGB>::Ptr volumeBoundedCloudPtr(new PointCloud<PointXYZRGB>);
  ConditionAnd<pcl::PointXYZRGB>::Ptr boundingCondition(new ConditionAnd<pcl::PointXYZRGB>);
  if (!req.segmentOnRobot)
  {
    // find table surface
    SACSegmentation<PointXYZRGB> planeSeg;
    PointIndices::Ptr inliers(new PointIndices);
    ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    PointCloud<PointXYZRGB>::Ptr planePtr(new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr planeRemovedPtr(new PointCloud<PointXYZRGB>);
    planeSeg.setOptimizeCoefficients(true);
    planeSeg.setModelType(SACMODEL_PERPENDICULAR_PLANE);
    planeSeg.setAxis(Eigen::Vector3f(0, 0, 1));
    planeSeg.setEpsAngle(.15);
    planeSeg.setMethodType(SAC_RANSAC);
    planeSeg.setMaxIterations(100);
    planeSeg.setDistanceThreshold(.01);
    do
    {
      planeSeg.setInputCloud(filteredCloudPtr);
      planeSeg.segment(*inliers, *coefficients);
      if (inliers->indices.size() == 0)
      {
        ROS_INFO("Could not find a table surface");
        return false;
      }
      ExtractIndices<PointXYZRGB> extract;
      extract.setInputCloud(filteredCloudPtr);
      extract.setIndices(inliers);
      extract.setNegative(false);
      extract.filter(*planePtr);
      extract.setNegative(true);
      extract.filter(*planeRemovedPtr);
      *filteredCloudPtr = *planeRemovedPtr;
      // check point height, if the plane is the floor, extract another plane
    } while (planePtr->points[0].z < .2);
    
    // bound volume above table plane
    float planeHeight = 0.0;
    for (unsigned int i = 0; i < planePtr->size(); i++)
    {
      planeHeight += planePtr->points[i].z;
    }
    planeHeight /= (float)(planePtr->size());
    ROS_INFO("Found plane at height: %f", planeHeight);

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
  
  ConditionalRemoval<PointXYZRGB> heightRemoval(boundingCondition);
  heightRemoval.setInputCloud(filteredCloudPtr);
  heightRemoval.filter(*volumeBoundedCloudPtr);
  ROS_INFO("done filtering");

  EuclideanClusterExtraction<PointXYZRGB> seg;
  vector<PointIndices> clusterIndices;
  search::KdTree<PointXYZRGB>::Ptr searchTree(new search::KdTree<PointXYZRGB>);
  searchTree->setInputCloud(volumeBoundedCloudPtr);
  seg.setSearchMethod(searchTree);
  seg.setClusterTolerance(.02);
  seg.setMinClusterSize(MIN_CLUSTER_SIZE);
  seg.setMaxClusterSize(MAX_CLUSTER_SIZE);
  seg.setSearchMethod(searchTree);
  seg.setInputCloud(volumeBoundedCloudPtr);
  seg.extract(clusterIndices);

  ROS_INFO("Found %lu clusters.", clusterIndices.size());

  res.objects.clear();
  
  if (clusterIndices.size() > 0)
  {
    objectList.header.stamp = ros::Time::now();
    objectListVis.header.stamp = ros::Time::now();
    for (unsigned int i = 0; i < clusterIndices.size(); i++)
    {
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
        pcl_ros::transformPointCloud("map", *cluster, *transformedCluster, tfListener);
        toPCLPointCloud2(*transformedCluster, *tempCloudPtr);
      }
      else
      {
        toPCLPointCloud2(*cluster, *tempCloudPtr);
      }
      pcl_conversions::fromPCL(*tempCloudPtr, segmentedObject.objectCloud);
      segmentedObject.recognized = false;
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
