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

  segmentServer = n.advertiseService("rail_segmentation/segment", &RailSegmentation::segment, this);
}

void RailSegmentation::pointCloudCallback(const sensor_msgs::PointCloud2& pointCloud)
{
  PCLPointCloud2 tempCloud;
  pcl_conversions::toPCL(pointCloud, tempCloud);
  fromPCLPointCloud2(tempCloud, *cloudPtr);
}

bool RailSegmentation::segment(rail_segmentation::Segment::Request &req, rail_segmentation::Segment::Response &res)
{
  // convert point cloud to base footprint frame
  PointCloud<PointXYZRGB>::Ptr transformedCloudPtr(new PointCloud<PointXYZRGB>);
  pcl_ros::transformPointCloud("base_footprint", *cloudPtr, *transformedCloudPtr, tfListener);

  // filter bad values;
  PointCloud<PointXYZRGB>::Ptr filteredCloudPtr(new PointCloud<PointXYZRGB>);
  vector<int> filteredIndices;
  removeNaNFromPointCloud(*transformedCloudPtr, *filteredCloudPtr, filteredIndices);
  Eigen::Vector3f(0, 0, 1);
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

  // remove all points below the plane
  PointCloud<PointXYZRGB>::Ptr heightFilteredCloudPtr(new PointCloud<PointXYZRGB>);
  float planeHeight = 0.0;
  for (unsigned int i = 0; i < planePtr->size(); i++)
  {
    planeHeight += planePtr->points[i].z;
  }
  planeHeight /= (float)(planePtr->size());
  ROS_INFO("Plane at height: %f", planeHeight);
  ConditionAnd<pcl::PointXYZRGB>::Ptr heightCondition(new ConditionAnd<pcl::PointXYZRGB>);
  heightCondition->addComparison(
      FieldComparison<PointXYZRGB>::ConstPtr(new FieldComparison<PointXYZRGB>("z", ComparisonOps::GT, planeHeight)));
  // Temporary solution for bounding search area to table area: filter anything outside of the robots reach
  heightCondition->addComparison(
      FieldComparison<PointXYZRGB>::ConstPtr(new FieldComparison<PointXYZRGB>("x", ComparisonOps::LT, 1.25)));
  heightCondition->addComparison(
      FieldComparison<PointXYZRGB>::ConstPtr(new FieldComparison<PointXYZRGB>("x", ComparisonOps::GT, 0.44)));
  // End temporary solution
  ConditionalRemoval<PointXYZRGB> heightRemoval(heightCondition);
  heightRemoval.setInputCloud(filteredCloudPtr);
  heightRemoval.filter(*heightFilteredCloudPtr);
  *filteredCloudPtr = *heightFilteredCloudPtr;
  ROS_INFO("done filtering");

  EuclideanClusterExtraction<PointXYZRGB> seg;
  vector<PointIndices> clusterIndices;
  search::KdTree<PointXYZRGB>::Ptr searchTree(new search::KdTree<PointXYZRGB>);
  searchTree->setInputCloud(filteredCloudPtr);
  seg.setSearchMethod(searchTree);
  seg.setClusterTolerance(.02);
  seg.setMinClusterSize(MIN_CLUSTER_SIZE);
  seg.setMaxClusterSize(MAX_CLUSTER_SIZE);
  seg.setSearchMethod(searchTree);
  seg.setInputCloud(filteredCloudPtr);
  seg.extract(clusterIndices);

  ROS_INFO("Found %lu clusters.", clusterIndices.size());

  if (clusterIndices.size() > 0)
  {
    rail_segmentation::SegmentedObjectList objectList;
    rail_segmentation::SegmentedObjectList objectListVis;
    objectList.header.frame_id = filteredCloudPtr->header.frame_id;
    objectList.header.stamp = ros::Time::now();
    objectList.objects.clear();
    objectListVis.header.frame_id = filteredCloudPtr->header.frame_id;
    objectListVis.header.stamp = ros::Time::now();
    objectListVis.objects.clear();
    for (unsigned int i = 0; i < clusterIndices.size(); i++)
    {
      PointCloud<PointXYZRGB>::Ptr cluster(new PointCloud<PointXYZRGB>);
      for (unsigned int j = 0; j < clusterIndices[i].indices.size(); j++)
      {
        cluster->points.push_back(filteredCloudPtr->points[clusterIndices[i].indices[j]]);
      }
      cluster->width = cluster->points.size();
      cluster->height = 1;
      cluster->is_dense = true;
      cluster->header.frame_id = filteredCloudPtr->header.frame_id;

      rail_segmentation::SegmentedObject segmentedObject;
      PCLPointCloud2::Ptr tempCloudPtr(new PCLPointCloud2());
      toPCLPointCloud2(*cluster, *tempCloudPtr);
      pcl_conversions::fromPCL(*tempCloudPtr, segmentedObject.objectCloud);
      segmentedObject.recognized = false;
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rail_segmentation");
  RailSegmentation rs;
  ros::spin();

  return EXIT_SUCCESS;
}
