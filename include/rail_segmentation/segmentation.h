#ifndef SEGMENTATION_H_
#define SEGMENTATION_H_

//ROS
#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <rail_segmentation/Segment.h>
#include <rail_segmentation/SegmentedObjectList.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>

//PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

//Segmentation parameters
#define MIN_CLUSTER_SIZE 200
#define MAX_CLUSTER_SIZE 10000

class RailSegmentation
{
public:
  ros::NodeHandle n;

  RailSegmentation();

private:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> segmentedClouds;
  tf::TransformListener tfListener;

  ros::Publisher segmentedObjectsPublisher;
  ros::Publisher segmentedObjectsVisPublisher;
  ros::Subscriber pointCloudSubscriber;

  ros::ServiceServer segmentServer;

  void pointCloudCallback(const sensor_msgs::PointCloud2& pointCloud);

  bool segment(rail_segmentation::Segment::Request &req, rail_segmentation::Segment::Response &res);
};

#endif
