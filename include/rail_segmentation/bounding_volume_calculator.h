#ifndef RAIL_SEGMENTATION_BOUNDING_VOLUME_CALCULATOR_H
#define RAIL_SEGMENTATION_BOUNDING_VOLUME_CALCULATOR_H

// ROS
#include <rail_manipulation_msgs/BoundingVolume.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

// PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * @brief Static functions to compute point cloud bounding boxes for pick-and-place.
 */
class BoundingVolumeCalculator
{

public:

  /**
   * @brief Fit a z-axis-aligned bounding box to x-y plane principal direction of point cloud.
   * @param cloud point cloud to bound
   * @return computed bounding box
   */
  static rail_manipulation_msgs::BoundingVolume computeBoundingVolume(sensor_msgs::PointCloud2 cloud);

  /**
   * @brief Fit a z-axis-aligned bounding box to x-y plane principal direction of point cloud.
   * @param cloud point cloud to bound
   * @return computed bounding box
   */
  static rail_manipulation_msgs::BoundingVolume computeBoundingVolume(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

  /**
   * @brief Fit a z-axis-aligned bounding box to x-y plane principal direction of point cloud.
   * @param cloud point cloud to bound
   * @return computed bounding box
   */
  static rail_manipulation_msgs::BoundingVolume computeBoundingVolume(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
};

#endif // RAIL_SEGMENTATION_BOUNDING_VOLUME_CALCULATOR_H