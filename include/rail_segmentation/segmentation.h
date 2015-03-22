#ifndef SEGMENTATION_H_
#define SEGMENTATION_H_

//ROS
#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <rail_segmentation/Recognize.h>
#include <rail_segmentation/RemoveObject.h>
#include <rail_segmentation/Segment.h>
#include <rail_segmentation/SegmentedObjectList.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/image_encodings.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>

//PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/edge_aware_plane_comparator.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
//#include <pcl/surface/mls.h>
#include <pcl/surface/bilateral_upsampling.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/features/normal_3d.h>

#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>


//Segmentation parameters
#define MIN_CLUSTER_SIZE 200
#define MAX_CLUSTER_SIZE 10000

class RailSegmentation
{
public:
  ros::NodeHandle n;

  /**
   * Constructor
   */
  RailSegmentation();
  
private:
  float cameraPitch;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> segmentedClouds;
  tf::TransformListener tfListener;

  ros::Publisher segmentedObjectsPublisher;
  ros::Publisher segmentedObjectsVisPublisher;
  ros::Subscriber pointCloudSubscriber;
  ros::Subscriber cameraPitchSubscriber;

  ros::ServiceServer autoSegmentServer;
  ros::ServiceServer segmentServer;
  ros::ServiceServer recognizeServer;
  ros::ServiceServer removeObjectServer;
  ros::ServiceServer clearObjectsServer;
  
  ros::ServiceClient recognizeClient;

  rail_segmentation::SegmentedObjectList objectList;    //segmented object list
  rail_segmentation::SegmentedObjectList objectListVis; //downsampled segmented object list for visualization

  /**
   * \brief Callback for the point cloud listener
   * @param pointCloud point cloud from the camera stream
   */
  void pointCloudCallback(const sensor_msgs::PointCloud2& pointCloud);

  /**
  * \brief Callback for the pitch of the camera from which to segment
  * @param msg camera servo joint state
  */
  void cameraPitchCallback(const sensor_msgs::JointState& msg);

  /**
  * \brief Callback for automatic segmentation service, results will be published to /rail_segmentation/segmented_objects
  * @param req service request
  * @param res service response
  * @return true on success
  */
  bool segmentAuto(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  /**
   * \brief Callback for segmentation service
   * @param req empty service request
   * @param res empty service response
   * @return true on success
   */
  bool segment(rail_segmentation::Segment::Request &req, rail_segmentation::Segment::Response &res);

  /**
  * \brief Transform input point cloud to the robot's coordinate frame and do any filtering for preprocessing
  * @param cloudInPtr Input point cloud
  * @param cloudOutPtr Resulting filtered point cloud
  */
  void preprocessPointCloud (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudInPtr,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOutPtr);

  /**
  * \brief Detect the largest non-floor horizontal surface within a point cloud
  * @param pointCloudPtr Input/output point cloud from which to detect and remove a table surface
  * @return height of the removed plane
  */
  float removeTableSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudPtr,
      pcl::PointCloud<pcl::Normal>::Ptr normalCloudPtr, pcl::PointCloud<pcl::Label>::Ptr labels,
      std::vector<bool>* excludeLabels);

  /**
  * \brief Segment objects within a bounded volume
  * @param cloudInPtr Input point cloud
  * @param cloudOutPtr Output point cloud to which the returned PointIndices are registered
  * @param boundingCondition Conditions defining the bounded volume
  * @return A list of PointIndices representing each segmented cluster within the output point cloud
  */
  std::vector<pcl::PointIndices> boundAndExtractClusters(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudInPtr,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOutPtr, pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr boundingCondition,
      pcl::PointCloud<pcl::Normal>::Ptr normalCloudPtr, pcl::PointCloud<pcl::Label>::Ptr labels,
      std::vector<bool>* excludeLabels);


  void extractOrganizedClustersImage(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudInPtr,
      const std::vector<int> &indices, sensor_msgs::Image::Ptr im);

   /**
   * \brief Callback for recognizing any unrecognized segmented objects
   * @param req empty service request
   * @param res empty service response
   * @return true on success
   */
  bool recognize(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
   
   /**
   * \brief Callback for removing an object in the object list
   * @param req service request including index of the object to be removed
   * @param res empty service response
   * @return true on success
   */
  bool removeObject(rail_segmentation::RemoveObject::Request &req, rail_segmentation::RemoveObject::Response &res);
   
   /**
   * \brief Callback for clearing segmented objects
   * @param req empty service request
   * @param res empty service response
   * @return true on success
   */
  bool clearObjectsCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  
  /**
   * \brief Clears segmented objects and publishes to the object list and visualization topics
   */
  void clearObjects();

};

#endif
