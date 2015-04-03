/*!
 * \file Segmenter.h
 * \brief The main segmentation node object.
 *
 * The segmenter is responsible for segmenting clusters from a point cloud topic. Visualization and data latched topics
 * are published after each request. A persistent array of objects is maintained internally.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \author David Kent, WPI - davidkent@wpi.edu
 * \date March 17, 2015
 */

#ifndef RAIL_SEGMENTATION_SEGMENTER_H_
#define RAIL_SEGMENTATION_SEGMENTER_H_

// RAIL Segmentation
#include "SegmentationZone.h"

// ROS
#include <pcl_ros/point_cloud.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <rail_segmentation/RemoveObject.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// PCL
#include <pcl/filters/conditional_removal.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// BOOST
#include <boost/thread/mutex.hpp>

// C++ Standard Library
#include <string>

namespace rail
{
namespace segmentation
{

/*!
 * \class Segmenter
 * \brief The main grasp collector node object.
 *
 * The grasp collector is responsible for capturing and storing grasps. An action server is started is the main entry
 * point to grasp collecting.
 */
class Segmenter
{
public:
  /*! If a topic should be created to display debug information such as point clouds. */
  static const bool DEFAULT_DEBUG = false;
  /*! The angle epsilon (delta) threshold for the plane segmenter. */
  static const double SAC_EPS_ANGLE = 0.15;
  /*! The distance threshold for the plane segmenter. */
  static const double SAC_DISTANCE_THRESHOLD = 0.01;
  /*! The maximum interations for the plane segmenter */
  static const int SAC_MAX_ITERATIONS = 100;
  /*! The padding for surface removal. */
  static const double SURFACE_REMOVAL_PADDING = 0.005;
  /*! The minimum cluster size. */
  static const int MIN_CLUSTER_SIZE = 200;
  /*! The maximum cluster size. */
  static const int MAX_CLUSTER_SIZE = 10000;
  /*! The cluster tolerance level. */
  static const double CLUSTER_TOLERANCE = 0.02;
  /*! Leaf size of the voxel grid for downsampling. */
  static const float DOWNSAMPLE_LEAF_SIZE = 0.01;
  /*! Size of the marker visualization scale factor. */
  static const double MARKER_SCALE = 0.01;

  /*!
   * \brief Create a Segmenter and associated ROS information.
   *
   * Creates a ROS node handle, subscribes to the relevant topics and servers, and creates services for requesting
   * segmenations.
   */
  Segmenter();

  /*!
   * \brief A check for a valid Segmenter.
   *
   * This function will return true if valid segmenation zones were parsed from a YAML config file.
   *
   * \return True if valid segmenation zones were parsed.
   */
  bool okay() const;

private:
  /*!
   * \brief Callback for the point cloud topic.
   *
   * Saves a copy of the latest point cloud internally.
   *
   * \param pc The current point cloud message.
   */
  void pointCloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pc);

  /*!
   * \brief Determine the current zone based on the latest state of the TF tree.
   *
   * Checks each segmenation zone criteria based on teh latest state of the TF tree and returns a reference to that
   * zone. If multiple zones are met, the first is returned. If no valid zone is found, the first zone is returned
   * and a warning is sent to ROS_WARN.
   *
   * \return The zone that matches the current state of the TF tree.
   */
  const SegmentationZone &getCurrentZone() const;

  /*!
   * \brief Callback for the remove object request.
   *
   * Remote the object from the segmented object list with a given ID. This will publish both an updated segmented
   * object list and a marker array with a delete action for the given marker index.
   *
   * \param req The request with the index to use.
   * \param res The empty response (unused).
   * \return Returns true if a valid index was provided.
   */
  bool removeObjectCallback(rail_segmentation::RemoveObject::Request &req,
      rail_segmentation::RemoveObject::Response &res);

  /*!
   * \brief Callback for the clear request.
   *
   * Clears the current segmented object list. This will publish both an empty segmented object list and a marker
   * array with delete actions from the last segmentation request.
   *
   * \param req The empty request (unused).
   * \param res The empty response (unused).
   * \return Will always return true.
   */
  bool clearCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  /*!
   * \brief Callback for the main segmentation request.
   *
   * Performs a segmenation with the latest point cloud. This will publish both a segmented object list and a marker
   * array of the resulting segmentation.
   *
   * \param req The empty request (unused).
   * \param res The empty response (unused).
   * \return Returns true if the segmentation was successful.
   */
  bool segmentCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  /*!
   * \brief Find and remove a surface from the given point cloud.
   *
   * Find a surface in the input point cloud and attempt to remove it. The surface must be within the bounds provided
   * in order to be removed. The resulting point cloud is placed in the output cloud. If no surface is found, no
   * effect is made to the output cloud and negative infinity is returned.
   *
   * \param in The input point cloud.
   * \param indices_in The indices in the point cloud to consider.
   * \param z_min The minimum height of a surface to remove.
   * \param z_max The maximum height of a surface to remove.
   * \param indices_out The set of points that are not part of the surface.
   * \return The average height of the surface that was removed or negative infinity if no valid surface was found.
   */
  double findSurface(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &in, const pcl::IndicesConstPtr &indices_in,
      const double z_min, const double z_max, const pcl::IndicesPtr &indices_out) const;

  /*!
   * \brief Find clusters in a point cloud.
   *
   * Find the clusters in the given point cloud using euclidean cluster extraction and a KD search tree.
   *
   * \param in The point cloud to search for point clouds from.
   * \param indices_in The indices in the point cloud to consider.
   * \param clusters The indices of each cluster in the point cloud.
   */
  void extractClusters(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &in, const pcl::IndicesConstPtr &indices_in,
      std::vector<pcl::PointIndices> &clusters) const;

  /*!
   * \brief Bound a point cloud based on the inverse of a set of conditions.
   *
   * Extract a new point cloud based on the inverse of a set of conditions.
   *
   * \param in The point cloud to take points from.
   * \param indices_in The indices in the point cloud to consider.
   * \param conditions The conditions specifying which points to ignore.
   * \param indices_out The set of points that pass the condition test.
   */
  void inverseBound(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &in, const pcl::IndicesConstPtr &indices_in,
      const pcl::ConditionBase<pcl::PointXYZRGB>::Ptr &conditions, const pcl::IndicesPtr &indices_out) const;

  /*!
   * \brief Extract a new point cloud based on the given indices.
   *
   * Extract a new point cloud from the given indices. The resulting point cloud will be unorganized.
   *
   * \param in The point cloud to take points from.
   * \param indices_in The indices to create a new point cloud from.
   * \param out The point cloud to fill.
   */
  void extract(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &in, const pcl::IndicesConstPtr &indices_in,
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &out) const;

  /*!
   * \brief Find the average Z value of the point vector.
   *
   * Finds the average Z value of the point vector. An empty vector will return an average of 0.
   *
   * \param v The vector of points to average.
   * \return The average Z value of the provided points.
   */
  double averageZ(const std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> > &v) const;

  /*!
   * \brief Create a Marker from the given point cloud.
   *
   * Creates a new Marker message based on the PCL point cloud. The point cloud will first be downsampled.
   *
   * \param pc The PCL point cloud to create a marker for.
   * \return The corresponding marker for the given point cloud.
   */
  visualization_msgs::Marker createMarker(const pcl::PCLPointCloud2::ConstPtr &pc) const;

  /*!
   * \brief Create a cropped image of the segmented object.
   *
   * Creates a new ROS image based on the cropped segmented object.
   *
   * \param in The original organized point cloud.
   * \param cluster The indicies of the current cluster in the point cloud.
   * \return The corresponding image for the given cluster.
   */
  sensor_msgs::Image createImage(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &in,
      const pcl::PointIndices &cluster) const;

  /*! The debug, okay check, and first point cloud flags. */
  bool debug_, okay_, first_pc_in_;
  /*! Mutex for locking on the point cloud and current messages. */
  boost::mutex pc_mutex_, msg_mutex_;
  /*! List of segmentation zones. */
  std::vector<SegmentationZone> zones_;

  /*! The global and private ROS node handles. */
  ros::NodeHandle node_, private_node_;
  /*! Services advertised by this node */
  ros::ServiceServer segment_srv_, clear_srv_, remove_object_srv_;
  /*! Publishers used in the node. */
  ros::Publisher segmented_objects_pub_, markers_pub_, debug_pc_pub_, debug_img_pub_;
  /*! Subscribers used in the node. */
  ros::Subscriber point_cloud_sub_;
  /*! Main transform listener. */
  tf::TransformListener tf_;
  /*! The trasnform tree buffer for the tf2 listener. */
  tf2_ros::Buffer tf_buffer_;
  /*! The buffered trasnform client. */
  tf2_ros::TransformListener tf2_;

  /*! Latest point cloud. */
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pc_;
  /*! Current object list. */
  rail_manipulation_msgs::SegmentedObjectList object_list_;
  /*! Current marker array. */
  visualization_msgs::MarkerArray markers_;
};

}
}

#endif
