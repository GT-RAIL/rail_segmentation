#ifndef RAIL_SEGMENTATION_SEGMENTER_H_
#define RAIL_SEGMENTATION_SEGMENTER_H_

#include <pcl_ros/point_cloud.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <boost/thread/mutex.hpp>
#include <rail_segmentation/SegmentationZone.h>
#include <string>

namespace rail
{
namespace segmentation
{

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

  Segmenter();

  bool okay() const;

private:
  void pointCloudCallback(const pcl::PointCloud<pcl::PointXYZRGB> &pc);

  bool segmentCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  bool clearCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  double findSurface(const pcl::PointCloud<pcl::PointXYZRGB> &pc, const double z_min, const double z_max) const;

  void extractClusters(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pc, std::vector<pcl::PointIndices> &clusters) const;

  double averageZ(const std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> > &v) const;

  visualization_msgs::Marker createMaker(pcl::PCLPointCloud2::ConstPtr pc) const;

  const SegmentationZone &getCurrentZone() const;

  /*! The debug and okay check flags. */
  bool debug_, okay_;
  /*! Mutex for locking on the point cloud. */
  boost::mutex mutex_;
  /*! List of segmentation zones. */
  std::vector<SegmentationZone> zones_;

  /*! The global and private ROS node handles. */
  ros::NodeHandle node_, private_node_;
  /*! Services advertised by this node */
  ros::ServiceServer segment_srv_, clear_srv_;
  /*! Publishers used in the node. */
  ros::Publisher segmented_objects_pub_, markers_pub_, debug_pub_;
  /*! Subscribers used in the node. */
  ros::Subscriber point_cloud_sub_;
  /*! Main transform listener. */
  tf::TransformListener tf_;
  /*! The trasnform tree buffer for the tf2 listener. */
  tf2_ros::Buffer tf_buffer_;
  /*! The buffered trasnform client. */
  tf2_ros::TransformListener tf2_;

  /*! Latest point cloud. */
  pcl::PointCloud<pcl::PointXYZRGB> pc_;
};

}
}

#endif
