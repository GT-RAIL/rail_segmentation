#ifndef RAIL_SEGMENTATION_SEGMENTER_H_
#define RAIL_SEGMENTATION_SEGMENTER_H_

#include <pcl_ros/point_cloud.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <boost/thread/mutex.hpp>
#include <string>

namespace rail
{
namespace segmentation
{

class Segmenter
{
public:
  /*! The angle epsilon (delta) threshold for the plane segmenter. */
  static const double SAC_EPS_ANGLE = 0.15;
  /*! The distance threshold for the plane segmenter. */
  static const double SAC_DISTANCE_THRESHOLD = 0.01;
  /*! The maximum interations for the plane segmenter */
  static const int SAC_MAX_ITERATIONS = 100;

  Segmenter();

  bool okay() const;

private:
  void pointCloudCallback(const pcl::PointCloud<pcl::PointXYZRGB> &pc);

  bool segmentCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  bool clearCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  float removeSurface(pcl::PointCloud<pcl::PointXYZRGB> &pc, double threshold);

  float averageZ(const std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> > &v) const;

  /*! Mutex for locking on the point cloud and object list. */
  boost::mutex pc_mutex_, object_list_mutex_;
  /*! Fixed frame to work in. */
  std::string fixed_frame_id_;

  /*! The global and private ROS node handles. */
  ros::NodeHandle node_, private_node_;
  /*! Services advertised by this node */
  ros::ServiceServer segment_srv_, clear_srv_;
  /*! Publishers used in the node. */
  ros::Publisher segmented_objects_pub_;
  /*! Subscribers used in the node. */
  ros::Subscriber point_cloud_sub_;
  /*! Main transform listener. */
  tf::TransformListener tf_;

  /*! Latest point cloud. */
  pcl::PointCloud<pcl::PointXYZRGB> pc_;
  /*! Current list of segmented objects. */
  rail_manipulation_msgs::SegmentedObjectList object_list_;
};

//struct null_deleter
//{
//  void operator()(void const *) const
//  {
//  }
//};

}
}

#endif
