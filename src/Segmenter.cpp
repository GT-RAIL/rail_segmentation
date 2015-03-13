#include <rail_segmentation/Segmenter.h>
#include <pcl_ros/transforms.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

using namespace std;
using namespace rail::segmentation;

Segmenter::Segmenter() : private_node_("~"), fixed_frame_id_("base_footprint")
{
  // set defaults
  string point_cloud_topic("/camera/depth_registered/points");

  // grab any parameters we need
  private_node_.getParam("point_cloud_topic", point_cloud_topic);
  private_node_.getParam("fixed_frame_id", fixed_frame_id_);

  // setup publishers/subscribers we need
  segment_srv_ = private_node_.advertiseService("segment", &Segmenter::segmentCallback, this);
  clear_srv_ = private_node_.advertiseService("clear", &Segmenter::clearCallback, this);
  segmented_objects_pub_ = private_node_.advertise<rail_manipulation_msgs::SegmentedObjectList>(
      "segmented_objects", 1, true
  );
  point_cloud_sub_ = node_.subscribe(point_cloud_topic, 1, &Segmenter::pointCloudCallback, this);

  // set the fixed frame for our list
  object_list_.header.frame_id = fixed_frame_id_;

  ROS_INFO("Segmenter Successfully Initialized");
}

void Segmenter::pointCloudCallback(const pcl::PointCloud<pcl::PointXYZRGB> &pc)
{
  // lock for the point cloud
  boost::mutex::scoped_lock lock(pc_mutex_);
  // simply store the latest point cloud
  pc_ = pc;
}

bool Segmenter::clearCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  // lock for the list
  boost::mutex::scoped_lock lock(object_list_mutex_);
  // clear the list
  object_list_.objects.clear();
  object_list_.header.seq++;
  object_list_.header.stamp = ros::Time::now();
  // republish
  segmented_objects_pub_.publish(object_list_);
  return true;
}

bool Segmenter::segmentCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  // clear the objects first
  this->clearCallback(req, res);

  // transform the point cloud to the fixed frame
  pcl::PointCloud<pcl::PointXYZRGB> transformed_pc_;
  // lock on the point cloud
  {
    boost::mutex::scoped_lock lock(pc_mutex_);
    // perform the copy/transform
    pcl_ros::transformPointCloud(fixed_frame_id_, ros::Time(0), pc_, pc_.header.frame_id, transformed_pc_, tf_);
    transformed_pc_.header.frame_id = fixed_frame_id_;
    transformed_pc_.header.seq = pc_.header.seq;
    transformed_pc_.header.stamp = pc_.header.stamp;
  }

  // remove invalid (NaN) values from the cloud
  pcl::PointCloud<pcl::PointXYZRGB> filtered_pc_;
  vector<int> indices;
  pcl::removeNaNFromPointCloud(transformed_pc_, filtered_pc_, indices);

  // TODO bounding areas

  cout << filtered_pc_.size() << endl;
  cout << this->removeSurface(filtered_pc_, 0) << endl;
  cout << filtered_pc_.size() << endl;

  return true;
}

float Segmenter::removeSurface(pcl::PointCloud<pcl::PointXYZRGB> &pc, double threshold)
{
  // use a plane (SAC) segmenter
  pcl::SACSegmentation<pcl::PointXYZRGB> plane_seg;
  // set the segmenation parameters
  plane_seg.setOptimizeCoefficients(true);
  plane_seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  plane_seg.setAxis(Eigen::Vector3f(0, 0, 1));
  plane_seg.setEpsAngle(SAC_EPS_ANGLE);
  plane_seg.setMethodType(pcl::SAC_RANSAC);
  plane_seg.setMaxIterations(SAC_MAX_ITERATIONS);
  plane_seg.setDistanceThreshold(SAC_DISTANCE_THRESHOLD);

  // create a copy to work with
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_ptr(new pcl::PointCloud<pcl::PointXYZRGB>(pc));
  plane_seg.setInputCloud(pc_ptr);

  // Check point height -- if the plane is too low, extract another
  float height = -1;
  while (height < threshold)
  {
    // points included in the plane (surface)
    pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices);

    // segment the the current cloud
    pcl::ModelCoefficients coefficients;
    plane_seg.segment(*inliers_ptr, coefficients);

    // check if we found a surface
    if (inliers_ptr->indices.size() == 0)
    {
      ROS_WARN("Could not find a surface above %fm.", threshold);
      return -1;
    }

    // remove the plane
    pcl::PointCloud<pcl::PointXYZRGB> plane;
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(pc_ptr);
    extract.setIndices(inliers_ptr);
    extract.setNegative(false);
    extract.filter(plane);
    // extract everything else
    extract.setNegative(true);
    extract.filter(*pc_ptr);

    height = this->averageZ(plane.points);
  }

  // set the values
  pc = *pc_ptr;
  return height;
}

float Segmenter::averageZ(const vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> > &v) const
{
  float avg = 0.0;
  for (size_t i = 0; i < v.size(); i++)
  {
    avg += v[i].z;
  }
  return (avg / (float) v.size());
}
