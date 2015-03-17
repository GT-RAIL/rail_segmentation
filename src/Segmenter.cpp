/*!
 * \file Segmenter.cpp
 * \brief The main segmentation node object.
 *
 * The segmenter is responsible for segmenting clusters from a point cloud topic. Visualization and data latched topics
 * are published after each request. A persistent array of objects is maintained internally.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \author David Kent, WPI - davidkent@wpi.edu
 * \date March 17, 2015
 */

#include <rail_segmentation/Segmenter.h>

#include <pcl_ros/transforms.h>
#include <ros/package.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <yaml-cpp/yaml.h>

using namespace std;
using namespace rail::segmentation;

Segmenter::Segmenter() : private_node_("~"), tf2_(tf_buffer_)
{
  // set defaults
  debug_ = DEFAULT_DEBUG;
  string point_cloud_topic("/camera/depth_registered/points");
  string zones_file(ros::package::getPath("rail_segmentation") + "/config/zones.yaml");

  // grab any parameters we need
  private_node_.getParam("debug", debug_);
  private_node_.getParam("point_cloud_topic", point_cloud_topic);
  private_node_.getParam("mapping_config", zones_file);

  // setup publishers/subscribers we need
  segment_srv_ = private_node_.advertiseService("segment", &Segmenter::segmentCallback, this);
  clear_srv_ = private_node_.advertiseService("clear", &Segmenter::clearCallback, this);
  remove_object_srv_ = private_node_.advertiseService("remove_object", &Segmenter::removeObjectCallback, this);
  segmented_objects_pub_ = private_node_.advertise<rail_manipulation_msgs::SegmentedObjectList>(
      "segmented_objects", 1, true
  );
  markers_pub_ = private_node_.advertise<visualization_msgs::MarkerArray>("markers", 1, true);
  point_cloud_sub_ = node_.subscribe(point_cloud_topic, 1, &Segmenter::pointCloudCallback, this);
  // setup a debug publisher if we need it
  if (debug_)
  {
    debug_pub_ = private_node_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("debug", 1);
  }

  // check the YAML version
#ifdef YAMLCPP_GT_0_5_0
  // parse the segmentation zones
  YAML::Node zones_config = YAML::LoadFile(zones_file);
  for (size_t i = 0; i < zones_config.size(); i++)
  {
    YAML::Node cur = zones_config[i];
    // create a zone with the frame ID information
    SegmentationZone zone(cur["name"].as<string>(), cur["parent_frame_id"].as<string>(),
        cur["child_frame_id"].as<string>(), cur["bounding_frame_id"].as<string>(),
        cur["segmentation_frame_id"].as<string>());

    // check for the remove surface flag
    if (cur["remove_surface"].IsDefined())
    {
      zone.setRemoveSurface(cur["remove_surface"].as<bool>());
    }

    // check for any set limits
    if (cur["roll_min"].IsDefined())
    {
      zone.setRollMin(cur["roll_min"].as<double>());
    }
    if (cur["roll_max"].IsDefined())
    {
      zone.setRollMax(cur["roll_max"].as<double>());
    }
    if (cur["pitch_min"].IsDefined())
    {
      zone.setPitchMin(cur["pitch_min"].as<double>());
    }
    if (cur["pitch_max"].IsDefined())
    {
      zone.setPitchMax(cur["pitch_max"].as<double>());
    }
    if (cur["yaw_min"].IsDefined())
    {
      zone.setYawMin(cur["yaw_min"].as<double>());
    }
    if (cur["yaw_max"].IsDefined())
    {
      zone.setYawMax(cur["yaw_max"].as<double>());
    }
    if (cur["x_min"].IsDefined())
    {
      zone.setXMin(cur["x_min"].as<double>());
    }
    if (cur["x_max"].IsDefined())
    {
      zone.setXMax(cur["x_max"].as<double>());
    }
    if (cur["y_min"].IsDefined())
    {
      zone.setYMin(cur["y_min"].as<double>());
    }
    if (cur["y_max"].IsDefined())
    {
      zone.setYMax(cur["y_max"].as<double>());
    }
    if (cur["z_min"].IsDefined())
    {
      zone.setZMin(cur["z_min"].as<double>());
    }
    if (cur["z_max"].IsDefined())
    {
      zone.setZMax(cur["z_max"].as<double>());
    }

    zones_.push_back(zone);
  }
  // check how many zones we have
  if (zones_.size() > 0)
  {
    ROS_INFO("%d segmenation zone(s) parsed.", (int) zones_.size());
    ROS_INFO("Segmenter Successfully Initialized");
    okay_ = true;
  } else
  {
    ROS_ERROR("No valid segmenation zones defined. Check %s.", zones_file.c_str());
    okay_ = false;
  }
#else
  ROS_ERROR("Unsupported version of YAML. Config files could not be parsed.");
  okay_ = false;
#endif
}

bool Segmenter::okay() const
{
  return okay_;
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
  // lock for the messages
  boost::mutex::scoped_lock lock(msg_mutex_);
  // empty the list
  object_list_.objects.clear();
  // set header information
  object_list_.header.seq++;
  object_list_.header.stamp = ros::Time::now();
  // republish
  segmented_objects_pub_.publish(object_list_);
  // delete markers
  for (size_t i = 0; i< markers_.markers.size(); i++) {
    markers_.markers[i].action = visualization_msgs::Marker::DELETE;
  }
  markers_pub_.publish(markers_);
  markers_.markers.clear();
  return true;
}

bool Segmenter::removeObjectCallback(rail_segmentation::RemoveObject::Request &req,
    rail_segmentation::RemoveObject::Response &res)
{
  // lock for the messages
  boost::mutex::scoped_lock lock(msg_mutex_);
  // check the index
  if (req.index < object_list_.objects.size() && req.index < markers_.markers.size())
  {
    // remove
    object_list_.objects.erase(object_list_.objects.begin() + req.index);
    // set header information
    object_list_.header.seq++;
    object_list_.header.stamp = ros::Time::now();
    // republish
    segmented_objects_pub_.publish(object_list_);
    // delete marker
    markers_.markers[req.index].action = visualization_msgs::Marker::DELETE;
    markers_pub_.publish(markers_);
    markers_.markers.erase(markers_.markers.begin() + req.index);
    return true;
  } else
  {
    ROS_ERROR("Attempted to remove index %d from list of size %d.", req.index, (int) object_list_.objects.size());
    return false;
  }
}

bool Segmenter::segmentCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  // clear the objects first
  this->clearCallback(req, res);

  // determine the correct segmentation zone
  const SegmentationZone &zone = this->getCurrentZone();
  ROS_INFO("Segmenting in zone '%s'.", zone.getName().c_str());

  // transform the point cloud to the fixed frame
  pcl::PointCloud<pcl::PointXYZRGB> transformed_pc;
  // lock on the point cloud
  {
    boost::mutex::scoped_lock lock(pc_mutex_);
    // perform the copy/transform using TF
    pcl_ros::transformPointCloud(zone.getBoundingFrameID(), ros::Time(0), pc_, pc_.header.frame_id,
        transformed_pc, tf_);
    transformed_pc.header.frame_id = zone.getBoundingFrameID();
    transformed_pc.header.seq = pc_.header.seq;
    transformed_pc.header.stamp = pc_.header.stamp;
  }

  // remove invalid (NaN) values from the cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
  vector<int> indices;
  pcl::removeNaNFromPointCloud(transformed_pc, *filtered_pc, indices);

  // check if we need to remove a surface
  double z_min = zone.getZMin();
  if (zone.getRemoveSurface())
  {
    double z_surface = this->findSurface(*filtered_pc, *filtered_pc, zone.getZMin(), zone.getZMax());
    // check the new bound for Z
    z_min = max(zone.getZMin(), z_surface + SURFACE_REMOVAL_PADDING);
  }

  // check bounding areas
  pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr bounds(new pcl::ConditionAnd<pcl::PointXYZRGB>);
  if (z_min > -numeric_limits<double>::infinity())
  {
    bounds->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::GT, z_min))
    );
  }
  if (zone.getZMax() < numeric_limits<double>::infinity())
  {
    bounds->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::LT, zone.getZMax()))
    );
  }
  if (zone.getYMin() > -numeric_limits<double>::infinity())
  {
    bounds->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZRGB>("y", pcl::ComparisonOps::GT, zone.getYMin()))
    );
  }
  if (zone.getYMax() < numeric_limits<double>::infinity())
  {
    bounds->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZRGB>("y", pcl::ComparisonOps::LT, zone.getYMax()))
    );
  }
  if (zone.getXMin() > -numeric_limits<double>::infinity())
  {
    bounds->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::GT, zone.getXMin()))
    );
  }
  if (zone.getXMax() < numeric_limits<double>::infinity())
  {
    bounds->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::LT, zone.getXMax()))
    );
  }

  // remove past the given bounds
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr bounded_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ConditionalRemoval<pcl::PointXYZRGB> bounds_removal(bounds);
  bounds_removal.setInputCloud(filtered_pc);
  bounds_removal.filter(*bounded_pc);

  // publish the filtered and bounded PC pre-segmentation
  if (debug_)
  {
    debug_pub_.publish(bounded_pc);
  }

  // extract clusters
  vector<pcl::PointIndices> clusters;
  this->extractClusters(bounded_pc, clusters);


  if (clusters.size() > 0)
  {
    // lock for the messages
    boost::mutex::scoped_lock lock(msg_mutex_);

    // check each cluster
    for (size_t i = 0; i < clusters.size(); i++)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
      for (size_t j = 0; j < clusters[i].indices.size(); j++)
      {
        cluster->points.push_back(bounded_pc->points[clusters[i].indices[j]]);
      }
      cluster->width = cluster->points.size();
      cluster->height = 1;
      cluster->is_dense = true;
      cluster->header.frame_id = bounded_pc->header.frame_id;

      // check if we need to transform to a different frame
      pcl::PCLPointCloud2::Ptr converted(new pcl::PCLPointCloud2);
      if (zone.getBoundingFrameID() != zone.getSegmentationFrameID())
      {
        // perform the copy/transform using TF
        pcl::PointCloud<pcl::PointXYZRGB> tmp;
        pcl_ros::transformPointCloud(zone.getSegmentationFrameID(), ros::Time(0), *cluster, cluster->header.frame_id,
            tmp, tf_);
        tmp.header.frame_id = zone.getSegmentationFrameID();
        tmp.header.seq = cluster->header.seq;
        tmp.header.stamp = cluster->header.stamp;
        pcl::toPCLPointCloud2(tmp, *converted);
      } else
      {
        pcl::toPCLPointCloud2(*cluster, *converted);
      }

      // convert to a SegmentedObject message
      rail_manipulation_msgs::SegmentedObject segmented_object;
      segmented_object.recognized = false;

      // set the point cloud
      pcl_conversions::fromPCL(*converted, segmented_object.point_cloud);
      segmented_object.point_cloud.header.stamp = ros::Time::now();
      // create a marker and set the extra fields
      segmented_object.marker = this->createMaker(converted);
      segmented_object.marker.id = i;

      // set the centroid
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*cluster, centroid);
      segmented_object.centroid.x = centroid[0];
      segmented_object.centroid.y = centroid[1];
      segmented_object.centroid.z = centroid[2];

      // add to the final list
      object_list_.objects.push_back(segmented_object);
      // add to the markers
      markers_.markers.push_back(segmented_object.marker);
    }

    // publish the new list
    object_list_.header.seq++;
    object_list_.header.stamp = ros::Time::now();
    object_list_.header.frame_id = zone.getSegmentationFrameID();
    segmented_objects_pub_.publish(object_list_);

    // publish the new marker array
    markers_pub_.publish(markers_);
  } else
  {
    ROS_WARN("No segmented objects found.");
  }

  return true;
}

double Segmenter::findSurface(const pcl::PointCloud<pcl::PointXYZRGB> &in, pcl::PointCloud<pcl::PointXYZRGB> &out,
    const double z_min, const double z_max) const
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
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_copy(new pcl::PointCloud<pcl::PointXYZRGB>(in));
  plane_seg.setInputCloud(pc_copy);

  // Check point height -- if the plane is too low or high, extract another
  while (true)
  {
    // points included in the plane (surface)
    pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices);

    // segment the the current cloud
    pcl::ModelCoefficients coefficients;
    plane_seg.segment(*inliers_ptr, coefficients);

    // check if we found a surface
    if (inliers_ptr->indices.size() == 0)
    {
      ROS_WARN("Could not find a surface above %fm and below %fm.", z_min, z_max);
      return -numeric_limits<double>::infinity();
    }

    // remove the plane
    pcl::PointCloud<pcl::PointXYZRGB> plane;
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(pc_copy);
    extract.setIndices(inliers_ptr);
    extract.setNegative(false);
    extract.filter(plane);
    // extract everything else and try again
    extract.setNegative(true);
    extract.filter(*pc_copy);

    // check the height
    double height = this->averageZ(plane.points);
    if (height >= z_min && height <= z_max)
    {
      ROS_INFO("Surface found at %fm.", height);
      out = *pc_copy;
      return height;
    }
  }
}

void Segmenter::extractClusters(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pc,
    vector<pcl::PointIndices> &clusters) const
{
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> seg;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  kd_tree->setInputCloud(pc);
  seg.setClusterTolerance(CLUSTER_TOLERANCE);
  seg.setMinClusterSize(MIN_CLUSTER_SIZE);
  seg.setMaxClusterSize(MAX_CLUSTER_SIZE);
  seg.setSearchMethod(kd_tree);
  seg.setInputCloud(pc);
  seg.extract(clusters);
}

double Segmenter::averageZ(const vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> > &v) const
{
  double avg = 0.0;
  for (size_t i = 0; i < v.size(); i++)
  {
    avg += v[i].z;
  }
  return (avg / (double) v.size());
}

const SegmentationZone &Segmenter::getCurrentZone() const
{
  // check each zone
  for (size_t i = 0; i < zones_.size(); i++)
  {
    // get the current TF information
    geometry_msgs::TransformStamped tf = tf_buffer_.lookupTransform(zones_[i].getParentFrameID(),
        zones_[i].getChildFrameID(), ros::Time(0));

    // convert to a Matrix3x3 to get RPY
    tf2::Matrix3x3 mat(tf2::Quaternion(tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z,
        tf.transform.rotation.w));
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);

    // check if all the bounds meet
    if (roll >= zones_[i].getRollMin() && pitch >= zones_[i].getPitchMin() && yaw >= zones_[i].getYawMin() &&
        roll <= zones_[i].getRollMax() && pitch <= zones_[i].getPitchMax() && yaw <= zones_[i].getYawMax())
    {
      return zones_[i];
    }
  }

  ROS_WARN("Current state not in a valid segmentation zone. Defaulting to first zone.");
  return zones_[0];
}

visualization_msgs::Marker Segmenter::createMaker(pcl::PCLPointCloud2::ConstPtr pc) const
{
  visualization_msgs::Marker marker;
  // set header field
  marker.header.frame_id = pc->header.frame_id;

  // default position
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // default scale
  marker.scale.x = MARKER_SCALE;
  marker.scale.y = MARKER_SCALE;
  marker.scale.z = MARKER_SCALE;

  // set the type of marker and our color of choice
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  // TODO maybe use average RGB value of cluster?
  marker.color.r = ((float) (rand()) / (float) (RAND_MAX)) / 3.0 + 0.66;
  marker.color.g = ((float) (rand()) / (float) (RAND_MAX)) / 4.0;
  marker.color.b = ((float) (rand()) / (float) (RAND_MAX)) / 5.0;
  marker.color.a = 1.0;

  // downsample point cloud for visualization
  pcl::PCLPointCloud2 downsampled;
  pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_grid;
  voxel_grid.setInputCloud(pc);
  voxel_grid.setLeafSize(DOWNSAMPLE_LEAF_SIZE, DOWNSAMPLE_LEAF_SIZE, DOWNSAMPLE_LEAF_SIZE);
  voxel_grid.filter(downsampled);

  // convert to an easy to use point cloud message
  sensor_msgs::PointCloud2 pc2_msg;
  pcl_conversions::fromPCL(downsampled, pc2_msg);
  sensor_msgs::PointCloud pc_msg;
  sensor_msgs::convertPointCloud2ToPointCloud(pc2_msg, pc_msg);

  // place in the marker message
  marker.points.resize(pc_msg.points.size());
  for (size_t j = 0; j < pc_msg.points.size(); j++)
  {
    marker.points[j].x = pc_msg.points[j].x;
    marker.points[j].y = pc_msg.points[j].y;
    marker.points[j].z = pc_msg.points[j].z;
  }

  return marker;
}
