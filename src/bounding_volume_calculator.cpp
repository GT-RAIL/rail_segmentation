#include <rail_segmentation/bounding_volume_calculator.h>

rail_manipulation_msgs::BoundingVolume BoundingVolumeCalculator::computeBoundingVolume(sensor_msgs::PointCloud2 cloud)
{
  //convert point cloud to pcl point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 converter;
  pcl_conversions::toPCL(cloud, converter);
  pcl::fromPCLPointCloud2(converter, *object_cloud);

  return BoundingVolumeCalculator::computeBoundingVolume(object_cloud);
}

rail_manipulation_msgs::BoundingVolume
BoundingVolumeCalculator::computeBoundingVolume(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*cloud, *xyz_cloud);

  return BoundingVolumeCalculator::computeBoundingVolume(xyz_cloud);
}

rail_manipulation_msgs::BoundingVolume
BoundingVolumeCalculator::computeBoundingVolume(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  //calculate original point cloud bounds
  pcl::PointXYZ min_original, max_original;
  pcl::getMinMax3D(*cloud, min_original, max_original);

  // project point cloud to x-y plane (this will create a bounding box that aligns with gravity; not always accurate,
  // but a useful assumption for tabletop, shelf, and floor pick-and-place applications)
  pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
  coefficients->values.resize(4);
  coefficients->values[0] = coefficients->values[1] = 0;
  coefficients->values[2] = 1;
  coefficients->values[3] = 0;
  pcl::ProjectInliers<pcl::PointXYZ> projector;
  projector.setModelType(pcl::SACMODEL_PLANE);
  projector.setInputCloud(cloud);
  projector.setModelCoefficients(coefficients);
  projector.filter(*projected_cloud);

  // compute principal direction
  Eigen::Matrix3f covariance;
  Eigen::Vector4f centroid;
  //use center instead of centroid so as not to strongly weight parts of the point cloud closer to the sensor
  centroid[0] = static_cast<float>((min_original.x + max_original.x)/2.0);
  centroid[1] = static_cast<float>((min_original.y + max_original.y)/2.0);
  centroid[2] = 0;  //because point cloud is projected to x-y plane
  pcl::computeCovarianceMatrixNormalized(*projected_cloud, centroid, covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
  Eigen::Matrix3f eig_dx = eigen_solver.eigenvectors();
  eig_dx.col(2) = eig_dx.col(0).cross(eig_dx.col(1));

  //move the points to that reference frame
  Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
  p2w.block(0, 0, 3, 3) = eig_dx.transpose();
  p2w.block(0, 3, 3, 1) = -1.f * (p2w.block(0, 0, 3, 3) * centroid.head(3));
  pcl::PointCloud<pcl::PointXYZ> c_points;
  pcl::transformPointCloud(*projected_cloud, c_points, p2w);

  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(c_points, min_pt, max_pt);
  const Eigen::Vector3f mean_diag = 0.5f * (max_pt.getVector3fMap() + min_pt.getVector3fMap());

  //final transform
  const Eigen::Quaternionf qfinal(eig_dx);
  const Eigen::Vector3f tfinal = eig_dx * mean_diag + centroid.head(3);

  //set object shape
  rail_manipulation_msgs::BoundingVolume bounding_box;
  bounding_box.dimensions.x = max_original.z - min_original.z;
  bounding_box.dimensions.y = max_pt.y - min_pt.y;
  bounding_box.dimensions.z = max_pt.z - min_pt.z;
  bounding_box.pose.header.frame_id = cloud->header.frame_id;
  bounding_box.pose.pose.position.x = tfinal[0];
  bounding_box.pose.pose.position.y = tfinal[1];
  bounding_box.pose.pose.position.z = (min_original.z + max_original.z)/2.0;
  bounding_box.pose.pose.orientation.w = qfinal.w();
  bounding_box.pose.pose.orientation.x = qfinal.x();
  bounding_box.pose.pose.orientation.y = qfinal.y();
  bounding_box.pose.pose.orientation.z = qfinal.z();

  return bounding_box;
}