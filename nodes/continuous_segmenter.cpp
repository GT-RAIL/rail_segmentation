/*!
 * \file rail_segmentation.cpp
 * \brief The main segmentation node.
 *
 * The segmenter is responsible for segmenting clusters from a point cloud topic. Visualization and data latched topics
 * are published after each request. A persistent array of objects is maintained internally.
 *
 * \author Russell Toris, WPI - russell.toris@gmail.com
 * \date March 17, 2015
 */

#include <ros/ros.h>
#include <std_srvs/Empty.h>

using namespace std;

/*!
 * Creates and runs the rail_segmentation node.
 *
 * \param argc argument count that is passed to ros::init.
 * \param argv arguments that are passed to ros::init.
 * \return EXIT_SUCCESS if the node runs correctly or EXIT_FAILURE if an error occurs.
 */
int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "continuous_segmenter");

  ros::NodeHandle node;
  ros::NodeHandle pnh("~");
  ros::ServiceClient segmentClient;
  segmentClient = node.serviceClient<std_srvs::Empty>("rail_segmentation/segment");
  std_srvs::Empty srv;
  double rate;
  pnh.param("segmentation_rate", rate, 0.5);

  ros::Rate loop_rate(rate);
  while (ros::ok())
  {
    segmentClient.call(srv);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}
