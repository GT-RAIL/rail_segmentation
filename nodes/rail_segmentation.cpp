/*!
 * \file rail_segmentation.cpp
 * \brief The main segmentation node.
 *
 * The segmenter is responsible for segmenting clusters from a point cloud topic. Visualization and data latched topics
 * are published after each request. A persistent array of objects is maintained internally.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 17, 2015
 */

#include "rail_segmentation/Segmenter.h"

using namespace std;
using namespace rail::segmentation;

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
  ros::init(argc, argv, "rail_segmentation");
  Segmenter segmenter;
  // check if everything started okay
  if (segmenter.okay())
  {
    ros::spin();
    return EXIT_SUCCESS;
  } else
  {
    return EXIT_FAILURE;
  }
}
