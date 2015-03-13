#include <rail_segmentation/Segmenter.h>

using namespace std;
using namespace rail::segmentation;

/*!
 * Creates and runs the rail_segmentation node.
 *
 * \param argc argument count that is passed to ros::init.
 * \param argv arguments that are passed to ros::init.
 * \return EXIT_SUCCESS if the node runs correctly.
 */
int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "rail_segmentation");
  Segmenter segmenter;
  ros::spin();
  return EXIT_SUCCESS;
}
