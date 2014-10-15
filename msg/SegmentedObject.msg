sensor_msgs/PointCloud2 objectCloud     #segmented point cloud
bool recognized                         #true if the object is recognized
string name                             #object name if recognized
int32 model                             #model index if recognized
geometry_msgs/PoseStamped[] graspPoses  #list of grasp poses if recognized
