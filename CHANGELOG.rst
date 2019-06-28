^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rail_segmentation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.15 (2019-06-28)
-------------------
* Minor update to fix an incorrect declaration of a static const variable under certain compilers
* Contributors: David Kent

0.1.14 (2019-06-27)
-------------------
* Added new service which takes in a point cloud to segment
* Contributors: David Kent, Weiyu Liu

0.1.13 (2019-06-07)
-------------------
* Safer wait for point cloud that checks timestamps
* Switched continuous point cloud polling for waitForMessage
* Added an option to publish marker labels for each cluster for debugging
* Added a flag to the segmentation zone config to require a table; if this is set to true, segmentation will only be performed if a surface was successfully detected in the current segmentation zone.
* New service for re-calculating (or filling in uncalculated) features of segmented objects, assuming that at minimum the point cloud field is set
* Exposed cluster tolerance as a parameter
* Optional parameter for cropping the workspace before table detection (defaults to false so it won't change behavior for anything currently using rail_segmentation), which potentially speeds up segmentation but may cause table detection to fail more often when segmenting in small, cluttered segmentation zones
* Contributors: David Kent

0.1.12 (2018-10-12)
-------------------
* Replacing deprecated pcl call to upgrade ROS versions
* Contributors: David Kent

0.1.11 (2018-09-21)
-------------------
* Segmenter.h: Fix compilation error with -std=c++11.
  This commit fixes compilation errors due to compilation with std=c++11,
  such as:
  include/rail_segmentation/Segmenter.h:82:23: error: ‘constexpr’ needed for
  in-class initialization of static data member
  ‘const double rail::segmentation::Segmenter::SAC_EPS_ANGLE’ of
  non-integral type [-fpermissive]
  static const double SAC_EPS_ANGLE = 0.15;
  ^~~~~~~~~~~~~
  Signed-off-by: Elvis Dowson <elvis.dowson@gmail.com>
* Fixing a small bug so that the segment_objects service matches the topic data
* Better bounding box calculation, added average rgb and cielab color to segmented object messages as they are calculated anyway, and added an alternative service api, segment_objects, that returns the segmented object list in the service response (while still broadcasting the segmented object list on the topic)
* Constant definition fix for functions with reference parameters
* Added option for euclidean + RGB clustering instead of solely euclidean distance
* Contributors: David Kent, Elvis Dowson, Levon Avagyan, Russell Toris, Siddhartha Banerjee

0.1.10 (2016-09-17)
-------------------
* Merge pull request `#4 <https://github.com/GT-RAIL/rail_segmentation/issues/4>`_ from velveteenrobot/publish-table
  Now publishes table as SegmentedObject and table marker as Marker
* Now publishes table as SegmentedObject and table marker as Marker
* New travis for indigo and jade
* email update
* Contributors: Russell Toris, Sarah Elliott

0.1.9 (2016-02-23)
------------------
* Update .travis.yml
* Update README.md
* Update package.xml
* Added a node that continuously calls the segmentation service
* Contributors: David Kent

0.1.8 (2015-05-14)
------------------
* allows for params of min/max cluster size
* Contributors: Russell Toris

0.1.7 (2015-05-07)
------------------
* removed hard coded constants
* angle fix
* Approximated segmented object orientation with PCA
* Contributors: David Kent, Russell Toris

0.1.6 (2015-04-22)
------------------
* cleared flag added
* Contributors: Russell Toris

0.1.5 (2015-04-15)
------------------
* Added center point calculation for segmented objects
* Contributors: David Kent

0.1.4 (2015-04-14)
------------------
* quick travis fix
* old parser format
* Update .travis.yml
* Contributors: Russell Toris

0.1.3 (2015-04-10)
------------------
* bounding box info added
* Contributors: Russell Toris

0.1.2 (2015-04-03)
------------------
* cmake cleanup
* header cleanup
* header cleanup
* header cleanup
* checks for incoming point cloud first
* new lines added
* new lines added
* more const ptrs
* moved to ptr based storage
* const ptrs
* Contributors: Russell Toris

0.1.1 (2015-03-31)
------------------
* segmentation debug is now latched
* Merge branch 'develop' of github.com:WPI-RAIL/rail_segmentation into develop
* redid zones for default
* Fixed centroid calculation when the segmentation frame doesn't match the bounding box frame
* Contributors: David Kent, Russell Toris

0.1.0 (2015-03-24)
------------------
* added RGB image to message
* average RGB on marker
* uses indices instead of new PCs
* Merge pull request #1 from WPI-RAIL/refactor
  Refactor
* merge conflicts
* Revert "plane detection refactored"
  This reverts commit 7160b0b12e55755451ec5c8a9318e05552924cc6.
* doc added
* cleanup of old files
* first pass of new segmentation node
* plane detection refactored
* Added a recognize all action which gives feedback throughout the recognition process; the recognize all server remains for compatibility, but it's recommended to use the action server instead.
* Edited .travis.yml
* Merge branch 'develop' of github.com:WPI-RAIL/rail_segmentation into develop
* Updated to reflect moving some messages from rail_segmentation to rail_manipulation_messages
* Contributors: David Kent, Russell Toris

0.0.5 (2015-02-17)
------------------
* Fixed a possible exception thrown due to transforming a point cloud at an invalid time
* Merge branch 'develop' of github.com:WPI-RAIL/rail_segmentation into develop
* Added an automatic segmentation service which will determine how best to segment based on camera angle
* Contributors: David Kent

0.0.4 (2015-02-06)
------------------
* Update .travis.yml
* visualized object list initialization
* Contributors: David Kent, Russell Toris

0.0.3 (2014-10-22)
------------------
* Incorporated calls to object recognition
* Contributors: David Kent

0.0.2 (2014-10-03)
------------------
* added object clearing service and clearing on segmentation of zero objects
* Updated segmentation with an option for on-robot segmentation, added documentation
* Updated segmentation service to allow segmentation in either the map frame or the robot frame, also added optional object clearing on segmentation call
* merge
* updates for pick and place
* Contributors: dekent

0.0.1 (2014-09-22)
------------------
* bad source file fixed
* pcl_ros build
* pcl_ros build
* travis tests
* travis now runs updates
* indigo ros_pcl added
* cleanup for release
* segmentation tuning and updates
* stopped segmentation from identifying non-horizontal planes
* initial commit
* Contributors: Russell Toris, dekent
