^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rail_segmentation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
