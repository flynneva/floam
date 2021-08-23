^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package floam
^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2021-08-22)
------------------
* increase mapping odom queue size
* trying to reduce copying
* improve edge and surface detection
* trying to improve edge detection
* rewrote mapping class
* minor fixes for single-scan device
* working for unorganized single scanner
* fix launch files for scanner with 1 scan line
* functional mapping node
* functional odom node
* clean up diff logic, fix edge threshold param name
* more clean up, default to exact sync in launch
* fix frame_id, use variable for nearest k
* use variable for k points to search
* try different algos for predicting odom
* clean up odom estimation classes
* clean up coordinate frames, fix some error prints, transform pointcloud to new frame
* fix odom node, launch files
* make edgeThreshold param
* scanning lidar nodelets functional
* implement scanning lidar logic too
* add ros ci actions
* rewrote mapping nodelet
* change defaults for lidar launch, add approx sync callback for odom
* rewrote odom nodelet
* implemented templates for imager and scanner
* migrate to templates
* split lidar class to scanning and imaging
* edge detection working
* switch to nodelets
* fix launch files
* fix package depends, clean up files
* fix cmake error
* add demo link
* update
* Merge pull request `#24 <https://github.com/flynneva/floam/issues/24>`_ from Chris7462/master
  Fix some minor issues
* add img
* update rosbag link
* update edge cost
* Fix frame_id names and downSizeFilterSurf and downSizeFilterEdge misplaced
* remove multiple pointcloud subscription
* change video
* add video demo
* change rosbag location
* add floam ssl description
* change frame name
* add resultion setting and add support for velodyne VLP-16
* fix function mismatch problem
* add VLP16 description
* add new points selection rules and some code optimization
* add tf
* add trajectory sever install description
* change rosbag path
* change picture location
* change picture location
* add mapping gif
* add mapping node
* add some comments to code
* change frame name
* add license
* add comments to floam
* remove opencv dependency
* add csome instruction to roslaunch
* add csome instruction to roslaunch
* add csome instruction to roslaunch
* update some comparison
* update some comparison
* update some comparison
* update some comparison
* floam
* Contributors: Evan Flynn, Wang Han 王晗, chris7462, wh200720041
