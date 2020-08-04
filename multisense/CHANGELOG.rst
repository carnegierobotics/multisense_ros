^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package multisense
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.0.5 (2020-08-03)
------------------

4.0.4 (2020-07-01)
------------------
* update release to reference github instead of bitbucket

4.0.3 (2020-01-07)
------------------
* undo early commit
* incremented package.xml version numbers for next release
* Contributors: Quentin Torgerson <qtorgerson@carnegierobotics.com>

4.0.2 (2019-04-03)
------------------
* added C++98 compatibility
* Contributors: Quentin Torgerson <qtorgerson@carnegierobotics.com>

3.3.0 (2014-09-30)
------------------
* Updated LibMultiSense to build under C++11. Added URDF for the MultiSense S7/S7S and BCAM. Added support for 16 bit mono images. Added support for the MultiSense ST21 thermal stereo camera. Added organized pointcloud publishing. Changed laser and camera pointcloud color fields to FLOAT32 for PCL compatibility. Changed default color image encoding to BGR8. Added spindle joint publishing via the ROS joint_state_publisher. Updated multisense_cal_check to handle various serial number entires. Added the launch-file sensor parameter to load different URDF’s on startup. Published camera info topics for each image topic (for unrectified topics K, D, and R are populated). Added default laser transform publishing to keep the laser TF tree valid even when there are no subscriptions to laser topics.
* Changed license from LGPL to BSD in both the ROS Driver and LibMultiSense C++ library. Fixed bug in disparity image publishing.  Fixed bug in raw_cam_config publishing.  Fixed bug in building using rosbuild under Groovy, Hydro, Indigo, etc.  Fixed Jenkins linking issue with libpng. Fixed termination bug in process_bags.py.
* Add histogram topic (only published when subscribed to an image topic.)  Fix bitbucket issue #5 (color pointcloud published at a very slow rate.) Misc. fixes to build configuration.
* Add support for catkin and rosbuild (Builds under Fuerte, Groovy, Hydro, and Indigo). Transitioned laser calibration from KDL and joint_state_publisher to pure ROS TF messages. Add support for multiple Multisene units via namespacing and tf_prefix's. Modified default topic names to reflect the new namespacing parameters (Default base namespace is now /multisense rather than /multisense_sl). Add support for 3.1_beta sensor firmware which includes support for Multisense-S21 units. Please note that the 3.1 ROS driver release is fully backwards compatible with all 2.X firmware versions.
* Contributors: Eric Kratzer <ekratzer@carnegierobotics.com>, Matt Alvarado <malvarado@carnegierobotics.com>
