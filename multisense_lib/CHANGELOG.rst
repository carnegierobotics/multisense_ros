^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package multisense_lib
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.0.5 (2020-08-03)
------------------

4.0.4 (2020-07-01)
------------------
* update release to reference github instead of bitbucket

4.0.3 (2020-01-07)
------------------
* undo early commit
* incremented package.xml version numbers for next release
* libturbo in multisense_ros
* updated libpng; added libturbojpeg to package.xml
* Contributors: Kevin Nickels <knickels@trinity.edu>, Quentin Torgerson <qtorgerson@carnegierobotics.com>

4.0.2 (2019-04-03)
------------------
* added C++98 compatibility
* Contributors: Quentin Torgerson <qtorgerson@carnegierobotics.com>

3.3.0 (2014-09-30)
------------------
* Updated LibMultiSense to build under C++11. Added URDF for the MultiSense S7/S7S and BCAM. Added support for 16 bit mono images. Added support for the MultiSense ST21 thermal stereo camera. Added organized pointcloud publishing. Changed laser and camera pointcloud color fields to FLOAT32 for PCL compatibility. Changed default color image encoding to BGR8. Added spindle joint publishing via the ROS joint_state_publisher. Updated multisense_cal_check to handle various serial number entires. Added the launch-file sensor parameter to load different URDF’s on startup. Published camera info topics for each image topic (for unrectified topics K, D, and R are populated). Added default laser transform publishing to keep the laser TF tree valid even when there are no subscriptions to laser topics.
* Changed license from LGPL to BSD in both the ROS Driver and LibMultiSense C++ library. Fixed bug in disparity image publishing.  Fixed bug in raw_cam_config publishing.  Fixed bug in building using rosbuild under Groovy, Hydro, Indigo, etc.  Fixed Jenkins linking issue with libpng. Fixed termination bug in process_bags.py.
* Remove '-march=native' compiler options as it is causing assembler errors on some platforms (specifically: Haswell/12.04_amd64/ROS Hydro)
* Add wire-protocol support for DirectedStreams on CRL's Monocular IP Camera. Misc. other bugfixes.
* Adding support files for MultiSense-S21 and Mono IP Camera products
* Add initial support for CRL's Mono IP Camera. Numerous fixes in catkin build infrastructure.
* Add support for catkin and rosbuild (Builds under Fuerte, Groovy, Hydro, and Indigo). Transitioned laser calibration from KDL and joint_state_publisher to pure ROS TF messages. Add support for multiple Multisene units via namespacing and tf_prefix's. Modified default topic names to reflect the new namespacing parameters (Default base namespace is now /multisense rather than /multisense_sl). Add support for 3.1_beta sensor firmware which includes support for Multisense-S21 units. Please note that the 3.1 ROS driver release is fully backwards compatible with all 2.X firmware versions.
* Release_3.0_beta: Add support for 3.0_beta sensor firmware (SGM hardware stereo core: disparity at all resolutions, 2:1 rectangular pixel modes, 64/128/256 disparity modes, hardware bi-lateral post-stereo disparity filter support with tuning), add colorized points2 topic, add pointcloud egde and range filtering, add raw left/right disparitiy image topics, add stereo-cost image topic, misc other feature enhancements and bugfixes.  Please note that the 3.0_beta release is fully backwards compatiblie with all 2.X firmware versions.
* Release_2.3: Add support for 2.3 sensor firmware (IMU / CMV4000 support), add 'MultiSenseUpdater' firmware upgrade tool, add smart dynamic_reconfigure presentation, remove multisense_diagnostics/multisense_dashboard, wire protocol to version 3.0 (w/ support for forthcoming SGM core), misc. other bugfixes and feature enhancements.
* Low-level c++ API to version 2.2
* -Add PPS topic: /multisense_sl/pps (std_msgs/Time)
  -Corrected step size of color images, which now display correctly using image_view
  -Add 'network_time_sync' option to dynamic reconfigure
* Imported Release 2.0 of MultiSense-SL ROS driver.
* Contributors: David LaRose <dlr@carnegierobotics.com>, Eric Kratzer <ekratzer@carnegierobotics.com>, Matt Alvarado <malvarado@carnegierobotics.com>
