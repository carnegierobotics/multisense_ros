# multisense_ros

Wrappers, drivers, tools and additional API's for using MultiSense S27, S30, KS21, SL, S7, S7S, S21, M, ST21, BCAM  with ROS.

### Installation, Documentation and Tutorials

See the [documentation](https://docs.carnegierobotics.com/software/ros1.html) on on installing and using multisense_ros

Please see the following link for detailed documentation on the [LibMultiSense API](https://docs.carnegierobotics.com/libmultisense/index.html) wrapped by the ROS driver.

### Driver Layout
The MultiSense ROS driver contains five ROS packages. The responsibility of each package is outlined below

##### multisense_bringup
This is the set of launch files and configuration files used to start the ROS driver, as well as configuration scripts and other configuration files.

#### multisense_cal_check
This package provides software for evaluating the quality of the laser calibration stored in the MultiSense-SL non-volatile memory.
Note this package only supports MultiSense SL units.

#### multisense_description
This package contains the http://ros.org/wiki/urdf robot description XML file and associated meshes that represent the sensor head, sensor placement and kinematic structure of a MultiSense S21 sensor.

#### multisense_lib
This is the library that implements the wire protocol for communication with the MultiSense S21 sensor.

#### multisense_ros
This package contains the actual ROS drivers for the MultiSense S21. Individual drivers are included for the Camera and IMU subsystems.

### Develop and Contribute

See [Contribute](https://github.com/carnegierobotics/multisense_ros/blob/master/CONTRIBUTING.md) page.

### Support

To report an issue with this library or request a new feature,
please use the [GitHub issues system](https://github.com/carnegierobotics/multisense_ros/issues)

For product support, please see the [support section of our website](https://carnegierobotics.com/support)
Individual support requests can be created in our [support portal](https://support.carnegierobotics.com/hc/en-us)
