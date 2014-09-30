#!/usr/bin/env python

'''
Simple Script to Processes Bag files from RawSnapshot in Release 2.0
and writes the data to file for calibration

Please direct any question to multisense@carnegierobotics.com or
    http://support.carnegierobotics.com
'''

import sys
import csv
import time
import os
import numpy as np
import string
try:
    import rospy
    import rosbag
except:
    raise Exception("Error importing ROS. Source the ROS environment" \
                   +" in the workspace where the multisense stack is located")

class _BagProcessor():

    def __init__(self, bag_file):
        self.bag_file = bag_file

    #Wrapper method to processes the bag file
    #Returns bag file name
    def process(self, directory='.', rename_file = True,
                namespace = 'multisense'):
        self.process_laser(directory, namespace)
        self.process_image(directory, namespace)
        self.process_camera_yaml(directory, namespace)
        self.process_laser_yaml(directory, namespace)

        return_value = self.bag_file
        if rename_file:
            return_value = self.rename_bag(directory)
        # end if
        return return_value

    #Method to extract laser data from bag file and save into .csv
    def process_laser(self, directory='.', namespace='multisense'):
        bag = rosbag.Bag(self.bag_file)
        with open(directory + '/' + 'lidarData.csv', 'wb') as laser_file:
            topic_name = '/%s/calibration/raw_lidar_data' % namespace
            laser_writer = csv.writer(laser_file, delimiter=',')
            for topic, msg, t in bag.read_messages(topics=[topic_name]):
                #Unbundle message
                scan_count = msg.scan_count
                time_start = float(msg.time_start.secs +
                                        msg.time_start.nsecs * 1e-9)
                time_end = float(msg.time_end.secs + msg.time_end.nsecs * 1e-9)
                angle_start = msg.angle_start
                angle_end = msg.angle_end
                dist = msg.distance
                reflect = msg.intensity

                #Expected Format: scan_count, time_start, time_end, angle_start,
                #angle_end, , range, , intensity, ,len(range)
                row = [scan_count, time_start, time_end,
                       angle_start, angle_end, " "] + list(dist) + [" "] \
                       + list(reflect) + [" "] + [len(list(dist))]

                laser_writer.writerow(row)

        laser_file.close()


    #Method to extract first instance of disparity and recitified images from
    #bag
    def process_image(self, directory='.', namespace='multisense'):
        topic_name = '/%s/calibration/raw_cam_data' % namespace
        bag = rosbag.Bag(self.bag_file)

        #Attempt to find both disparity and rectified images before quitting
        found_rectified_image = False
        found_disparity_image = False

        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            width = msg.width
            height = msg.height

            #Write to .pgm file stereo_left_0000.pgm
            if not found_rectified_image and len(list(msg.gray_scale_image)) != 0:
                self.write_pgm(np.array(list(msg.gray_scale_image)),
                                directory + '/' + "stereo_left_0000.pgm",
                                width, height, 8)
                found_rectified_image = True

            if not found_disparity_image and len(list(msg.disparity_image)) != 0:
                self.write_pgm(np.array(list(msg.disparity_image),
                                dtype=np.uint16),
                                directory + '/' + "disparity_0000.pgm",
                                width, height, 16)
                found_disparity_image = True

            #Quit once disparity and rectified images have been found
            if found_disparity_image and found_rectified_image:
                break

    #Method to write an image to a .pgm file
    def write_pgm(self, data, name, width, height, bits):
        image = open(name, 'wb')

        #Create .pgm file header
        pgm_header = 'P5' + '\n' + str(width) + ' ' \
                     + str(height) + '\n' + str(2**bits - 1) + '\n'

        #Data needs to be big endian not little endian for 16bit images
        if bits == 16:
            data = data.byteswap()

        image.write(pgm_header)
        data.tofile(image)
        image.close()

    #Extract image intrinsics from RawCamConfig.msg
    def process_camera_yaml(self, directory='.', namespace='multisense'):
        topic_name = '/%s/calibration/raw_cam_config' % namespace
        bag = rosbag.Bag(self.bag_file)
        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            fx = msg.fx
            fy = msg.fy
            cx = msg.cx
            cy = msg.cy
            tx = msg.tx

            #Follow expected format of YAML file
            p1 =  "[ %.17e, 0., %d, 0., 0., \n" % (fx, cx) \
                 + "       %.17e, %d, 0., 0., 0., 1., 0.]" % (fy, cy) \

            p2 =  "[ %.17e, 0., %d, %.17e, 0., \n" % (fx, cx, tx*fx) \
                 + "       %.17e, %d, 0., 0., 0., 1., 0.]" % (fy, cy) \

            self.write_camera_yaml(directory + "/extrinsics_0p5mp.yml", p1, p2)


    #Writes P1 and P2 in openCV format to be used internally.
    #P1 and P2 represent the camera intrinsics
    def write_camera_yaml(self, name, p1, p2):
        yaml = open(name, 'wb')

        yaml_header = "%YAML:1.0\n"
        yaml.write(yaml_header)

        p1_header = "P1: !!opencv-matrix\n" \
                     + "   rows: 3\n" \
                     + "   cols: 4\n" \
                     + "   dt: d\n"


        p2_header = "P2: !!opencv-matrix\n" \
                     + "   rows: 3\n" \
                     + "   cols: 4\n" \
                     + "   dt: d\n"

        yaml.write(p1_header)
        yaml.write("   data: " + p1 + '\n\n')
        yaml.write(p2_header)
        yaml.write("   data: " + p2 + '\n')
        yaml.close()

    #Extract Laser To Spindle and Camera To Spindle Extrinsics from
    #RawLidarCal.msg
    def process_laser_yaml(self, directory=".", namespace='multisense'):
        topic_name = '/%s/calibration/raw_lidar_cal' % namespace
        bag = rosbag.Bag(self.bag_file)
        i = 0
        for topic, msg, t in bag.read_messages(topics=[topic_name]):

            #Only use one message
            if i > 0:
                break
            #Laser to spindle
            lts = list(msg.laserToSpindle)
            #Camera to spindle
            cts = list(msg.cameraToSpindleFixed)

            #Mimics existing OpenCV format
            laser_t_spind = "[ %.17e, %.17e,\n" % (lts[0], lts[1]) \
                            +"      %.17e, %.17e,\n" % (lts[2], lts[3]) \
                            +"      %.17e, %.17e,\n" % (lts[4], lts[5]) \
                            +"      %.17e, %.17e,\n" % (lts[6], lts[7]) \
                            +"      %.17e, %.17e,\n" % (lts[8], lts[9]) \
                            +"      %.17e, 0., 0., 0., 0., 1. ]" % (lts[10])


            camera_t_spind = "[ %.17e, %.17e,\n" % (cts[0], cts[1]) \
                             +"      %.17e, %.17e,\n" % (cts[2], cts[3]) \
                             +"      %.17e, %.17e,\n" % (cts[4], cts[5]) \
                             +"      %.17e, %.17e,\n" % (cts[6], cts[7]) \
                             +"      %.17e, %.17e,\n" % (cts[8], cts[9]) \
                             +"      %.17e, %.17e, 0., 0., 0., 1. ]" % (cts[10], cts[11])

            self.write_laser_yaml(directory + "/laser_cal.yml",
                                  laser_t_spind, camera_t_spind)
            i = i+1

    #Write laser calibration in expected OpenCV format
    def write_laser_yaml(self, name, laser_t_spind, cam_t_spind):
        yaml = open(name, 'wb')
        yaml_header = "%YAML:1.0\n"
        yaml.write(yaml_header)

        laser_t_spind_h = "laser_T_spindle: !!opencv-matrix\n" \
                          +"   rows: 4\n" \
                          +"   cols: 4\n" \
                          +"   dt: d\n"

        cam_t_spind_h = "camera_T_spindle_fixed: !!opencv-matrix\n" \
                        +"   rows: 4\n" \
                        +"   cols: 4\n" \
                        +"   dt: d\n"

        yaml.write(laser_t_spind_h)
        yaml.write("   data: " + laser_t_spind + '\n')
        yaml.write(cam_t_spind_h)
        yaml.write("   data: " + cam_t_spind + '\n')
        yaml.close()

    #Writes out sensor information appends SN to bagfile name
    #Returns new bag file name
    def rename_bag(self, directory=".", namespace='multisense'):
        topic_name = '/%s/calibration/device_info' % namespace
        bag = rosbag.Bag(self.bag_file)
        for topic, msg, t in bag.read_messages(topics=[topic_name]):

            #Extract all the digits in the serial number. The format of the
            #serial number varies from unit to unit (SNXXX, SL XXXX, SN SLXXXX)
            sn = msg.serialNumber.strip()
            exclude_characters = string.letters + string.punctuation + " "
            sn = sn.strip(exclude_characters)

            #If for whatever reason there are characters still in our serial
            #number (XXXvX) just append SN to the front
            try:
                sn = "SN%04d" % int(sn)
            except:
                sn = "SN" + sn

            info = open(os.path.join(directory, "dev_info.txt"), 'wb')

            info.write(str(msg))
            info.close


            bag = os.path.basename(self.bag_file)
            path = os.path.dirname(self.bag_file)

            fname = os.path.join(path,  os.path.splitext(bag)[0]\
                               + "_SL_%s_calCheck.bag" % sn)


            os.rename(self.bag_file, fname)
            self.bag_file = fname
            return fname


def usage(argv):
    print "\nUsage: %s bagFile <outputDirectory> <namespace>" % argv[0]
    print ""
    print "If no value is specified for outputDirectory, then a "
    print "directory will be created using the current time as "
    print "part of its name."
    print ""
    print "If a value is provided for namespace, it will be used "
    print "in extracting data from the bag file.  The default namespace "
    print "is multisense.  You might want to specify multisense_sl or "
    print "laser instead, if you are processing an older bag file."
# end def

if __name__ == '__main__':
    # Set up default arguments
    bag_file_name = 'dummy.bag'
    output_directory_name = time.strftime('%Y-%m-%d_%H-%M-%S_process_bags')
    namespace = 'multisense'
    rename_file = True

    # Process command line.
    if (len(sys.argv) < 2) | (len(sys.argv) > 4):
        usage(sys.argv)
        sys.exit(65)
    # end if
    if len(sys.argv) >= 2:
        bag_file_name = sys.argv[1]
    # end if
    if len(sys.argv) >= 3:
        output_directory_name = sys.argv[2]
    # end if
    if len(sys.argv) >= 4:
        namespace = sys.argv[3]
    # end if

    # Prepare to run _BagProcessor.
    if not os.path.exists(output_directory_name):
        os.makedirs(output_directory_name)
    # end if
    if not os.path.isdir(output_directory_name):
        raise IOError('%s is not a directory' % output_directory_name)
    # end if

    # Do the processsing.
    bagProcessor = _BagProcessor(bag_file_name)
    bagProcessor.process(output_directory_name, rename_file, namespace)
    sys.exit(0)
# end if
