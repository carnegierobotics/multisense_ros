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
import getopt
try:
    import rospy
    import rosbag
except:
    raise Exception("Error importing ROS. Source the ROS environment" \
                   +" in the workspace where the multisense stack is located")

class _BagProcessor():

    def __init__(self, bag_file):
        self.bag_file = bag_file

        # Transforms to move points in the laser frame to the left
        # camera optical frame
        self.laserToSpindle = None
        self.spindleToCamera = None

        # The matrix used to reproject disparity point into 3D. We will use this
        # to perform the opposite operation and transform laser points into
        # the camera frame
        self.qMatrix = None
        self.qMatrixInverse = None

        # The disparity image read from the bag file
        self.disparityImage = None
        self.disparityHeight = 0
        self.disparityWidth = 0

        # This is the error in terms of pixels between the disparity value
        # reported by the camera and the theoretical disparity value
        # that the camera would need to have to perfectly match the laser data
        self.errorImage = None

        # A debug image which is the laser data as a disparity image
        self.laserDisparity = None
    #end def

    #Wrapper method to processes the bag file
    #Returns bag file name
    def process(self, directory='.', rename_file = True,
                namespace = 'multisense'):

        # Save out the stereo and camera images for debugging. Also store
        # the disparity image to be used when computing the laser error
        # metric
        if not self.process_image(directory, namespace):
            print "Could not find valid images in the bag file"
            return None
        #end if

        # Get the calibration for our camera
        self.process_camera_yaml(directory, namespace)

        # Get the transforms to convert the laser data into the left
        # camera optical frame, the same coordinate frame which the disparity image
        # in in
        self.process_laser_yaml(directory, namespace)

        # Process our laser data and convert each scan into disparity space
        self.process_laser(directory, namespace)

        # Compute a error metric based on our processed stereo and laser
        # data
        average_error = self.compute_average_error()

        print "Average Pixel Error:", average_error, "pixels"

        if average_error < 2.:
            print "Calibration is good"
        else:
            print "Calibration appears to be poor. Place the unit 1 meter from"\
                  " a texture-rich corner an rerun the check."
        #end if


        # Save a image which displays where the largest error are in our
        # disparity image
        self.compute_error_image(average_error, directory)

        return_value = self.bag_file
        if rename_file:
            return_value = self.rename_bag(directory)
        # end if

        return return_value
    #end def

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

                # Transform our laser data into the camera frame
                self.compute_laser_error(msg)

            #end for

        laser_file.close()

        #end with
    #end def


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
            #end if

            if not found_disparity_image and len(list(msg.disparity_image)) != 0:
                self.disparityImage = np.array(list(msg.disparity_image), dtype=np.uint16)
                self.errorImage = np.zeros(len(list(msg.disparity_image)), dtype=np.float64) - 1
                self.laserDisparity = np.zeros(len(list(msg.disparity_image)), dtype=np.uint16)
                self.disparityHeight = height
                self.disparityWidth = width
                self.write_pgm(self.disparityImage,
                                directory + '/' + "disparity_0000.pgm",
                                width, height, 16)
                found_disparity_image = True
            #end if


            #Quit once disparity and rectified images have been found
            if found_disparity_image and found_rectified_image:
                return True
            #end if
        #end for

        return False
    #end def

    #Method to write an image to a .pgm file
    def write_pgm(self, data, name, width, height, bits):
        image = open(name, 'wb')

        #Create .pgm file header
        pgm_header = 'P5' + '\n' + str(width) + ' ' \
                     + str(height) + '\n' + str(2**bits - 1) + '\n'

        #Data needs to be big endian not little endian for 16bit images
        if bits == 16:
            data = data.byteswap()
        #end if

        image.write(pgm_header)
        data.tofile(image)
        image.close()
    #end def

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



            self.qMatrix = np.matrix([[fy * tx, 0 , 0, -fy * cx * tx],
                                      [0, fx * tx, 0, -fx * cy * tx],
                                      [0, 0, 0, fx * fy * tx],
                                      [0, 0, -fy, 0]])

            self.qMatrixInverse = np.linalg.inv(self.qMatrix)

            break
        #end for
    #end def


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
        for topic, msg, t in bag.read_messages(topics=[topic_name]):

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

            self.laserToSpindle = np.matrix([[lts[0], lts[1], lts[2], lts[3]],
                                             [lts[4], lts[5], lts[6], lts[7]],
                                             [lts[8], lts[9], lts[10], lts[11]],
                                             [lts[12], lts[13], lts[14], lts[15]]])

            self.spindleToCamera = np.matrix([[cts[0], cts[1], cts[2], cts[3]],
                                             [cts[4], cts[5], cts[6], cts[7]],
                                             [cts[8], cts[9], cts[10], cts[11]],
                                             [cts[12], cts[13], cts[14], cts[15]]])


            break
        #end for
    #end def

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
    #end def

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
        #end for
    #end def

    # Compute the average of all our errors
    def compute_average_error(self):
        average = 0.
        valid_elements = 0

        for item in self.errorImage:
            # A value of -1 means the cell has not been initialized
            if item != -1:
                average += item
                valid_elements += 1
            #end if
        #end for

        average /= valid_elements

        return average

    #end def

    # Convert our errors into a image to see where the calibration is poor
    def compute_error_image(self, average_error, directory='.'):
        min_error = 0
        max_error = 3 * average_error

        eight_bit_error = np.zeros(len(self.errorImage), dtype=np.uint8)

        # Iterate through our error image and convert it to a 8 bit image
        # using the average_error error as our max error
        for i, item in enumerate(self.errorImage):
            # A value of -1 means the cell has not been initialized
            if item != -1:
                eight_bit_error[i] = min((item - min_error) / (max_error - min_error) * 255, 255)
            #end if
        #end for

        self.write_pgm(eight_bit_error, os.path.join(directory,"errorImage.pgm"),
                       self.disparityWidth, self.disparityHeight, 8)

        if self.laserDisparity != None:
            self.write_pgm(self.laserDisparity, os.path.join(directory,"laserDisparity.pgm"),
                           self.disparityWidth, self.disparityHeight, 16)
        #end if

    #end def


    # Compute the error or an individual scan message
    def compute_laser_error(self, laser_msg):
        # Transform a laser scan so it is in the camera frame
        laser_scan = self.transform_laser_scan(laser_msg)

        # Make sure our laser_scan point are not None. I.e. we got valid
        # laser data from the bag file
        if laser_scan == None:
            return
        #end if

        # Convert our laser scan into the disparity space of the camera
        disparity_scan = self.qMatrixInverse * laser_scan

        for i in range(disparity_scan.shape[1]):


            # Scale our projected vector
            u = round(disparity_scan[0,i]/disparity_scan[3,i])
            v = round(disparity_scan[1,i]/disparity_scan[3,i])
            d = disparity_scan[2,i]/disparity_scan[3,i]

            # Make sure the point projects into our image
            if (u < 0 or u >= self.disparityWidth or
                v < 0 or v >= self.disparityHeight):
                continue
            #end if

            index = u + self.disparityWidth * v

            # Save our computed disparity value in a image. We will store
            # the value in 1/6 of a pixel
            self.laserDisparity[index] = d * 16.


            # Get the actual disparity. This is a value which is in 1/16 of a pixel.
            # Here we will convert it to pixels
            actual_disparity = self.disparityImage[index]/16.

            # If our actual disparity is 0 it means our value is not valid. Here
            # we will just skip the point
            if actual_disparity == 0:
                continue
            #end if


            # A value of -1 means we dont have a disparity error value for the pixel.
            # Otherwise we will select the minimum error
            if self.errorImage[index] == -1:
                self.errorImage[index] = abs(d - actual_disparity)
            else:
                self.errorImage[index] = min(self.errorImage[index], abs(d - actual_disparity))
            #end if

        #end for
    #end def

    # Transform a single laser scan message into the camera optical frame
    def transform_laser_scan(self, laser_msg):

        # Angles are in micro radians. Also we need to handle rollover of the angle
        angle_start = self.normalize_angle(laser_msg.angle_start * 1e-6)
        angle_end = self.normalize_angle(laser_msg.angle_end * 1e-6)
        angle_range = self.normalize_angle(angle_end - angle_start)

        # Hokuyo UTM30-LX-EW scanners range from -135 to 135 degrees
        start_mirror = -135.0 * np.pi /180.
        end_mirror = 135.0 * np.pi/180.

        laserPoints = None

        # Iterate through each laser point and compute the spindle transform
        for i, d in enumerate(laser_msg.distance):

            percentage = i / float(len(laser_msg.distance))

            mirror_angle = start_mirror + percentage * (end_mirror - start_mirror)

            # Data from the Hokuyo is in millimeters. Convert it to meters
            distance = 1e-3 * d

            # The expected laser data coordinate frame has the laser data in the x-z plane
            point = np.matrix([[ distance * np.sin(mirror_angle)],
                               [ 0.],
                               [ distance * np.cos(mirror_angle)],
                               [1.]])

            # Determine the angle of our spindle for this point. We will
            # perform a simple linear interpolation between the start and
            # end spindle angles
            spindle_angle = angle_start + percentage * angle_range

            # The spindle rotates about the z axis of the laser scan data
            spindle_transform = np.matrix([[np.cos(spindle_angle), -np.sin(spindle_angle), 0., 0.],
                                           [np.sin(spindle_angle), np.cos(spindle_angle), 0., 0.],
                                           [0., 0., 1., 0.],
                                           [0., 0., 0., 1.]])

            # Transform our spindle into the camera coordinate frame
            transformed_point = self.spindleToCamera * spindle_transform * self.laserToSpindle * point;

            # Skip points whose Z coordinates are too close to the camera.
            # The minimum range of the MultiSense is 0.4 m
            if transformed_point[2, 0] < 0.4:
                continue
            #end if

            # Add our transformed points to our ouput laser scan
            if laserPoints == None:
                laserPoints = transformed_point
            else:
                laserPoints = np.append(laserPoints, transformed_point, axis=1)
            #end if

        #end for

        return laserPoints
    #end def

    # Normalize angles so it is positive and bound within 0 and 2pi
    def normalize_angle(self, angle):
        # first make sure our angle is positive
        positive_angle = ((angle % (2 * np.pi)) + (2 * np.pi)) % (2 * np.pi)

        # Bound our angle to 2 pi
        if positive_angle > (2 * np.pi):
            return positive_angle - (2 * np.pi)
        #end if

        return positive_angle;
    #end def



def usage(argv):
    print "\nUsage: %s -b <bagfile> [Options]" % argv
    print "Where [Options] are"
    print "     -o <output_directory>\t Specifies theoutput directory to save"\
                +" the output debug files. Default: ."
    print "     -n <namespace>\t\t The namespace the Multisense topics are in. "\
                + "Default: multisense"
    print "     -r\t\t\t\t Rename the input bag file to send to CRL"
    print "     -h\t\t\t\t Print the usage"
# end def

if __name__ == '__main__':
    # Set up default arguments
    bag_file_name = None
    output_directory_name = time.strftime('%Y-%m-%d_%H-%M-%S_process_bags')
    namespace = 'multisense'
    rename_file = False

    try:
        opts, args = getopt.getopt(sys.argv[1:], "b:o:n:rh")
    except getopt.GetoptError as err:
        print str(err)
        usage(sys.argv[0])
        sys.exit(2)
    for o, a in opts:
        if o == "-h":
            usage(sys.argv[0])
            sys.exit()
        elif o == "-r":
            rename_file = True
        elif o == "-b":
            bag_file_name = str(a)
        elif o == "-o":
            output_directory_name = str(a)
        else:
            usage(sys.argv[0])
            sys.exit()
        #end if
    #end for

    if None == bag_file_name:
        print "Error no bag file specified"
        usage(sys.argv[0])
        sys.exit()
    #end if

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
