#!/usr/bin/env python

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import CompressedImage

class obstacles_detector:
    def __init__(self):
        self.subscriber = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.callback, queue_size = 1)
        cv2.namedWindow('mask')
        cv2.createTrackbar('Upper H','mask', 65,  255, self.nothing)
        cv2.createTrackbar('Upper S','mask', 99,  255, self.nothing)
        cv2.createTrackbar('Upper V','mask', 156, 255, self.nothing)
        cv2.createTrackbar('Lower h','mask', 41,  255, self.nothing)
        cv2.createTrackbar('Lower s','mask', 40,  255, self.nothing)
        cv2.createTrackbar('Lower v','mask', 140, 255, self.nothing)

    def callback(self, ros_data):
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        image_hsv = cv2.cvtColor(image_np, cv2.COLOR_BGR2HSV)
        image_gray = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)

        HU= int(cv2.getTrackbarPos('Upper H','mask'))
        SU= int(cv2.getTrackbarPos('Upper S','mask'))
        VU= int(cv2.getTrackbarPos('Upper V','mask'))
        hl= int(cv2.getTrackbarPos('Lower h','mask'))
        sl= int(cv2.getTrackbarPos('Lower s','mask'))
        vl= int(cv2.getTrackbarPos('Lower v','mask'))

        #print("HU: " + str(HU) + ", SU: " + str(SU) + ", VU: " + str(VU) + ", hl: " + str(hl) + ", sl: " + str(sl) + ", vl: " + str(vl))
        
        # define range of red color in HSV
        upper_hsv = np.array([HU, SU, VU])  
        lower_hsv = np.array([hl, sl, hl])
    
        #Threshold the HSV image to get only red colors
        mask = cv2.inRange(image_hsv, lower_hsv, upper_hsv)


        cv2.imshow('real_img', image_np)
        #cv2.imshow('hsv_img', image_hsv)
        #cv2.imshow('gray_img', image_gray)
        cv2.imshow('mask', mask)
        cv2.waitKey(2)
    
    #trackbar callback fucntion does nothing but required for trackbar
    def nothing(x):
        pass

def main():
    print("Starting obstacles_detector by Luis Nava.")
    ic = obstacles_detector()
    rospy.init_node('obstacles_detector', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()