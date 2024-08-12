#!/usr/bin/env python3
import numpy as np
import yaml
import cv2
import rospy
import os
from sensor_msgs.msg import CompressedImage

class obstacles_color_calibrator:
    def __init__(self, config):
        self.subscriber = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.callback, queue_size = 1)
        cv2.namedWindow('mask')
        cv2.createTrackbar('Upper H','mask', config['upper_hsv']['H'],  255, self.nothing)
        cv2.createTrackbar('Upper S','mask', config['upper_hsv']['S'],  255, self.nothing)
        cv2.createTrackbar('Upper V','mask', config['upper_hsv']['V'], 255, self.nothing)
        cv2.createTrackbar('Lower h','mask', config['lower_hsv']['H'],  255, self.nothing)
        cv2.createTrackbar('Lower s','mask', config['lower_hsv']['S'],  255, self.nothing)
        cv2.createTrackbar('Lower v','mask', config['lower_hsv']['V'], 255, self.nothing)
        self.upper_hsv = np.array([255, 255, 255])
        self.lower_hsv = np.array([0, 0, 0])

    def callback(self, ros_data):
        np_arr = np.frombuffer(ros_data.data, np.uint8)
        try: 
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            image_hsv = cv2.cvtColor(image_np, cv2.COLOR_BGR2HSV)
            image_gray = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)

            HU= int(cv2.getTrackbarPos('Upper H','mask'))
            SU= int(cv2.getTrackbarPos('Upper S','mask'))
            VU= int(cv2.getTrackbarPos('Upper V','mask'))
            hl= int(cv2.getTrackbarPos('Lower h','mask'))
            sl= int(cv2.getTrackbarPos('Lower s','mask'))
            vl= int(cv2.getTrackbarPos('Lower v','mask'))
            
            # Define range of red color in HSV
            self.upper_hsv = np.array([HU, SU, VU])
            self.lower_hsv = np.array([hl, sl, vl])
            #print("HSV: ", self.upper_hsv, "| hsv: ", self.lower_hsv)

            #Threshold the HSV image to get only red colors
            mask = cv2.inRange(image_hsv, self.lower_hsv, self.upper_hsv)


            cv2.imshow('real_img', image_np)
            #cv2.imshow('hsv_img', image_hsv)
            #cv2.imshow('gray_img', image_gray)
            cv2.imshow('mask', mask)
            cv2.waitKey(1)
        except cv2.error as e:
            rospy.logerr(f"There was an error decoding image: {e}")

    #trackbar callback function does nothing but required for trackbar
    def nothing(self, x):
        pass

def load_config():
    filePath = os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + '/assets/params/hsv_config.yaml'
    config_data = {
        'upper_hsv': {'H': 255, 'S': 255, 'V': 255},
        'lower_hsv': {'H': 0, 'S': 0, 'V': 0}
    }
    try:
        with open(filePath, 'r') as file:
            config_data = yaml.safe_load(file)
    except:
        print("No param file")
    return config_data

def save_hsv_values(upper_hsv, lower_hsv):
    print("Saving HSV values:")
    HU, SU, VU = upper_hsv
    hl, sl, vl = lower_hsv
    config_data = {
        'upper_hsv': {'H': int(HU), 'S': int(SU), 'V': int(VU)},
        'lower_hsv': {'H': int(hl), 'S': int(sl), 'V': int(vl)}
    }
    print(config_data)
    filePath = os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + '/assets/params/'

    if not os.path.exists(filePath):
        os.makedirs(filePath)

    with open(filePath + 'hsv_config.yaml', 'w') as file:
        yaml.dump(config_data, file)


def main():
    print("Starting obstacles_color_calibrator by Luis Nava.")
    config_data = load_config()
    ic = obstacles_color_calibrator(config_data)
    rospy.init_node('obstacles_color_calibrator', anonymous=True)
    try:
        rospy.spin()
        save_hsv_values(ic.upper_hsv, ic.lower_hsv)
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()