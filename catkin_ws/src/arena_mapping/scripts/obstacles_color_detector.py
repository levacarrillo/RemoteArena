#!/usr/bin/env python3
import yaml
import rospy
import os
import cv2
import sys
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage

class obstacles_color_detector:
    def __init__(self, config):
        self.subscriber = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.callback, queue_size = 1)
        self.publisher = rospy.Publisher("/tracked_image", Image, queue_size=1)
        self.bridge = CvBridge()
        HU = config['upper_hsv']['H']
        SU = config['upper_hsv']['S']
        VU = config['upper_hsv']['V']
        hl = config['lower_hsv']['H']
        sl = config['lower_hsv']['S']
        vl = config['lower_hsv']['V']

        # Define range of red color in HSV
        self.upper_hsv = np.array([HU, SU, VU])
        self.lower_hsv = np.array([hl, sl, vl])

        cv_no_image = cv2.imread(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + '/assets/images/no_image.jpg')
        gray_img = cv2.cvtColor(cv_no_image, cv2.COLOR_BGR2GRAY)
        self.tracked_image_msg = self.bridge.cv2_to_imgmsg(gray_img , encoding="mono8")

    def callback(self, ros_data):
        np_arr = np.frombuffer(ros_data.data, np.uint8)
        try: 
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            image_hsv = cv2.cvtColor(image_np, cv2.COLOR_BGR2HSV)
            image_gray = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)
            #print("HSV: ", self.upper_hsv, "| hsv: ", self.lower_hsv)

            #Threshold the HSV image to get only red colors
            tracked_img = cv2.inRange(image_hsv, self.lower_hsv, self.upper_hsv)

            #cv2.imshow('real_img', image_np)
            #cv2.imshow('hsv_img', image_hsv)
            #cv2.imshow('gray_img', image_gray)
            #cv2.imshow('tracked_img', tracked_img)
            #cv2.waitKey(1)
            
            self.tracked_image_msg = self.bridge.cv2_to_imgmsg(tracked_img, encoding="mono8")
            self.publisher.publish(self.tracked_image_msg)
        except CvBridgeError as e:
            rospy.logerr(f"There was an error converting image: {e}")
        except cv2.error as e:
            rospy.logerr(f"There was an error decoding image: {e}")
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


def main():
    print("Starting obstacles_color_detector by Luis Nava.")
    config_data = load_config()
    ic = obstacles_color_detector(config_data)
    rospy.init_node('obstacles_color_detector', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()