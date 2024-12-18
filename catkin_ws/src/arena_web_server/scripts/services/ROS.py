import rospy
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from mobile_base.srv import MoveMinibot
from hardware.srv import LightReadings

class ROS:
    def __init__(self):
        rospy.init_node('controller_node', anonymous=True)
        rospy.Subscriber('/hardware/battery_percent', String, self.batt_callback)
        rospy.Subscriber('/hardware/cpu_temp', String, self.cpu_temp_callback)
        rospy.Subscriber('/hardware/scan', LaserScan, self.scan_callback)
        rospy.wait_for_service('/mobile_base/move_to_pose')
        rospy.wait_for_service('/hardware/light_readings')
        self.pub_vel = rospy.Publisher('/mobile_base/cmd_vel', Twist, queue_size=10)
        self.selected_robot = rospy.get_param(rospy.search_param('/hardware/robot_id'))
        self.lightBulbsState = [False, False]
        self.batt_percent = "0%"
        self.cpu_temp = "--"
        self.laser_scan = []
        self.selected_behavior = "light_follower"

        self.rate = rospy.Rate(10)
        self.movement_time = 1.3 #sec
        self.linear_vel  = 0.15
        self.angular_vel = 1.2

    def cpu_temp_callback(self, data):
        self.cpu_temp = data.data + '°C'

    def batt_callback(self, data):
        self.batt_percent = data.data

    def scan_callback(self, data):
        self.laser_scan = data.ranges

    def select_behavior(self, rest_data):
        self.selected_behavior = rest_data['behavior']

    def run_command(self, rest_data):
        exec_command = rest_data['command']
        if exec_command == 'run_algorithm':
            rospy.set_param('/motion_planner/behavior', self.selected_behavior)
            rospy.set_param('/mobile_base/enable_movements', True)
        elif exec_command == 'stop':
            rospy.set_param('/mobile_base/enable_movements', False)

    def move_robot(self, rest_data):
        move_command = rest_data['command']
        twist_msg = Twist()
        # print(move_command)

        if move_command == 'foward':
            twist_msg.linear.x  = self.linear_vel
        elif move_command == 'left':
            twist_msg.angular.z = self.angular_vel
        elif move_command == 'right':
            twist_msg.angular.z = -self.angular_vel
        elif move_command == 'stop':
            twist_msg.linear.x  = 0.0
            twist_msg.angular.z = 0.0
        elif move_command == 'backward':
            twist_msg.linear.x  = - self.linear_vel

        start_time = time.time()
        while not rospy.is_shutdown() and (time.time() - start_time < self.movement_time):
            self.pub_vel.publish(twist_msg)
            self.rate.sleep()
        
        twist_msg.linear.x  = 0.0
        twist_msg.angular.z = 0.0
        self.pub_vel.publish(twist_msg)

    def move_robot_to_pose(self, rest_data):
        angle = rest_data['angle']
        distance = rest_data['distance']
        rospy.set_param('/motion_planner/behavior', 'none')
        rospy.set_param('/mobile_base/enable_movements', True)
        try:
            movement = rospy.ServiceProxy('/mobile_base/move_to_pose', MoveMinibot)
            resp = movement(angle, distance)
            # print('' + resp)
            return resp.done
        except rospy.ServiceException as error:
            print('Service failed to call service /mobile_base/move_to_pose %s' %error)

    def get_max_light_sensor(self):
        try:
            client = rospy.ServiceProxy('/hardware/light_readings', LightReadings)
            resp = client()
            # print(resp)
            return resp.sensor_max_intensity
        except rospy.ServiceException as error:
            print('Service failed to call /hardware/light_readings service')
            

    def set_light_bulbs_state(self, rest_data):
        # print(rest_data)
        lights_array = rest_data['lights']
        if 1 in lights_array:
            rospy.set_param('/hardware/light_bulbs', [True, False])
            if 2 in lights_array:
                rospy.set_param('/hardware/light_bulbs', [True, True])
        elif 2 in lights_array:
            rospy.set_param('/hardware/light_bulbs', [False, True])
        else:
            rospy.set_param('/hardware/light_bulbs', [False, False])
