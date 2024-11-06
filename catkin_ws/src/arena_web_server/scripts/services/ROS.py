import rospy
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class ROS:
    def __init__(self):
        rospy.init_node('controller_node', anonymous=True)
        rospy.Subscriber('battery_percent', String, self.batt_callback)
        rospy.Subscriber('cpu_temp', String, self.cpu_temp_callback)
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.selected_robot = rospy.get_param(rospy.search_param('selected_robot'))
        self.lightBulbsState = [False, False]
        self.batt_percent = "0%"
        self.cpu_temp = "--"

        self.rate = rospy.Rate(10)
        self.movement_time = 1.3 #sec
        self.linear_vel  = 0.2
        self.angular_vel = 0.4

    def cpu_temp_callback(self, data):
        self.cpu_temp = data.data + '°C'

    def batt_callback(self, data):
        self.batt_percent = data.data

    def run_command(self, rest_data):
        exec_command = rest_data['command']
        if exec_command == 'run_algorithm':
            rospy.set_param('run_algorithm', True)
        elif exec_command == 'stop_algorithm':
            rospy.set_param('run_algorithm', False)

    def move_robot(self, rest_data):
        move_command = rest_data['command']
        twist_msg = Twist()
        print(move_command)

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

    def set_light_bulbs_state(self, rest_data):
        lights_array = rest_data['lights']
        if 1 in lights_array:
            rospy.set_param('light_bulbs', [True, False])
            if 2 in lights_array:
                rospy.set_param('light_bulbs', [True, True])
        elif 2 in lights_array:
            rospy.set_param('light_bulbs', [False, True])
        else:
            rospy.set_param('light_bulbs', [False, False])
