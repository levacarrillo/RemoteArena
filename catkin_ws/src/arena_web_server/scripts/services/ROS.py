import rospy
from arena_control.srv import LightBulbsControl
from std_msgs.msg import String

class ROS:
    def __init__(self):
        rospy.init_node('controller_node', anonymous=True)
        rospy.Subscriber('battery_percent', String, self.batt_callback)
        rospy.Subscriber('cpu_temp', String, self.cpu_temp_callback)
        self.lightBulbsState = [False, False]
        self.selected_robot = rospy.get_param(rospy.search_param('selected_robot')) 
        self.batt_percent = "0%"
        self.cpu_temp = "--"

    def cpu_temp_callback(self, data):
        self.cpu_temp = data.data + '°C'

    def batt_callback(self, data):
        # print('from callback', data.data)
        self.batt_percent = data.data
        #self.socketio.emit('batt-data', { 'data': data.data })

    def set_light_bulbs_state(self, light_state):
        id = light_state['id']
        state = light_state['state']
        rospy.wait_for_service('light_bulbs_state')

        try:
            lights_control = rospy.ServiceProxy('light_bulbs_state', LightBulbsControl)
            if id == "bulb_1":
                self.lightBulbsState[0] = state
            elif id == "bulb_2":
                self.lightBulbsState[1] = state
            else:
                self.lightBulbsState = [False, False]
            response = lights_control(self.lightBulbsState)
            return response.success    
        except rospy.ServiceException as error:
            print('Service call failed at ROS.set_light_bulbs_state(): %s'%e)
            return "Internal server error"
