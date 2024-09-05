import rospy
from arena_control.srv import LightBulbsControl
from std_msgs.msg import String
#from flask_socketio import SocketIO, emit

class ROS:
    def __init__(self):
        rospy.init_node('controller_node', anonymous=True)
        rospy.Subscriber('batt_percent', String, self.batt_callback)
        self.lightBulbsState = [False, False]
        self.batt_percent = "0%"
        #self.socketio = SocketIO(app)
        #self.socketio.run(app, host='0.0.0.0', port=4000)

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
