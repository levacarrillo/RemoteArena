import rospy
from arena_control.srv import LightBulbsControl
from std_msgs.msg import String
from flask_socketio import SocketIO, emit

class ROS:
    def __init__(self, app):
        print('Setting ROS class')
        rospy.init_node('controller_node', anonymous=True)
        rospy.Subscriber('batt_percent', String, self.batt_callback)
        self.lightBulbsState = [False, False]
        socketio = SocketIO(app)
        # socketio.run(app, host='0.0.0.0', port=4000)

    def batt_callback(self, data):
        print('from callback', data.data)
        socketio.emit('batt-data', { 'data': data.data })

    def set_light_bulbs_state(self):
        rospy.wait_for_service('light_bulbs_state')

        try:
            lights_control = rospy.ServiceProxy('light_bulbs_state', LightBulbsControl)
            
            response = lights_control(self.lightBulbsState)
        except rospy.ServiceException as error:
            print('Service call failed: %s'%e)

        return response.success