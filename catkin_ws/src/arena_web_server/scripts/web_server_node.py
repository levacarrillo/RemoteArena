#!/usr/bin/env python3
import rospy
from flask import Flask, render_template, jsonify
from std_msgs.msg import String

app = Flask(__name__, static_folder = '../templates/static', template_folder = '../templates')

@app.route('/viewer')
def home():
    return render_template('index.html')

@app.route('/executed_command/<command>', methods=['POST'])
def executed_command(command):
    print('ros-command:' + command)
    if not rospy.is_shutdown():
        pub.publish(command)
    return jsonify(message='Command->' + command)

def start_ros_node():
    global pub
    rospy.init_node('web_server_node', anonymous=True)
    pub = rospy.Publisher('ros_publisher', String, queue_size=10)

if __name__ == '__main__':
    start_ros_node()
    app.run(debug=True, use_reloader=True, port=4000)