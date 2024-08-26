#!/usr/bin/env python3
import rospy
import os
from flask import Flask, render_template, request, jsonify
from arena_control.srv import LightBulbsControl

app = Flask(__name__, static_folder = '../templates/static', template_folder = '../templates')

lightBulbsState = [0, 0]

@app.route('/viewer')
def viewer():
    return render_template('viewer.html')

@app.route('/home')
def home():
    return render_template('home.html')

@app.route('/index')
def index():
    return render_template('index.html')

@app.route('/robot_control_command/<command>', methods=['POST'])
def robot_control_command(command):
    print('ros-command:' + command)
    if not rospy.is_shutdown():
        pub.publish(command)
    return jsonify(message='Command->' + command)

@app.route('/light_control_command/<id>/<state>', methods=['POST'])
def light_control_command(id, state):
    print('light: ' + id + ' turn on: ' + state)
    rospy.wait_for_service('light_bulbs_state')
    try:
        lights_control = rospy.ServiceProxy('light_bulbs_state', LightBulbsControl)
        if id == "bulb_1":
            lightBulbsState[0] = int(state.lower() in ["true"])
        else:
            lightBulbsState[1] = int(state.lower() in ["true"])

        resp = lights_control(lightBulbsState)
        print(resp)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    return jsonify(message='Command->')

@app.route('/upload', methods=['POST'])
def upload_file():

    if 'file' not in request.files:
        print('No file part')
        return 'No file part'

    file = request.files['file']


    if file.filename == '':
        print('No selecte file')
        return 'No selected file'
    
    if file: 
        user_name = os.getlogin()
        abspath = f'/home/{user_name}/ProgramsFile'
        if not os.path.exists(abspath):
            os.makedirs(abspath)
        print('Saving file->', file.filename, 'at->', abspath)
        file_path = os.path.join(abspath, file.filename)
        file.save(file_path)

    return 'file successfully added'

def start_ros_node():
    global pub, light_pub
    rospy.init_node('web_server_node', anonymous=True)

if __name__ == '__main__':
    start_ros_node()
    app.run(debug=True, use_reloader=True, port=4000)
