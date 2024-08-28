#!/usr/bin/env python3
import rospy
import os
from flask import Flask, render_template, request, jsonify
from arena_control.srv import LightBulbsControl

app = Flask(__name__, static_folder = '../templates/static', template_folder = '../templates')

lightBulbsState = [False, False]

@app.route('/viewer')
def viewer():
    return render_template('viewer.html')

@app.route('/')
def home():
    return render_template('home.html')

@app.route('/index')
def index():
    return render_template('index.html')

@app.route('/move_robot_command', methods=['POST'])
def move_robot_command():
    request_data = request.get_json()
    print(request_data)
    movement = request_data['movement']
    print(movement)

    # rospy.wait_for_service('move_robot_command')
    # try:
    #     robot_command = rospy.ServiceProxy('move_robot_command', MoveRobotCommand)
    #     response = robot_command(movement)
    # except rospy.ServiceException as e:
    #     print("Service call failed: %s"%e)

    # return jsonify(message='Command success: ' + str(response.success))
    return jsonify(message='Command success: ' + movement)

@app.route('/light_bulbs_control', methods=['POST'])
def light_control_command():
    global lightBulbsState
    request_data = request.get_json()
    # print(request_data)
    id = request_data['id']
    state = request_data['state']
    # print("id->", id, 'state->', state)

    rospy.wait_for_service('light_bulbs_state')

    try:
        lights_control = rospy.ServiceProxy('light_bulbs_state', LightBulbsControl)
        if id == "bulb_1":
            lightBulbsState[0] = state
        elif id == "bulb_2":
            lightBulbsState[1] = state
        else:
            lightBulbsState = [False, False]

        response = lights_control(lightBulbsState)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    return jsonify(message='Command success: ' + str(response.success))

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
        abspath = f'/home/{user_name}/GlusterMR/Programs'
        if not os.path.exists(abspath):
            os.makedirs(abspath)
        #print('Saving file->', file.filename, 'at->', abspath)
        file_path = os.path.join(abspath, file.filename)
        file.save(file_path)

    return 'file successfully added'

def start_ros_node():
    rospy.init_node('web_server_node', anonymous=True)

if __name__ == '__main__':
    start_ros_node()
    app.run(debug=True, use_reloader=True, port=4000)
