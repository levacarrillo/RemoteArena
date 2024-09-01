#!/usr/bin/env python3
import rospy
import os
from flask import Flask, render_template, request, jsonify, session, redirect, url_for
from arena_control.srv import LightBulbsControl
from models import db, Files
import json

app = Flask(__name__, static_folder = '../templates/static', template_folder = '../templates')
port = 5000
app.config['SQLALCHEMY_DATABASE_URI']= 'postgresql+psycopg2://usuario:password@localhost:5432/programs'
app.config['SQLALCHEMY_TRACK_MODIFICATIONS']=False

lightBulbsState = [False, False]
programFiles = []

@app.route('/')
def index():
    if 'user' in session:
        return render_template('home.html', programFiles=programFiles)
    return redirect(url_for('login'))

@app.route('/login', methods=['GET', 'POST'])
def login():
    
    return render_template('login.html')

@app.route('/home')
def home():
    #programFiles =[]
    user_name = os.getlogin()
    abspath = f'/home/{user_name}/GlusterMR/Programs'
    programFiles = get_file_list()
    print(programFiles)
    
    # programs = ['program 1', 'program 2']
    return render_template('home.html', programFiles=programFiles)

@app.route('/run_command', methods=['POST'])
def run_command():
    request_data = request.get_json()
    print(request_data)
    command = request_data['command']
    print(command)

    # rospy.wait_for_service('move_robot_command')
    # try:
    #     robot_command = rospy.ServiceProxy('move_robot_command', MoveRobotCommand)
    #     response = robot_command(movement)
    # except rospy.ServiceException as e:
    #     print("Service call failed: %s"%e)

    # return jsonify(message='Command success: ' + str(response.success))
    return jsonify(message='Command success: ' + command)

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
    #data = request.files["file"]
    
    if 'file' not in request.files:
        print('No file part')
        return 'No file part'

    file = request.files['file']


    if file.filename == '':
        print('No selecte file')
        return 'No selected file'
    
    try:
        dataForm = json.loads(request.form["jsonData"])
        print(dataForm)

        username = dataForm['username']
        document_name = dataForm['document_name']
        algorithm = dataForm['algorithm']
        status = dataForm['status']
        file_path = dataForm['file_path']

        if not document_name:
            return jsonify({'message': 'Bad request, please check data.'}), 400

        new_file = Files(username=username, document_name=document_name, algorithm=algorithm, status=status, file_path=file_path)
        db.session.add(new_file)
        db.session.commit()

        if file: 
            user_name = os.getlogin()
            abspath = f'/home/{user_name}/GlusterMR/Programs'
            if not os.path.exists(abspath):
                os.makedirs(abspath)
            #print('Saving file->', file.filename, 'at->', abspath)
            file_path = os.path.join(abspath, file.filename)
            file.save(file_path)
        
        programFiles = get_file_list()
        return jsonify('message', 'File added successfully.'), 200 

    except Exception as error:
        print('Error', error)
        return jsonify({'message', 'Internal server error.'}), 500
        
def get_file_list():
    try:
        files = Files.query.all()
        files_data = []
        
        for file in files:
            file_data = {
                'id': file.id,
                'username': file.username,
                'document_name': file.document_name,
                'algorithm': file.algorithm,
                'status': file.status,
                'file_path': file.file_path,
                'upload_date': file.upload_date
            }
            files_data.append(file_data)
        return (files_data)
    except Exception as error:
        print('Error', error)
        return []

@app.route('/get_files_list', methods=['GET'])
def get_files_list():
    return jsonify(get_file_list()), 200


@app.route('/list_program_files', methods=['GET'])
def list_program_files():
    try:
        files = Files.query.all()
        files_data = []
        
        for file in files:
            file_data = {
                'id': file.id,
                'user': file.user,
                'document_name': file.document_name,
                'algorithm': file.algorithm,
                'status': file.status,
                'path': file.path,
                'upload_date': file.upload_date
            }
            files_data.append(file_data)
        return jsonify(files_data)
    except Exception as error:
        print('Error', error)
        return jsonify({'message': 'Internal server error'}), 500

def start_ros_node():
    rospy.init_node('web_server_node', anonymous=True)

if __name__ == '__main__':
    # STARTING ROS NODE
    start_ros_node()
    # CONNECTING WITH DATABASE
    db.init_app(app)
    with app.app_context():
        db.create_all()
    app.run(debug=True, use_reloader=True, port=port)
