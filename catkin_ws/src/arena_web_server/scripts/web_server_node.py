#!/usr/bin/env python3
import rospy
import os
from flask import Flask, render_template, request, jsonify
from arena_control.srv import LightBulbsControl
from models import db, Files

app = Flask(__name__, static_folder = '../templates/static', template_folder = '../templates')
port = 5000
app.config['SQLALCHEMY_DATABASE_URI']= 'postgresql+psycopg2://usuario:password@localhost:5432/programs'
app.config['SQLALCHEMY_TRACK_MODIFICATIONS']=False

lightBulbsState = [False, False]

@app.route('/')
def home():
    programFiles =[]
    user_name = os.getlogin()
    abspath = f'/home/{user_name}/GlusterMR/Programs'
    programFiles = [f for f in os.listdir(abspath) if os.path.isfile(os.path.join(abspath, f))]
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

@app.route('/programFile', methods=['POST'])
def add_program_file():
    try:
        data = request.json
        print(data)
        user = data.get('user')
        document_name = data.get('document_name')
        algorithm = data.get('algorithm')
        status = data.get('status')
        path = data.get('path')
        upload_date = data.get('upload_date')
        if not document_name:
            return jsonify({'message': 'Bad request, isbn or name or cantPages or author not found'}), 400
        new_file = Files(user=user, document_name=document_name, algorithm=algorithm, status=status, path=path, upload_date=upload_date)
        db.session.add(new_file)
        db.session.commit()
        return jsonify({'File': {'id': new_file.id, 'user': new_file.user, 'document_name': new_file.document_name, 'algorithm': new_file.algorithm, 'status': new_file.status, 'path': new_file.path, 'upload_date': new_file.upload_date}})
    except Exception as error:
        print('Error', error)
        return jsonify({'message': 'Internal server error'}), 500

@app.route('/get_programs', methods=['GET'])
def get_programs():
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
        return jsonify({'files': files_data})
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
