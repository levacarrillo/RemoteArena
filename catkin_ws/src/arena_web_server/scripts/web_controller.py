#!/usr/bin/env python3
from flask import Flask, render_template, request, redirect, url_for, jsonify
from flask_cors import CORS
from services.User import * 
from services.Data import *
from services.ROS import *


app = Flask(__name__, static_folder = '../templates/static', template_folder = '../templates')
CORS(app)
port = 4000

# USER
@app.route('/login', methods=['POST'])
def login():
    user.login(request)
    return jsonify({})

@app.route('/logout', methods=['GET'])
def logout():
    return jsonify({})

# PERSISTENCE
@app.route('/get_users', methods=['POST'])
def get_users():
    users = data.get_users()
    return jsonify(users)

@app.route('/get_assignments', methods=['POST'])
def get_assignments():
    return jsonify(data.get_assignments())

@app.route('/get_assignment_files', methods=['POST'])
def get_assignment_files():
    return jsonify(data.get_assignment_files())

# @app.route('/get_file_list', methods=['GET'])
# def get_file_list():
#     return jsonify(data.get_file_list())

# @app.route('/save_file', methods=['POST'])
# def save_file():
#     return jsonify(data.save_file(user.name, request))

@app.route('/get_about', methods=['GET'])
def get_about():
    return jsonify(data.get_about())

# ROS
@app.route('/set_light_bulbs_state', methods=['POST'])
def set_light_bulbs_state():
    return jsonify(ros.set_light_bulbs_state(request.get_json()))

@app.route('/get_selected_robot', methods=['GET'])
def get_selected_robot():
    return jsonify(ros.selected_robot)

@app.route('/get_battery_percent', methods=['GET'])
def get_battery_percent():
    return jsonify(ros.batt_percent)

@app.route('/get_cpu_temp', methods=['GET'])
def get_cpu_temp():
    return jsonify(ros.cpu_temp)

@app.route('/get_laser_values', methods=['GET'])
def get_laser_values():
    return jsonify(ros.laser_scan)

@app.route('/get_max_light_sensor')
def get_max_light_sensor():
    return jsonify(ros.get_max_light_sensor())

@app.route('/move_robot_command', methods=['POST'])
def move_robot_command():
    return jsonify(ros.move_robot(request.get_json()))

@app.route('/move_robot_to_pose', methods=['POST'])
def move_robot_to_pose():
    return jsonify(ros.move_robot_to_pose(request.get_json()))


@app.route('/run_executable', methods=['POST'])
def run_executable():
    return jsonify(ros.run_command(request.get_json()))


if __name__ == '__main__':
    user = User(app)
    data = Data()
    ros = ROS()
    app.run(debug=True, use_reloader=True, port=port)
