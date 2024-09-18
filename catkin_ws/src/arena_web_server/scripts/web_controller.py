#!/usr/bin/env python3
from flask import Flask, render_template, request, redirect, url_for, jsonify
from services.User import * 
from services.Data import *
from services.ROS import *


app = Flask(__name__, static_folder = '../templates/static', template_folder = '../templates')
port = 4000

# VIEW
@app.route('/', methods=['GET', 'POST'])
def index():
    return redirect(url_for('home'))

@app.route('/home', methods=['GET'])
def home():
    if user.session_active:
        return render_template('home.html', username=user.name, admin=user.admin, programFiles=data.get_file_list())
    return redirect(url_for('login'))

@app.route('/recordings', methods=['GET'])
def recordings():
    return render_template('recordings.html',  username=user.name, admin=user.admin)

@app.route('/userManager')
def userManager():
    if not user.admin: 
        return redirect(url_for('home'))
    return render_template('userManager.html',  username=user.name, admin=user.admin)

@app.route('/fileManager')
def fileManager():
    if not user.admin: 
        return redirect(url_for('home'))
    return render_template('fileManager.html',  username=user.name, admin=user.admin)


@app.route('/login', methods=['GET', 'POST'])
def login():
    if request.method == 'POST' and user.login(request):
            return redirect(url_for('index'))

    return render_template('login.html')

@app.route('/logout', methods=['GET'])
def logout():
    user.logout('administrator')
    return redirect(url_for('login'))


# PERSISTENCE
@app.route('/get_users', methods=['GET'])
def get_users():
    users = data.get_users()
    return jsonify(users)

@app.route('/get_file_list', methods=['GET'])
def get_file_list():
    return jsonify(data.get_file_list())

@app.route('/save_file', methods=['POST'])
def save_file():
    return jsonify(data.save_file(user.name, request))


# ROS
@app.route('/set_light_bulbs_state', methods=['POST'])
def set_light_bulbs_state():
    return jsonify(ros.set_light_bulbs_state(request.get_json()))

@app.route('/get_battery_percent', methods=['GET'])
def get_battery_percent():
    return jsonify(ros.batt_percent)

@app.route('/move_robot_command', methods=['POST'])
def move_robot_command():
    request_data = request.get_json()
    print(request_data)
    movement = request_data['movement']
    print(movement)
    return jsonify(movement)

@app.route('/run_executable', methods=['POST'])
def run_executable():
    request_data = request.get_json()
    print(request_data)
    command = request_data['command']
    print(command)
    return jsonify(command)


if __name__ == '__main__':
    user = User(app)
    data = Data()
    ros = ROS()
    app.run(debug=True, use_reloader=True, port=port)
