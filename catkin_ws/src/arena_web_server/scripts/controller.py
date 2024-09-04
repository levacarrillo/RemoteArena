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
        return render_template('home.html', username=user.name)
    return redirect(url_for('login'))

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
    return data.get_file_list()

# ROS

if __name__ == '__main__':
    user = User(app)
    data = Data()
    # ros = ROS(app)
    app.run(debug=True, use_reloader=True, port=port)