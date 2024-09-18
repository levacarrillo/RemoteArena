from flask import session
from .persistence.DataBase import *

class User:
    def __init__(self, app):
        self.name = ''
        self.admin = False
        self.session_active = False
        self.db = DataBase(app)
        self.users = self.db.get_users()

    def login(self, request):
        username = request.form['username']
        password = request.form['password']

        for user_data in self.users:
            if username == user_data['username'] and  password == user_data['password']:
                session['username'] = username
                self.name = user_data['username']
                self.admin = user_data['admin']
                self.session_active = True
                return True

        return False

    def logout(self, username):
        session.pop('username', None)
        self.name = ''
        self.session_active = False
