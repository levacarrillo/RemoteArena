from flask import session
from .persistence.DataBase import *

class User:
    def __init__(self, app):
        self.name = ''
        self.session_active = False
        self.db = DataBase(app)
        self.users = self.db.get_users()

    def login(self, request):
        username = request.form['username']
        password = request.form['password']
        if username in self.users:
            session['username'] = username
            self.session_active = True
            self.name = username
            return True

        return False

    def logout(self, username):
        session.pop('username', None)
        self.name = ''
        self.session_active = False
