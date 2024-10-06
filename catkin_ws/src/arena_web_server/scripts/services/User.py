from .persistence.DataBase import *

class User:
    def __init__(self, app):
        self.name = ''
        self.db = DataBase(app)
        # self.users = self.db.get_users()

    def login(self, request):
        # username = request.form['username']
        # password = request.form['password']
        user_loaded = self.db.load_user()
        print(user_loaded)
        return {}

    def logout(self, username):
        # session.pop('username', None)
        self.name = ''
