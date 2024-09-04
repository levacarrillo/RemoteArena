from .persistence.DataBase import *

class Data:
    def __init__(self):
            self.db = DataBase()

    def get_users(self):
        try:
            users = self.db.get_users()
            return users
        except Exception as error:
            print('There was an error in: Data.get_users()->', error)
            return {}

    def get_file_list(self):
        try:
            file_list = self.db.get_get_file_list()
            return file_list
        except Exception as error:
            print('There was an error in: Data.get_file_list()->', error)
            return []
