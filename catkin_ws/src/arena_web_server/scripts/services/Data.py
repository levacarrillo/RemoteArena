from .persistence.DataBase import *
import os
import json

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

    def get_assignments(self):
        try:
            assignments = self.db.get_assignments()
            return assignments
        except Exception as error:
            print('There was an error in: Data.get_assignments()->', error)
            return {}

    def get_assignment_files(self):
        try:
            files = self.db.get_assignment_files()
            return files
        except Exception as error:
            print('There was an error in: Data.get_assignment_files()->', error)
            return {}

    def get_about(self):
        try:
            about = self.db.get_about()
            return about
        except Exception as error:
            print('There was an error in: Data.get_about()->', error)
            return {}

    # def get_file_list(self):
    #     try:
    #         file_list = self.db.get_file_list()
    #         return file_list
    #     except Exception as error:
    #         print('There was an error in: Data.get_file_list()->', error)
    #         return []

    # def save_file(self, username, request):
    #     if 'file' not in request.files:
    #         return 'No file part'

    #     file = request.files['file']

    #     if file.filename == '':
    #         return 'No selected file'

    #     dataForm = json.loads(request.form['dataForm'])

    #     try:
    #         file_name = dataForm['file_name']
    #         file_path = dataForm['file_path']
    #         algorithm = dataForm['algorithm']
    #         print(f"file_name: {file_name} | file_path: {file_path} | algorithm {algorithm}")

    #         if not file_name:
    #             return 'Bad request, please check data'
            
    #         if file:
    #             user_name = os.getlogin()
    #             abspath = f'/home/{user_name}/GlusterMR/Programs' + file_path
                
    #             if not os.path.exists(abspath):
    #                 os.makedirs(abspath)
    #             print('Saving file->', file.filename, ' at->', abspath)
    #             file.save(os.path.join(abspath, file_name))

    #             # SAVING IN DATABASE
    #             self.db.save_file(username, file_name, file_path, algorithm)

    #         return 'File added successfully'
    #     except Exception as error:
    #         print('There was an error in: Data.save_file()->', error)
    #         return 'Internal server error'
