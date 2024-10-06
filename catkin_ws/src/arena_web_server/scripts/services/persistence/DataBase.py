from .models.ModelsDB import Users, Assignments, AssignmentFiles, About, db
from dotenv import load_dotenv
import os
import rospkg

class DataBase:
    def __init__(self, app=None):
        self.admin = 1
        self.professor = 2
        self.student = 3

        if app is not None:
            self.app = app
            rospack = rospkg.RosPack()
            relative_path = '/scripts/services/persistence/server.env'
            pathenv = rospack.get_path('arena_web_server') + relative_path;
            load_dotenv(dotenv_path=pathenv)
            
            db_key  = os.getenv("SECRET_KEY")
            db_user = os.getenv("DB_USER")
            db_password = os.getenv("DB_PASSWORD")
            db_host = os.getenv("DB_HOST")
            db_port = os.getenv("DB_PORT")
            db_name = os.getenv("DB_NAME")
            print(f"Connecting database at {db_host}:{db_port} with {db_user}")

            app.secret_key = db_key
            app.config['SQLALCHEMY_DATABASE_URI']= f"postgresql+psycopg2://{db_user}:{db_password}@{db_host}:{db_port}/{db_name}"
            app.config['SQLALCHEMY_TRACK_MODIFICATIONS']=False
            db.init_app(app)
            with app.app_context():
                db.create_all()
            app.app_context().push()

    def load_user(self):
        try:
            user = Users.query.get(1)
            print(user)
            if user:
                user_dict = {
                    'username': user.username,
                    'names': user.names,
                    'role': user.roles.name
                }
                return user_dict;
            else:
                return {}
        except Exception as error:
            print('Error in DataBase.load_user(): ', error)
            return {}

    def get_users(self):
        try:
            user_list = []
            users = Users.query.filter(Users.role_id==self.student, Users.active==True)
            for user in users:
                user_data = {
                    'username': user.username,
                    'account_id': user.account_id,
                }
                user_list.append(user_data)
            return user_list
        except Exception as error:
            print('Error in DataBase.get_users(): ', error)
            return []

    def get_assignments(self):
        try:
            assignment_list = []
            assignments = Assignments.query
            assignments = assignments.join(Users, Assignments.professor_id == Users.id)
            assignments = assignments.all()
            for assignment in assignments:
                assignment_data = {
                    'id': assignment.id,
                    'title': assignment.title,
                    'description': assignment.description,
                    'due_date': assignment.due_date,
                    'professor_names': assignment.professor.names,
                }
                assignment_list.append(assignment_data)
            return assignment_list
        except Exception as error:
            print('Error in DataBase.get_assignments():', error)
            return []

    def get_assignment_files(self):
        try:
            file_list = []
            files = AssignmentFiles.query
            files = files.join(Users, AssignmentFiles.student_id == Users.id)
            files = files.all()
            for file in files:
                file_data = {
                    'id': file.id,
                    'comments': file.comments,
                    'compilation_status': file.compilation_status,
                    'grade': file.grade,
                    'uploaded_date': file.uploaded_date,
                    'test_date': file.test_date,
                    'username': file.student.username,
                    'account_id': file.student.account_id,
                    'name': file.student.names,
                    'fathers_lastname': file.student.fathers_lastname,
                    'mothers_lastname': file.student.mothers_lastname,
                }
                file_list.append(file_data)
            print(file_list)
            return file_list
        except Exception as error:
            print('Error in DataBase.get_assignment_files(): ', error)

    def get_about(self):
        try:
            about = About.query.filter_by(active=True).first()
            if about:
                about_dict = {
                    'description': about.description,
                    'contact_name': about.contact_name,
                    'contact_email': about.contact_email,
                }
                return about_dict
            return {}
        except Exception as error:
            print('Error in DataBase.get_users(): ', error)
            return {}
        

    # def save_file(self, username, file_name, file_path, algorithm):
    #     try:
    #         new_file = Files(username=username, file_name=file_name, file_path =file_path, algorithm=algorithm)
    #         db.session.add(new_file)
    #         db.session.commit()
    #         return 'File added in database successfully'
    #     except Exception as error:
    #         print('Error in DataBase.save_file(): ', error)
    #         return 'Error in DataBase.save_file(): ' + error