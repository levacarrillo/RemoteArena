from .models.ModelsDB import Users, Files, db
from dotenv import load_dotenv
import os
import rospkg

class DataBase:
    def __init__(self, app=None):
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
        self.users = {}
        self.files = []

    def get_users(self):
        with self.app.app_context():
            try:
                users = Users.query.all()
                for user in users:
                    self.users.update({ user.username: user.password })

                return self.users

            except Exception as error:
                print('Error in DataBase.get_users(): ', error)
                return self.users

    def get_file_list(self):
        try:
            files = Files.query.all()
            for file in files:
                file_data = {
                    'id': file.id,
                    'username': file.username,
                    'file_name': file.file_name,
                    'file_path': file.file_path,
                    'algorithm': file.algorithm,
                    'status': file.status,
                    'active': file.active,
                    'upload_date': file.upload_date
                }
                self.files.append(file_data)
            return self.files
        except Exception as error:
            print('Error in DataBase.get_file_list(): ', error)
            return []

    def save_file(self, username, file_name, file_path, algorithm):
        try:
            new_file = Files(username=username, file_name=file_name, file_path =file_path, algorithm=algorithm)
            db.session.add(new_file)
            db.session.commit()
            return 'File added in database successfully'
        except Exception as error:
            print('Error in DataBase.save_file(): ', error)
            return 'Error in DataBase.save_file(): ' + error