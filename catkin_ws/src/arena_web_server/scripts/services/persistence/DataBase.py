from .models.ModelsDB import Users, Files, db

class DataBase:
    def __init__(self, app=None):
        if app is not None:
            self.app = app
            app.secret_key = 'salt'
            app.config['SQLALCHEMY_DATABASE_URI']= 'postgresql+psycopg2://usuario:password@localhost:5432/remote_arena'
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
                print('Error in get_users(): ', error)
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
            print('Error in get_file_list(): ', error)
            return []
