from flask_sqlalchemy import SQLAlchemy
import datetime

db = SQLAlchemy()

users_groups = db.Table('users_groups',
    db.Column('user_id', db.Integer, db.ForeignKey('users.id'), primary_key=True),
    db.Column('group_id', db.Integer, db.ForeignKey('groups.id'), primary_key=True)
)

class Users(db.Model):
    __tablename__ = 'users'
    id = db.Column(db.Integer, primary_key=True)
    account_id = db.Column(db.String(255), nullable=False)
    username = db.Column(db.String(255), nullable=False)
    password = db.Column(db.String(255), nullable=False)
    names = db.Column(db.String(255), nullable=False)
    fathers_lastname = db.Column(db.String(255), nullable=False)
    mothers_lastname = db.Column(db.String(255), nullable=True)
    email_1 = db.Column(db.String(255), nullable=False)
    email_2 = db.Column(db.String(255), nullable=True)
    active = db.Column(db.Boolean, nullable=False)

    role_id = db.Column(db.Integer, db.ForeignKey('roles.id'), nullable=False)
    assignments = db.relationship('Assignments', backref='professor', lazy=True)
    assignment_files = db.relationship('AssignmentFiles', backref='student', lazy=True)
    groups = db.relationship('Groups', secondary=users_groups, backref=db.backref('users', lazy=True))

    def __repr__(self):
        return f'<Users {self.username}>'

class Roles(db.Model):
    __tablename__ = 'roles'
    id = db.Column(db.Integer, primary_key=True)
    name = db.Column(db.String(255), nullable=False)
    users = db.relationship('Users', backref='roles', lazy=True)

    def __repr__(self):
        return f'<Roles {self.name}>'

class Groups(db.Model):
    __tablename__ = 'groups'
    id = db.Column(db.Integer, primary_key=True)
    number = db.Column(db.String(255), nullable=False)
    class_name = db.Column(db.String(255), nullable=False)
    academic_year = db.Column(db.String(255), nullable=False)
    active = db.Column(db.Boolean, nullable=False)

    def __repr__(self):
        return f'<Groups {self.number}>'


class Assignments(db.Model):
    __tablename__ = 'assignments'
    id = db.Column(db.Integer, primary_key=True)
    title = db.Column(db.String(255), nullable=False)
    description = db.Column(db.String(255), nullable=False)
    document_path = db.Column(db.String(255), nullable=True)
    due_date = db.Column(db.DateTime, nullable=False, default=datetime.datetime.now())
    active = db.Column(db.Boolean, nullable=False)

    professor_id = db.Column(db.Integer, db.ForeignKey('users.id'), nullable=False)    

    def __repr__(self):
        return f'<Assignments {self.title}>'

class AssignmentFiles(db.Model):
    __tablename__ = 'assignment_files'
    id = db.Column(db.Integer, primary_key=True)
    comments = db.Column(db.String(255), nullable=True)
    file_path = db.Column(db.String(255), nullable=False)
    compilation_status = db.Column(db.String(255), nullable=False)
    grade = db.Column(db.Integer, nullable=True)
    uploaded_date = db.Column(db.DateTime, nullable=False, default=datetime.datetime.now())
    test_date = db.Column(db.DateTime, nullable=False, default=datetime.datetime.now())
    active = db.Column(db.Boolean, nullable=False)

    student_id = db.Column(db.Integer, db.ForeignKey('users.id'), nullable=False)
    assignment_video = db.relationship('AssignmentVideos', uselist=False, backref='assignment_files', cascade='all, delete-orphan')

    def __repr__(self):
        return f'<AssignmentFiles {self.file_path}>'

class AssignmentVideos(db.Model):
    __tablename__ = 'assignment_videos'
    id = db.Column(db.Integer, primary_key=True)
    file_path = db.Column(db.String(255), nullable=False)
    recording_date = db.Column(db.DateTime, nullable=False, default=datetime.datetime.now())
    active = db.Column(db.Boolean, nullable=False)
    
    assignment_file_id =db.Column(db.Integer, db.ForeignKey('assignment_files.id'), nullable=False)

    def __repr__(self):
        return f'<AssignmentVideos {self.file_path}>'


class About(db.Model):
    __tablename__ = 'about'
    id = db.Column(db.Integer, primary_key=True)
    description = db.Column(db.String(355), nullable=False)
    contact_name = db.Column(db.String(255), nullable=False)
    contact_email = db.Column(db.String(255), nullable=False)
    picture_path = db.Column(db.String(255), nullable=False)
    active = db.Column(db.Boolean, nullable=False)

    def __repr__(self):
        return f'<About {self.description}>'
