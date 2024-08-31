import datetime
from flask_sqlalchemy import SQLAlchemy

db = SQLAlchemy()

class Files(db.Model):
    __tablename__ = 'files'
    id = db.Column(db.Integer, primary_key=True)
    username = db.Column(db.String(255), nullable=False)
    document_name = db.Column(db.String(255), nullable=False)
    algorithm = db.Column(db.String(255), nullable=False)
    status = db.Column(db.String(255), nullable=False)
    file_path = db.Column(db.String(255), nullable=False)
    upload_date = db.Column(db.DateTime, nullable=False, default=datetime.datetime.now())
    active = db.Column(db.Boolean, nullable=False, default=True)
