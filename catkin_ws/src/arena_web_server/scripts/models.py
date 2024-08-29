import datetime
from flask_sqlalchemy import SQLAlchemy

db = SQLAlchemy()

class Files(db.Model):
    __tablename__ = 'files'
    id = db.Column(db.Integer, primary_key=True)
    user = db.Column(db.String(255), nullable=False)
    document_name = db.Column(db.String(255), nullable=False)
    algorithm = db.Column(db.String(255), nullable=False)
    status = db.Column(db.Integer, nullable=False)
    path = db.Column(db.String(255), nullable=False)
    upload_date = db.Column(db.DateTime, default=datetime.datetime.now())