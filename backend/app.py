from flask import Flask, request, jsonify, render_template
from flask_socketio import SocketIO, emit
from flask_sqlalchemy import SQLAlchemy
from datetime import datetime
import json
import time
import threading

app = Flask(__name__, static_folder='../frontend/static', template_folder='../frontend/templates')
app.config['SECRET_KEY'] = 'robot-monitoring-secret'
app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///robot_data.db'
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False

socketio = SocketIO(app, cors_allowed_origins="*")
db = SQLAlchemy(app)

# Database Models
class SensorReading(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    timestamp = db.Column(db.DateTime, default=datetime.utcnow)
    
    # Line follower data
    action = db.Column(db.String(50))
    
    # IR Sensors (0=line detected, 1=no line)
    left_far = db.Column(db.Integer)
    left = db.Column(db.Integer)
    center = db.Column(db.Integer)
    right = db.Column(db.Integer)
    right_far = db.Column(db.Integer)
    
    # MPU-6050 data
    accel_x = db.Column(db.Float)
    accel_y = db.Column(db.Float)
    accel_z = db.Column(db.Float)
    gyro_x = db.Column(db.Float)
    gyro_y = db.Column(db.Float)
    gyro_z = db.Column(db.Float)
    
    # Calculated values
    velocity_x = db.Column(db.Float)
    velocity_y = db.Column(db.Float) 
    velocity_z = db.Column(db.Float)
    speed_kmh = db.Column(db.Float)
    is_moving = db.Column(db.Boolean)
    
    # Ultrasonic data
    distance = db.Column(db.Float)

    def to_dict(self):
        return {
            'id': self.id,
            'timestamp': self.timestamp.isoformat(),
            'action': self.action,
            'ir_sensors': [self.left_far, self.left, self.center, self.right, self.right_far],
            'accel': {'x': self.accel_x, 'y': self.accel_y, 'z': self.accel_z},
            'gyro': {'x': self.gyro_x, 'y': self.gyro_y, 'z': self.gyro_z},
            'velocity': {'x': self.velocity_x, 'y': self.velocity_y, 'z': self.velocity_z},
            'speed_kmh': self.speed_kmh,
            'is_moving': self.is_moving,
            'distance': self.distance
        }

# Create database tables
with app.app_context():
    db.create_all()

# Routes
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/data', methods=['POST'])
def receive_data():
    data = request.json
    
    # Create new reading
    reading = SensorReading(
        action=data.get('action', ''),
        left_far=data.get('sensors', [1, 1, 1, 1, 1])[0],
        left=data.get('sensors', [1, 1, 1, 1, 1])[1],
        center=data.get('sensors', [1, 1, 1, 1, 1])[2],
        right=data.get('sensors', [1, 1, 1, 1, 1])[3],
        right_far=data.get('sensors', [1, 1, 1, 1, 1])[4],
        accel_x=data.get('accel', {}).get('x', 0),
        accel_y=data.get('accel', {}).get('y', 0),
        accel_z=data.get('accel', {}).get('z', 0),
        gyro_x=data.get('gyro', {}).get('x', 0),
        gyro_y=data.get('gyro', {}).get('y', 0),
        gyro_z=data.get('gyro', {}).get('z', 0),
        velocity_x=data.get('velocity', {}).get('x', 0),
        velocity_y=data.get('velocity', {}).get('y', 0),
        velocity_z=data.get('velocity', {}).get('z', 0),
        speed_kmh=data.get('speed_kmh', 0),
        is_moving=data.get('is_moving', False),
        distance=data.get('distance', -1)
    )
    
    db.session.add(reading)
    db.session.commit()
    
    # Broadcast data to all connected clients via SocketIO
    socketio.emit('new_data', reading.to_dict())
    
    return jsonify({'status': 'success'})

@app.route('/api/history', methods=['GET'])
def get_history():
    # Get last 100 records
    readings = SensorReading.query.order_by(SensorReading.timestamp.desc()).limit(100).all()
    return jsonify([reading.to_dict() for reading in readings])

# Main entry point
if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)