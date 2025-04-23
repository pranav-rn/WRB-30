import RPi.GPIO as GPIO
import time
import math
from smbus2 import SMBus
import requests
import json
import threading

# All your original code here...
# ======== LINE FOLLOWER CONFIG ========
# Motor A (Left) pins
IN1 = 20
IN2 = 21
EN1 = 24

# Motor B (Right) pins
IN3 = 16
IN4 = 12
EN2 = 25

# IR sensor pins (array order: [LEFT_FAR, LEFT, CENTER, RIGHT, RIGHT_FAR])
SENSOR_PINS = [5, 6, 13, 19, 26]

# Motor speed values
SPEED = 85  # Base speed for moving
TURN_SPEED = 80  # Speed when turning

# ======== MPU-6050 CONFIG ========
MPU6050_ADDR = 0x68  # I2C address of the MPU-6050
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

# ======== ULTRASONIC SENSOR CONFIG ========
TRIG = 18   # GPIO 18 for Trigger
ECHO = 25   # GPIO 25 for Echo

# ======== BACKEND SERVER CONFIG ========
BACKEND_URL = "http://localhost:5000/api/data"  # Change to your server address
DATA_UPLOAD_INTERVAL = 0.5  # Send data every half second

# ======== GPIO SETUP ========
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Setup motor pins
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(EN1, GPIO.OUT)
GPIO.setup(EN2, GPIO.OUT)

# Setup PWM for motor speed control
left_pwm = GPIO.PWM(EN1, 1000)  # 1000 Hz frequency
right_pwm = GPIO.PWM(EN2, 1000)
left_pwm.start(0)  # Start with 0% duty cycle
right_pwm.start(0)

# Setup ultrasonic sensor pins
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# Setup IR sensor pins as inputs
for pin in SENSOR_PINS:
    GPIO.setup(pin, GPIO.IN)

# Keep all your existing functions...
def read_word(bus, addr, reg):
    """Read a word from the I2C device"""
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg + 1)
    value = (high << 8) + low
    if value >= 0x8000:  # Convert to signed
        value = -((65535 - value) + 1)
    return value

def calibrate_sensor(bus, samples=100):
    """Calibrate the MPU-6050 sensor"""
    print("Calibrating MPU-6050 sensor... Keep the sensor still.")
    ax_offset, ay_offset, az_offset = 0, 0, 0

    # Gather calibration data
    for _ in range(samples):
        accel_x = read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H)
        accel_y = read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H + 2)
        accel_z = read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H + 4)

        accel_scale = 16384.0
        ax_offset += accel_x / accel_scale
        ay_offset += accel_y / accel_scale
        az_offset += accel_z / accel_scale
        time.sleep(0.01)

    # Calculate average offsets
    ax_offset /= samples
    ay_offset /= samples
    az_offset /= samples

    # Determine gravity direction - normalize so gravity reads as 1g in the appropriate axis
    gravity_magnitude = math.sqrt(ax_offset**2 + ay_offset**2 + az_offset**2)
    gravity_normalized = 1.0 / gravity_magnitude if gravity_magnitude > 0 else 1.0

    print(f"Calibration complete! Offsets: X={ax_offset:.4f}g, Y={ay_offset:.4f}g, Z={az_offset:.4f}g")
    print(f"Gravity normalization factor: {gravity_normalized:.4f}")

    return ax_offset, ay_offset, az_offset, gravity_normalized

def get_distance():
    """Measure distance using ultrasonic sensor"""
    # Ensure trigger is low
    GPIO.output(TRIG, False)
    time.sleep(0.05)

    # Send 10us pulse to trigger
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    # Wait for echo to go high (start)
    pulse_start = time.time()
    timeout_start = time.time()
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
        # Add timeout to prevent infinite loop
        if time.time() - timeout_start > 0.1:
            return -1  # Error value

    # Wait for echo to go low (end)
    timeout_start = time.time()
    pulse_end = pulse_start
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()
        # Add timeout to prevent infinite loop
        if time.time() - timeout_start > 0.1:
            return -1  # Error value

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Speed of sound: 34300 cm/s, divide by 2
    distance = round(distance, 2)
    return distance

def read_sensors():
    """Read all IR sensors and return their values as a list."""
    return [GPIO.input(pin) for pin in SENSOR_PINS]

def move_forward():
    """Move the robot forward."""
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    left_pwm.ChangeDutyCycle(SPEED+5)
    right_pwm.ChangeDutyCycle(SPEED)

def turn_left():
    """Turn the robot left."""
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)  # Left motor backward
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)   # Right motor forward
    left_pwm.ChangeDutyCycle(TURN_SPEED)
    right_pwm.ChangeDutyCycle(TURN_SPEED+5)

def turn_right():
    """Turn the robot right."""
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)   # Left motor forward
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)  # Right motor backward
    left_pwm.ChangeDutyCycle(TURN_SPEED)
    right_pwm.ChangeDutyCycle(TURN_SPEED)

def slight_left():
    """Slight adjustment to the left."""
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)   # Left motor forward (slower)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)   # Right motor forward (faster)
    left_pwm.ChangeDutyCycle(70)
    right_pwm.ChangeDutyCycle(TURN_SPEED+5)

def slight_right():
    """Slight adjustment to the right."""
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)   # Left motor forward (faster)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)   # Right motor forward (slower)
    left_pwm.ChangeDutyCycle(TURN_SPEED)
    right_pwm.ChangeDutyCycle(50)

def wander():
    """Move when no line is detected."""
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    left_pwm.ChangeDutyCycle(SPEED - 10)
    right_pwm.ChangeDutyCycle(SPEED - 10)

def stop():
    """Stop the robot."""
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    left_pwm.ChangeDutyCycle(0)
    right_pwm.ChangeDutyCycle(0)

# New function to send data to backend
def send_data_to_backend(data):
    try:
        response = requests.post(BACKEND_URL, json=data, timeout=1)
        if response.status_code == 200:
            print("Data successfully sent to backend")
        else:
            print(f"Failed to send data: {response.status_code}")
    except Exception as e:
        print(f"Error sending data: {e}")

# Modified integrated controller
def integrated_controller():
    try:
        print("Initializing integrated robot controller...")

        # Initialize I2C bus for MPU-6050
        bus = SMBus(1)  # Use I2C bus 1 on Raspberry Pi

        # Wake up MPU-6050 as it starts in sleep mode
        bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)

        # Let the sensor stabilize
        time.sleep(1)

        # Calibrate the MPU-6050 sensor
        ax_offset, ay_offset, az_offset, gravity_factor = calibrate_sensor(bus)

        # Variables for speed calculation
        velocity_x = 0.0
        velocity_y = 0.0
        velocity_z = 0.0
        last_time = time.time()
        last_data_upload = time.time()

        # Motion detection threshold
        motion_threshold = 0.03  # in g

        print("\n" + "="*50)
        print("Starting integrated robot control...")
        print("="*50 + "\n")

        current_action = "Not started"
        current_sensors = [1, 1, 1, 1, 1] # Default no line detected values
        
        while True:
            # ===== LINE FOLLOWER LOGIC =====
            sensors = read_sensors()
            current_sensors = sensors
            
            # Determine movement based on IR sensors
            if sensors[2] == 0:  # Center sensor
                move_forward()
                current_action = "Moving forward"
            elif sensors[3] == 0:  # Left sensor
                slight_left()
                current_action = "Slight left"
            elif sensors[4] == 0:  # Far left sensor
                turn_left()
                current_action = "Turn left"
            elif sensors[1] == 0:  # Right sensor
                slight_right()
                current_action = "Slight right"
            elif sensors[0] == 0:  # Far right sensor
                turn_right()
                current_action = "Turn right"
            else:
                wander()
                current_action = "Wandering"

            # Get current time for acceleration calculation
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time

            # Read accelerometer data
            accel_x = read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H)
            accel_y = read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H + 2)
            accel_z = read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H + 4)

            # Read gyroscope data
            gyro_x = read_word(bus, MPU6050_ADDR, GYRO_XOUT_H)
            gyro_y = read_word(bus, MPU6050_ADDR, GYRO_XOUT_H + 2)
            gyro_z = read_word(bus, MPU6050_ADDR, GYRO_XOUT_H + 4)

            # Convert raw data
            accel_scale = 16384.0  # Sensitivity for +/- 2g
            gyro_scale = 131.0     # Sensitivity for +/- 250deg/s

            ax = accel_x / accel_scale
            ay = accel_y / accel_scale
            az = accel_z / accel_scale

            gx = gyro_x / gyro_scale
            gy = gyro_y / gyro_scale
            gz = gyro_z / gyro_scale

            # Remove calibrated offsets
            ax_calibrated = (ax - ax_offset) * gravity_factor
            ay_calibrated = (ay - ay_offset) * gravity_factor
            az_calibrated = (az - az_offset) * gravity_factor

            # Convert to m/s²
            ax_ms2 = ax_calibrated * 9.81
            ay_ms2 = ay_calibrated * 9.81
            az_ms2 = az_calibrated * 9.81

            # Only count acceleration above threshold as movement
            is_moving = False
            if abs(ax_ms2) > motion_threshold * 9.81:
                velocity_x += ax_ms2 * dt
                is_moving = True

            if abs(ay_ms2) > motion_threshold * 9.81:
                velocity_y += ay_ms2 * dt
                is_moving = True

            if abs(az_ms2) > motion_threshold * 9.81:
                velocity_z += az_ms2 * dt
                is_moving = True

            # Apply decay
            decay_factor = 0.9 if is_moving else 0.5
            velocity_x *= decay_factor
            velocity_y *= decay_factor
            velocity_z *= decay_factor

            # Calculate overall velocity magnitude
            velocity_magnitude = math.sqrt(velocity_x**2 + velocity_y**2 + velocity_z**2)

            # Convert velocity from m/s to km/h
            speed_kmh = velocity_magnitude * 3.6  # 1 m/s = 3.6 km/h

            # Measure distance with ultrasonic sensor
            distance = get_distance()

            # Send data to backend at regular intervals
            if current_time - last_data_upload >= DATA_UPLOAD_INTERVAL:
                data = {
                    'action': current_action,
                    'sensors': current_sensors,
                    'accel': {
                        'x': ax_calibrated,
                        'y': ay_calibrated,
                        'z': az_calibrated
                    },
                    'gyro': {
                        'x': gx,
                        'y': gy,
                        'z': gz
                    },
                    'velocity': {
                        'x': velocity_x,
                        'y': velocity_y,
                        'z': velocity_z
                    },
                    'speed_kmh': speed_kmh,
                    'is_moving': is_moving,
                    'distance': distance
                }
                
                # Send data in a separate thread to avoid blocking
                threading.Thread(target=send_data_to_backend, args=(data,)).start()
                last_data_upload = current_time

            # Print to console for debugging
            print(f"Action: {current_action}, Speed: {speed_kmh:.2f} km/h, Distance: {distance:.2f} cm")
            
            # Small delay for processing
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nProgram terminated by user.")
    finally:
        stop()
        GPIO.cleanup()
        print("GPIO cleanup complete.")

# Main program
if __name__ == "__main__":
    integrated_controller()
