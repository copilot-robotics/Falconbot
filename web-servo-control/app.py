from flask import Flask, render_template, request, jsonify, Response, session, redirect, url_for, send_file
from flask_sqlalchemy import SQLAlchemy
from flask_session import Session
import serial
import serial.tools.list_ports
import threading
import time
import cv2
import hashlib
import random
import string
import math
import os
import wave
import struct
import subprocess
import re
from face_detection import FaceDetector

app = Flask(__name__)

app.config['SECRET_KEY'] = 'saudi-falcone-robot-secret-key-2024'
app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///users.db'
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False
app.config['SESSION_TYPE'] = 'filesystem'
app.config['SESSION_FILE_DIR'] = '/tmp/flask_session'
app.config['SESSION_FILE_THRESHOLD'] = 500

db = SQLAlchemy(app)
Session(app)

ser = None
connected = False
servo_ids = [2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
current_positions = {sid: 2048 for sid in servo_ids}
# Track last commanded (target) positions as fallback when hardware read fails
commanded_positions = {sid: 2048 for sid in servo_ids}
# Indicates whether the last read succeeded (hardware feedback available)
read_feedback_ok = {sid: False for sid in servo_ids}
# Timestamp of the last read attempt per servo, to throttle retries when RX is unavailable
last_read_attempt = {sid: 0.0 for sid in servo_ids}
# When feedback is unavailable, retry reads at this interval (seconds) instead of blocking the bus
READ_RETRY_INTERVAL = 5.0
serial_lock = threading.Lock()

# Sinusoidal motion state: each servo oscillates around a center position
# with its own amplitude (range) and frequency (Hz).
sin_motion_config = {sid: {'center': 2048, 'amplitude': 1000, 'frequency': 0.5, 'enabled': True} for sid in servo_ids}
sin_motion_running = False
sin_motion_thread = None
sin_motion_lock = threading.Lock()

# When True, the background position-reading loop pauses so the serial bus is
# fully available for ID scanning.
scanning_ids = False

INST_SYNC_WRITE = 0x83
INST_READ = 0x02
INST_WRITE = 0x03
INST_PING = 0x01
SMS_STS_ACC = 41
SMS_STS_PRESENT_POSITION_L = 56
SMS_STS_TORQUE_ENABLE = 40

sms_codes = {}

class User(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    username = db.Column(db.String(80), unique=True, nullable=False)
    password_hash = db.Column(db.String(120), nullable=False)
    phone = db.Column(db.String(20), unique=True, nullable=False)
    role = db.Column(db.String(20), default='user')
    is_verified = db.Column(db.Boolean, default=False)
    created_at = db.Column(db.DateTime, default=db.func.now())

    def __repr__(self):
        return f'<User {self.username}>'

def hash_password(password):
    return hashlib.sha256(password.encode()).hexdigest()

def check_password(password, hashed):
    return hashlib.sha256(password.encode()).hexdigest() == hashed

def generate_sms_code():
    return ''.join(random.choices(string.digits, k=6))

def send_sms_code(phone, code):
    print(f"[SMS] Sending verification code {code} to {phone}")
    sms_codes[phone] = {
        'code': code,
        'timestamp': time.time(),
        'attempts': 0
    }

def verify_sms_code(phone, code):
    if phone not in sms_codes:
        return False
    stored = sms_codes[phone]
    if time.time() - stored['timestamp'] > 300:
        del sms_codes[phone]
        return False
    if stored['attempts'] >= 3:
        return False
    if stored['code'] == code:
        del sms_codes[phone]
        return True
    stored['attempts'] += 1
    return False

def login_required(f):
    def decorated_function(*args, **kwargs):
        if 'user_id' not in session:
            return redirect(url_for('login'))
        return f(*args, **kwargs)
    decorated_function.__name__ = f.__name__
    decorated_function.__doc__ = f.__doc__
    return decorated_function

def host_to_scs(data):
    low = data & 0xFF
    high = (data >> 8) & 0xFF
    return low, high

def scs_to_host(low, high):
    return (high << 8) | low

def sync_write_pos_ex(ser, ids, positions, speed, acc):
    with serial_lock:
        id_count = len(ids)
        bytes_per_servo = 7

        mes_len = ((bytes_per_servo + 1) * id_count) + 4

        packet = bytearray()
        packet.append(0xFF)
        packet.append(0xFF)
        packet.append(0xFE)
        packet.append(mes_len)
        packet.append(INST_SYNC_WRITE)
        packet.append(SMS_STS_ACC)
        packet.append(bytes_per_servo)

        checksum = 0xFE + mes_len + INST_SYNC_WRITE + SMS_STS_ACC + bytes_per_servo

        for i in range(id_count):
            pos = positions[i]
            if pos < 0:
                pos = -pos
                pos |= (1 << 15)

            servo_speed = speed[i] if isinstance(speed, list) else speed
            servo_acc = acc[i] if isinstance(acc, list) else acc

            packet.append(ids[i])
            checksum += ids[i]

            packet.append(servo_acc)
            checksum += servo_acc

            pos_low, pos_high = host_to_scs(pos)
            packet.append(pos_low)
            packet.append(pos_high)
            checksum += pos_low
            checksum += pos_high

            packet.append(0)
            packet.append(0)
            checksum += 0
            checksum += 0

            speed_low, speed_high = host_to_scs(servo_speed)
            packet.append(speed_low)
            packet.append(speed_high)
            checksum += speed_low
            checksum += speed_high

        checksum = ~checksum & 0xFF
        packet.append(checksum)

        ser.reset_input_buffer()
        ser.write(packet)
        ser.flush()

def read_pos(ser, servo_id):
    with serial_lock:
        try:
            ser.reset_input_buffer()
            packet = bytearray()
            packet.append(0xFF)
            packet.append(0xFF)
            packet.append(servo_id)
            packet.append(4)
            packet.append(INST_READ)
            packet.append(SMS_STS_PRESENT_POSITION_L)
            packet.append(2)

            checksum = servo_id + 4 + INST_READ + SMS_STS_PRESENT_POSITION_L + 2
            checksum = ~checksum & 0xFF
            packet.append(checksum)

            ser.write(packet)
            ser.flush()
            time.sleep(0.03)

            response = ser.read(8)
            if len(response) >= 7 and response[0] == 0xFF and response[1] == 0xFF:
                pos_low = response[5]
                pos_high = response[6]
                return scs_to_host(pos_low, pos_high)
        except Exception as e:
            print(f"[serial] Error reading position for servo {servo_id}: {e}")
        return -1

def read_all_positions():
    """Read current position from all servos and update current_positions dict.

    When hardware readback is unavailable (e.g. half-duplex RX not connected),
    skip reads until READ_RETRY_INTERVAL elapses to avoid blocking the serial
    bus (and thus delaying move commands). Falls back to the last commanded
    (target) position in the meantime.
    """
    if not connected or not ser or not ser.is_open:
        return
    now = time.time()
    for servo_id in servo_ids:
        # If feedback has been unavailable, only retry at the throttled interval
        # to keep the serial bus free for move commands.
        if not read_feedback_ok[servo_id]:
            if now - last_read_attempt.get(servo_id, 0.0) < READ_RETRY_INTERVAL:
                current_positions[servo_id] = commanded_positions.get(servo_id, current_positions[servo_id])
                continue
        last_read_attempt[servo_id] = now
        pos = read_pos(ser, servo_id)
        if pos >= 0:
            current_positions[servo_id] = pos
            read_feedback_ok[servo_id] = True
        else:
            read_feedback_ok[servo_id] = False
            current_positions[servo_id] = commanded_positions.get(servo_id, current_positions[servo_id])

def disable_torque(ser, servo_id):
    with serial_lock:
        try:
            ser.reset_input_buffer()
            packet = bytearray()
            packet.append(0xFF)
            packet.append(0xFF)
            packet.append(servo_id)
            packet.append(3)
            packet.append(INST_WRITE)
            packet.append(SMS_STS_TORQUE_ENABLE)
            packet.append(0)

            checksum = servo_id + 3 + INST_WRITE + SMS_STS_TORQUE_ENABLE + 0
            checksum = ~checksum & 0xFF
            packet.append(checksum)

            ser.write(packet)
            ser.flush()
        except Exception as e:
            print(f"[serial] Error disabling torque for servo {servo_id}: {e}")

def ping_servo(ser, servo_id, timeout=0.1):
    """Send PING to a single servo ID and return True if it responds.

    Args:
        ser: serial.Serial instance
        servo_id: servo ID to ping (0-253)
        timeout: read timeout in seconds (default 0.1s = 100ms)

    Returns:
        bool: True if servo responds correctly
    """
    with serial_lock:
        try:
            ser.reset_input_buffer()
            packet = bytearray()
            packet.append(0xFF)
            packet.append(0xFF)
            packet.append(servo_id)
            packet.append(2)
            packet.append(INST_PING)

            checksum = ~(servo_id + 2 + INST_PING) & 0xFF
            packet.append(checksum)

            ser.write(packet)
            ser.flush()
            time.sleep(timeout)

            response = ser.read(6)
            if len(response) >= 6 and response[0] == 0xFF and response[1] == 0xFF and response[2] == servo_id:
                return True
        except Exception as e:
            print(f"[serial] Error pinging servo {servo_id}: {e}")
        return False

def scan_servo_ids(ser, id_range=range(0, 31)):
    """Scan a range of servo IDs via PING and return the list of responding IDs.

    Uses 100ms timeout per ping. Returns list of responding IDs.
    """
    found = []
    for sid in id_range:
        if ping_servo(ser, sid, timeout=0.1):
            found.append(sid)
    return found

def update_positions_loop():
    global current_positions
    while True:
        if scanning_ids:
            time.sleep(0.1)
            continue
        if connected and ser and ser.is_open:
            read_all_positions()
            # When all reads fail (RX unavailable), back off so the serial bus
            # stays free for move commands. Polling aggressively only causes
            # lock contention that delays slider-driven moves.
            any_ok = any(read_feedback_ok.values())
            sleep_time = 0.3 if any_ok else READ_RETRY_INTERVAL
        else:
            sleep_time = 1.0
        time.sleep(sleep_time)

position_thread = threading.Thread(target=update_positions_loop, daemon=True)
position_thread.start()

camera_capture = None
camera_lock = threading.Lock()

# ---- Video recording ----
VIDEO_CLIPS_DIR = '/tmp/video_clips'
video_recording = False
video_writer = None
video_record_start = 0.0
video_record_thread = None
video_recording_lock = threading.Lock()

face_detector = None
try:
    face_detector = FaceDetector()
    print(f"[face] FaceDetector ready (method: {face_detector.method})")
except Exception as _e:
    import traceback
    traceback.print_exc()
    print(f"[face] Failed to initialize FaceDetector: {_e}")

def start_camera():
    global camera_capture
    with camera_lock:
        if camera_capture is None:
            try:
                camera_capture = cv2.VideoCapture(0)
                camera_capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                camera_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                camera_capture.set(cv2.CAP_PROP_FPS, 30)
                if not camera_capture.isOpened():
                    camera_capture = None
                    print("Failed to open camera")
                else:
                    print("Camera opened successfully")
            except Exception as e:
                print(f"Failed to start camera: {e}")
                camera_capture = None

def stop_camera():
    global camera_capture
    with camera_lock:
        if camera_capture:
            camera_capture.release()
            camera_capture = None

@app.route('/')
def login():
    if 'user_id' in session:
        return redirect(url_for('dashboard'))
    return render_template('login.html')

@app.route('/dashboard')
@login_required
def dashboard():
    ports = serial.tools.list_ports.comports()
    port_list = [port.device for port in ports]
    user = User.query.get(session['user_id'])
    return render_template('index.html', ports=port_list, user=user)

@app.route('/api/auth/login', methods=['POST'])
def api_login():
    data = request.json
    username = data.get('username')
    password = data.get('password')
    
    if not username or not password:
        return jsonify({'success': False, 'message': 'Username and password are required'})
    
    user = User.query.filter_by(username=username).first()
    
    if not user:
        return jsonify({'success': False, 'message': 'Invalid username or password'})
    
    if not check_password(password, user.password_hash):
        return jsonify({'success': False, 'message': 'Invalid username or password'})
    
    if not user.is_verified:
        return jsonify({'success': False, 'message': 'Account not verified. Please verify your phone number.'})
    
    session['user_id'] = user.id
    session['username'] = user.username
    session['role'] = user.role
    
    return jsonify({'success': True, 'message': 'Login successful', 'redirect': '/dashboard'})

@app.route('/api/auth/register', methods=['POST'])
def api_register():
    data = request.json
    username = data.get('username')
    password = data.get('password')
    phone = data.get('phone')
    
    if not username or not password or not phone:
        return jsonify({'success': False, 'message': 'All fields are required'})
    
    if len(username) < 3 or len(username) > 20:
        return jsonify({'success': False, 'message': 'Username must be 3-20 characters'})
    
    if len(password) < 6:
        return jsonify({'success': False, 'message': 'Password must be at least 6 characters'})
    
    if User.query.filter_by(username=username).first():
        return jsonify({'success': False, 'message': 'Username already exists'})
    
    if User.query.filter_by(phone=phone).first():
        return jsonify({'success': False, 'message': 'Phone number already registered'})
    
    user = User(
        username=username,
        password_hash=hash_password(password),
        phone=phone,
        role='user',
        is_verified=False
    )
    db.session.add(user)
    db.session.commit()
    
    code = generate_sms_code()
    send_sms_code(phone, code)
    
    session['pending_user_id'] = user.id
    
    return jsonify({'success': True, 'message': 'Registration successful. Please verify your phone number.'})

@app.route('/api/auth/send_sms', methods=['POST'])
def api_send_sms():
    data = request.json
    phone = data.get('phone')
    
    if not phone:
        return jsonify({'success': False, 'message': 'Phone number is required'})
    
    user = User.query.filter_by(phone=phone).first()
    if not user:
        return jsonify({'success': False, 'message': 'Phone number not registered'})
    
    code = generate_sms_code()
    send_sms_code(phone, code)
    
    return jsonify({'success': True, 'message': 'Verification code sent'})

@app.route('/api/auth/verify_sms', methods=['POST'])
def api_verify_sms():
    data = request.json
    phone = data.get('phone')
    code = data.get('code')
    
    if not phone or not code:
        return jsonify({'success': False, 'message': 'Phone number and verification code are required'})
    
    if verify_sms_code(phone, code):
        user = User.query.filter_by(phone=phone).first()
        if user:
            user.is_verified = True
            db.session.commit()
            return jsonify({'success': True, 'message': 'Verification successful'})
        return jsonify({'success': False, 'message': 'User not found'})
    else:
        return jsonify({'success': False, 'message': 'Invalid or expired verification code'})

@app.route('/api/auth/logout', methods=['POST'])
def api_logout():
    session.clear()
    return jsonify({'success': True, 'message': 'Logout successful'})

@app.route('/api/auth/check_session')
def api_check_session():
    if 'user_id' in session:
        user = User.query.get(session['user_id'])
        return jsonify({'logged_in': True, 'user': {'username': user.username, 'role': user.role}})
    return jsonify({'logged_in': False})

@app.route('/api/connect', methods=['POST'])
@login_required
def servo_connect():
    global ser, connected
    data = request.json
    port = data.get('port', '/dev/ttyACM0')
    baud = data.get('baud', 1000000)

    try:
        if ser and ser.is_open:
            ser.close()
            time.sleep(0.05)

        ser = serial.Serial(port, baud, timeout=0.15)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        ser.flush()
        time.sleep(0.05)

        connected = True
        actual_baud = baud

        # Quick PING test to verify communication (try all configured servo IDs)
        ping_ok = False
        for sid in servo_ids:
            if ping_servo(ser, sid, timeout=0.15):
                ping_ok = True
                break
        if not ping_ok:
            # Try alternate baud rates
            alt_bauds = [115200, 921600]
            for alt in alt_bauds:
                orig_baud = ser.baudrate
                ser.baudrate = alt
                ser.reset_input_buffer()
                for sid in servo_ids:
                    if ping_servo(ser, sid, timeout=0.15):
                        ping_ok = True
                        break
                if ping_ok:
                    actual_baud = alt
                    break
                ser.baudrate = orig_baud

        if ping_ok:
            return jsonify({
                'success': True,
                'message': f'Connected to {port} at {actual_baud} baud — Servo communication verified ({actual_baud} baud)',
                'port': port,
                'baud': actual_baud,
                'servos_responding': True
            })
        else:
            return jsonify({
                'success': True,
                'message': f'Connected to {port} at {actual_baud} baud — WARNING: No servo response detected. '
                           'Ensure servos have external 6-12V power and are properly wired. '
                           'Run diagnostics for more info.',
                'port': port,
                'baud': actual_baud,
                'servos_responding': False,
                'diagnostic_tip': 'Click "Start Scan" then "Run Serial Diagnostics" to test multiple baud rates.'
            })
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})

@app.route('/api/disconnect', methods=['POST'])
@login_required
def servo_disconnect():
    global ser, connected
    try:
        if ser and ser.is_open:
            ser.close()
        connected = False
        return jsonify({'success': True, 'message': 'Disconnected'})
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})

@app.route('/api/move', methods=['POST'])
@login_required
def move():
    global ser, connected
    if not connected or not ser:
        return jsonify({'success': False, 'message': 'Not connected'})

    data = request.json
    positions = data.get('positions', {})
    speed = data.get('speed', 2400)
    acc = data.get('acceleration', 50)

    servo_positions = []
    for id in servo_ids:
        servo_positions.append(positions.get(str(id), 2048))

    try:
        sync_write_pos_ex(ser, servo_ids, servo_positions, speed, acc)
        # Record the commanded (target) positions as fallback when hardware read fails.
        # Update current_positions immediately so the UI reflects the new position
        # even without servo feedback (half-duplex RX may be unavailable).
        for i, id in enumerate(servo_ids):
            commanded_positions[id] = servo_positions[i]
            # Only fall back to commanded position if hardware read has been failing.
            # If reads succeed, the background loop keeps current_positions in sync.
            if not read_feedback_ok[id]:
                current_positions[id] = servo_positions[i]
        return jsonify({'success': True, 'message': 'Move command sent'})
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})

@app.route('/api/emergency_stop', methods=['POST'])
@login_required
def emergency_stop():
    global ser, connected
    if not connected or not ser:
        return jsonify({'success': False, 'message': 'Not connected'})
    
    try:
        # Stop any running sin motion / emotion action first
        stop_sin_motion()
        stop_emotion_action()
        for servo_id in servo_ids:
            disable_torque(ser, servo_id)
        return jsonify({'success': True, 'message': 'Emergency stop triggered - torque disabled on all servos'})
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})

def stop_sin_motion():
    """Signal the sin motion thread to stop (non-blocking)."""
    global sin_motion_running
    sin_motion_running = False

def sin_motion_loop():
    """Background thread that drives each servo along a sine wave.

    Each servo oscillates around its center position with its own
    amplitude (range) and frequency (Hz). The loop runs at ~20Hz so
    motion is smooth. It reads the config under sin_motion_lock so the
    frontend can update parameters on the fly without restarting.
    """
    global ser, connected, sin_motion_running
    start_time = time.time()
    while sin_motion_running:
        if not connected or not ser or not ser.is_open:
            sin_motion_running = False
            break
        t = time.time() - start_time
        positions = []
        with sin_motion_lock:
            cfg_snapshot = {sid: dict(sin_motion_config[sid]) for sid in servo_ids}
        for sid in servo_ids:
            c = cfg_snapshot[sid]
            if c.get('enabled', True):
                pos = c['center'] + c['amplitude'] * math.sin(2 * math.pi * c['frequency'] * t)
                pos = int(round(max(0, min(4095, pos))))
            else:
                # Disabled servo: hold its last commanded position (no movement)
                pos = commanded_positions.get(sid, c['center'])
            positions.append(pos)
        try:
            speed = 0      # 0 = max speed so the servo tracks the sine closely
            acc = 0
            sync_write_pos_ex(ser, servo_ids, positions, speed, acc)
            for i, sid in enumerate(servo_ids):
                commanded_positions[sid] = positions[i]
                if not read_feedback_ok[sid]:
                    current_positions[sid] = positions[i]
        except Exception as e:
            print(f"Sin motion write error: {e}")
        time.sleep(0.05)  # 20Hz update

@app.route('/api/sin_motion/start', methods=['POST'])
@login_required
def sin_motion_start():
    global sin_motion_running, sin_motion_thread
    if not connected or not ser:
        return jsonify({'success': False, 'message': 'Not connected'})

    data = request.json or {}
    with sin_motion_lock:
        for sid in servo_ids:
            key = str(sid)
            if key in data:
                cfg = data[key]
                if 'enabled' in cfg:
                    sin_motion_config[sid]['enabled'] = bool(cfg['enabled'])
                if 'amplitude' in cfg:
                    sin_motion_config[sid]['amplitude'] = max(0, min(2048, int(cfg['amplitude'])))
                if 'frequency' in cfg:
                    sin_motion_config[sid]['frequency'] = max(0.01, min(10.0, float(cfg['frequency'])))
                if 'center' in cfg:
                    sin_motion_config[sid]['center'] = max(0, min(4095, int(cfg['center'])))

    if sin_motion_running and sin_motion_thread and sin_motion_thread.is_alive():
        return jsonify({'success': True, 'message': 'Sin motion parameters updated (already running)'})

    sin_motion_running = True
    sin_motion_thread = threading.Thread(target=sin_motion_loop, daemon=True)
    sin_motion_thread.start()
    return jsonify({'success': True, 'message': 'Sin motion started'})

@app.route('/api/sin_motion/stop', methods=['POST'])
@login_required
def sin_motion_stop():
    stop_sin_motion()
    return jsonify({'success': True, 'message': 'Sin motion stopped'})

@app.route('/api/sin_motion/status', methods=['GET'])
@login_required
def sin_motion_status():
    return jsonify({
        'running': sin_motion_running,
        'config': {str(sid): sin_motion_config[sid] for sid in servo_ids}
    })

# ==================== Emotion Actions (行为动作) ====================
# Three emotion actions, each paired with a falcon call:
#   happy        (开心): full-body motion — all joints move
#   calm         (冷静): only head (yaw/pitch) and mouth move
#   enthusiastic (热情): only the legs move
#
# A background thread computes joint positions at 20 Hz. When drive_robot=True
# the frame is written to the real servos; otherwise it is animation-only.
# The frontend panel polls /api/emotion/status and renders the joint bars in
# both modes, so the animation always follows the action.

JOINT_HOME = 2048

# Per-joint oscillation parameters:
#   amp    – swing amplitude around JOINT_HOME (position units)
#   freq   – oscillation frequency (Hz)
#   phase  – phase offset (radians)
#   shape  – 'abs' = unipolar open/close envelope (mouth), default symmetric sine
EMOTION_ACTIONS = {
    'happy': {
        'name': 'Happy 开心',
        'description': 'Full-body joyful motion (all joints) 全身动作',
        'joints': {
            2:  {'amp': 800, 'freq': 2.0, 'phase': 0.0},    # tail wag
            3:  {'amp': 600, 'freq': 0.9, 'phase': 0.0},    # R hip bounce
            4:  {'amp': 500, 'freq': 0.9, 'phase': 2.1},    # R knee
            5:  {'amp': 400, 'freq': 0.9, 'phase': 4.2},    # R ankle
            6:  {'amp': 600, 'freq': 0.9, 'phase': 3.1},    # L hip (counter-phase)
            7:  {'amp': 500, 'freq': 0.9, 'phase': 5.2},    # L knee
            8:  {'amp': 400, 'freq': 0.9, 'phase': 1.0},    # L ankle
            9:  {'amp': 700, 'freq': 0.6, 'phase': 1.5},    # head yaw sweep
            10: {'amp': 350, 'freq': 1.2, 'phase': 0.7},    # head pitch bob
            11: {'amp': 800, 'freq': 2.4, 'phase': 0.0, 'shape': 'abs'},  # chirping mouth
        },
    },
    'calm': {
        'name': 'Calm 冷静',
        'description': 'Head and mouth only 头部和嘴巴',
        'joints': {
            9:  {'amp': 600, 'freq': 0.25, 'phase': 0.0},   # slow looking around
            10: {'amp': 250, 'freq': 0.35, 'phase': 1.2},   # gentle nodding
            11: {'amp': 700, 'freq': 0.5, 'phase': 0.0, 'shape': 'abs'},  # slow chewing
        },
    },
    'enthusiastic': {
        'name': 'Enthusiastic 热情',
        'description': 'Legs only 仅腿部',
        'joints': {
            3:  {'amp': 800, 'freq': 1.4, 'phase': 0.0},    # R hip march
            4:  {'amp': 650, 'freq': 1.4, 'phase': 1.6},    # R knee
            5:  {'amp': 500, 'freq': 1.4, 'phase': 3.1},    # R ankle
            6:  {'amp': 800, 'freq': 1.4, 'phase': 3.1},    # L hip (alternating step)
            7:  {'amp': 650, 'freq': 1.4, 'phase': 4.7},    # L knee
            8:  {'amp': 500, 'freq': 1.4, 'phase': 6.2},    # L ankle
        },
    },
    # IDLE waiting state: tiny random micro-movements across all joints that
    # simulate a living animal (breathing, idle head turns, tail flicks, weight
    # shifts on the legs, occasional mouth twitches). Uses a dedicated random
    # walk generator instead of fixed sine parameters.
    'idle': {
        'name': 'Idle 待机',
        'description': 'Alive idle motion 活体待机微动',
        'joints': {
            2:  {'amp': 120, 'base_freq': 0.4},   # tail
            3:  {'amp': 80,  'base_freq': 0.25},  # R hip
            4:  {'amp': 70,  'base_freq': 0.25},  # R knee
            5:  {'amp': 60,  'base_freq': 0.25},  # R ankle
            6:  {'amp': 80,  'base_freq': 0.25},  # L hip
            7:  {'amp': 70,  'base_freq': 0.25},  # L knee
            8:  {'amp': 60,  'base_freq': 0.25},  # L ankle
            9:  {'amp': 90,  'base_freq': 0.18},  # head yaw
            10: {'amp': 60,  'base_freq': 0.22},  # head pitch
            11: {'amp': 70,  'base_freq': 0.3},   # mouth
        },
    },
}

emotion_running = False
emotion_action = None
emotion_drive_robot = False
emotion_positions = {sid: JOINT_HOME for sid in servo_ids}
emotion_thread = None
emotion_sound_thread = None
emotion_lock = threading.Lock()

# Mapping from emotion action to the falcon sound that should loop while the
# action is running.  idle has no looping call (standby is silent).
EMOTION_SOUNDS = {
    'happy':         'cackle',   # excited chattering
    'calm':          'whistle',  # gentle whistle
    'enthusiastic':  'cry',      # loud cry
    'idle':          None,       # silent standby
}

def _emotion_joint_value(cfg, t):
    """Compute one joint's position at time t (seconds) from its config."""
    amp = cfg.get('amp', 0)
    freq = cfg.get('freq', 1.0)
    phase = cfg.get('phase', 0.0)
    if cfg.get('shape') == 'abs':
        # Unipolar 0..1 envelope: opens from home and closes back (mouth)
        return JOINT_HOME + abs(math.sin(math.pi * freq * t + phase)) * amp
    return JOINT_HOME + math.sin(2 * math.pi * freq * t + phase) * amp

class IdleMotionGenerator:
    """Generates small-amplitude, organic-looking random micro-movements.

    Each joint oscillates around JOINT_HOME with a slowly drifting phase
    (randomized frequency) bounded by its configured amplitude. Occasional
    random twitch impulses are injected into expressive joints (tail, head,
    mouth) to mimic a living animal fidgeting. Motion is deliberately subtle.
    """

    # Joints that occasionally get a quick twitch impulse.
    TWITCH_JOINTS = {2, 9, 10, 11}

    def __init__(self, joints_cfg):
        self.joints = joints_cfg
        # Per-joint phase (radians) and a small frequency jitter so every
        # joint's motion is asynchronous and never repeats in lock-step.
        self.phase = {sid: random.uniform(0, 2 * math.pi) for sid in joints_cfg}
        self.freq = {sid: cfg['base_freq'] * random.uniform(0.7, 1.3)
                     for sid, cfg in joints_cfg.items()}
        # Twitch state: impulse value decays exponentially each frame.
        self.twitch = {sid: 0.0 for sid in self.TWITCH_JOINTS}
        self.next_twitch_in = random.uniform(2.0, 5.0)

    def step(self, dt):
        """Advance one frame and return {sid: position}."""
        frame = {}
        # Slowly vary each joint's frequency for organic drift.
        for sid, cfg in self.joints.items():
            self.freq[sid] += random.uniform(-0.02, 0.02) * cfg['base_freq']
            self.freq[sid] = max(0.05, self.freq[sid])
            self.phase[sid] += 2 * math.pi * self.freq[sid] * dt
            pos = JOINT_HOME + math.sin(self.phase[sid]) * cfg['amp']
            if sid in self.TWITCH_JOINTS:
                pos += self.twitch[sid]
                self.twitch[sid] *= math.exp(-dt * 6.0)  # ~6s decay time constant
            frame[sid] = int(round(max(0, min(4095, pos))))

        # Trigger occasional twitches on one expressive joint.
        self.next_twitch_in -= dt
        if self.next_twitch_in <= 0:
            sid = random.choice(list(self.TWITCH_JOINTS))
            # Quick impulse, magnitude scaled to the joint's idle amplitude.
            self.twitch[sid] = random.uniform(0.6, 1.2) * self.joints[sid]['amp']
            self.next_twitch_in = random.uniform(2.5, 6.0)

        return frame

def _emotion_sound_loop(animal):
    """Background thread that loops the animal sound while emotion_running.

    Plays the WAV file with aplay, waits for it to finish, then plays again
    with a short randomised gap (0.3–1.0 s) to sound organic.  Stops as soon
    as emotion_running becomes False.
    """
    import random as _rng
    wav_path = os.path.join(SOUNDS_DIR, f'{animal}.wav')
    if not os.path.exists(wav_path):
        return
    while emotion_running:
        # Stop any previous playback (from this loop or elsewhere)
        stop_audio_internal()
        try:
            proc = subprocess.Popen(['aplay', '-q', '-D', SPEAKER_DEVICE, wav_path])
            with audio_lock:
                audio_state['operation'] = 'play'
                audio_state['animal'] = animal
                audio_state['process'] = proc
                audio_state['started_at'] = time.time()
                audio_state['duration'] = float(ANIMAL_SOUNDS.get(animal, {}).get('duration', 2.0))
        except Exception:
            break
        # Wait for playback to finish or emotion to stop, whichever is first
        while emotion_running and proc.poll() is None:
            time.sleep(0.1)
        if not emotion_running:
            break
        # Short randomised gap between calls
        gap = _rng.uniform(0.3, 1.0)
        t0 = time.time()
        while emotion_running and (time.time() - t0) < gap:
            time.sleep(0.1)
    # Clean up audio state when the loop exits
    stop_audio_internal()

def stop_emotion_action():
    """Signal the emotion thread (and sound loop) to stop and wait."""
    global emotion_running, emotion_thread, emotion_sound_thread
    emotion_running = False
    if emotion_thread and emotion_thread.is_alive():
        emotion_thread.join(timeout=1.5)
    emotion_thread = None
    if emotion_sound_thread and emotion_sound_thread.is_alive():
        emotion_sound_thread.join(timeout=2.0)
    emotion_sound_thread = None
    # Make sure no lingering aplay process survives
    stop_audio_internal()

def _emotion_loop():
    """Background thread driving the currently selected emotion action.

    Computes a full 10-joint frame at ~20 Hz. When drive_robot=True the frame
    is sync-written to the servos (max speed so it tracks the waveform); the
    frame is always published to emotion_positions for the frontend animation.
    """
    global emotion_running
    action = emotion_action
    drive_robot = emotion_drive_robot
    joints = EMOTION_ACTIONS[action]['joints']
    start_time = time.time()
    last_t = start_time
    # The idle action uses a dedicated organic random-walk generator instead of
    # fixed sine parameters, so each run looks different and never repeats.
    idle_gen = IdleMotionGenerator(joints) if action == 'idle' else None
    while emotion_running:
        if drive_robot and (not connected or not ser or not ser.is_open):
            emotion_running = False
            break
        now = time.time()
        t = now - start_time
        dt = max(0.001, now - last_t)
        last_t = now
        frame = {}
        if idle_gen is not None:
            frame = idle_gen.step(dt)
            for sid in servo_ids:
                if sid not in frame:
                    frame[sid] = JOINT_HOME
        else:
            for sid in servo_ids:
                cfg = joints.get(sid)
                pos = _emotion_joint_value(cfg, t) if cfg else JOINT_HOME
                frame[sid] = int(round(max(0, min(4095, pos))))
        with emotion_lock:
            for sid in servo_ids:
                emotion_positions[sid] = frame[sid]
        if drive_robot:
            try:
                pos_list = [frame[sid] for sid in servo_ids]
                sync_write_pos_ex(ser, servo_ids, pos_list, 0, 0)
                for i, sid in enumerate(servo_ids):
                    commanded_positions[sid] = pos_list[i]
                    if not read_feedback_ok[sid]:
                        current_positions[sid] = pos_list[i]
            except Exception as e:
                print(f"Emotion motion write error: {e}")
        time.sleep(0.05)  # 20 Hz update

    # Action finished: if we were driving the robot and it is still connected,
    # smoothly return all joints to the home position.
    if drive_robot and connected and ser and ser.is_open:
        try:
            home = [JOINT_HOME] * len(servo_ids)
            sync_write_pos_ex(ser, servo_ids, home, 1400, 30)
            for sid in servo_ids:
                commanded_positions[sid] = JOINT_HOME
                if not read_feedback_ok[sid]:
                    current_positions[sid] = JOINT_HOME
        except Exception as e:
            print(f"Emotion home return error: {e}")
    with emotion_lock:
        for sid in servo_ids:
            emotion_positions[sid] = JOINT_HOME

@app.route('/api/emotion/start', methods=['POST'])
@login_required
def emotion_start():
    global emotion_running, emotion_action, emotion_drive_robot
    global emotion_thread, emotion_sound_thread
    data = request.json or {}
    action = data.get('action')
    if action not in EMOTION_ACTIONS:
        return jsonify({'success': False, 'message': 'Unknown emotion action'})
    drive_robot = bool(data.get('drive_robot', False))
    if drive_robot and (not connected or not ser):
        return jsonify({'success': False, 'message': 'Not connected — cannot drive robot directly'})

    stop_emotion_action()
    # Keep the serial bus dedicated to this motion
    stop_sin_motion()
    with emotion_lock:
        for sid in servo_ids:
            emotion_positions[sid] = JOINT_HOME

    emotion_action = action
    emotion_drive_robot = drive_robot
    emotion_running = True
    emotion_thread = threading.Thread(target=_emotion_loop, daemon=True)
    emotion_thread.start()

    # Start looping the corresponding animal sound (if any)
    sound_animal = EMOTION_SOUNDS.get(action)
    if sound_animal:
        ensure_animal_sounds()
        emotion_sound_thread = threading.Thread(
            target=_emotion_sound_loop, args=(sound_animal,), daemon=True)
        emotion_sound_thread.start()

    mode = 'direct drive + animation' if drive_robot else 'animation only'
    sound_msg = f' + 🔊 {ANIMAL_SOUNDS[sound_animal]["name"]} loop' if sound_animal else ' (silent)'
    return jsonify({'success': True, 'message': f"{EMOTION_ACTIONS[action]['name']} started ({mode}{sound_msg})"})

@app.route('/api/emotion/stop', methods=['POST'])
@login_required
def emotion_stop():
    was_driving = emotion_drive_robot
    stop_emotion_action()
    msg = 'Emotion action stopped'
    if was_driving and connected:
        msg += ' — servos returning home'
    return jsonify({'success': True, 'message': msg})

@app.route('/api/emotion/status', methods=['GET'])
@login_required
def emotion_status():
    with emotion_lock:
        positions = dict(emotion_positions)
    sound_animal = EMOTION_SOUNDS.get(emotion_action) if emotion_action else None
    return jsonify({
        'running': emotion_running,
        'action': emotion_action,
        'drive_robot': emotion_drive_robot,
        'connected': connected,
        'sound': sound_animal,
        'sound_name': ANIMAL_SOUNDS.get(sound_animal, {}).get('name') if sound_animal else None,
        'positions': {str(sid): positions[sid] for sid in servo_ids},
    })

@app.route('/api/status')
@login_required
def status():
    return jsonify({
        'connected': connected,
        'servos': servo_ids,
        'current_positions': current_positions
    })

@app.route('/api/read_positions', methods=['GET'])
@login_required
def read_positions():
    read_all_positions()
    return jsonify({
        'success': True,
        'connected': connected,
        'current_positions': current_positions,
        'read_feedback_ok': read_feedback_ok
    })

@app.route('/api/scan_ids', methods=['POST'])
@login_required
def scan_ids():
    """Scan servo IDs via PING and return the responding ones.

    Tries multiple strategies for robustness:
    1. Ping with current baud rate
    2. If no results, re-try at 115200 baud (common alternate rate)
    """
    global scanning_ids
    if not connected or not ser:
        return jsonify({'success': False, 'message': 'Not connected'})

    stop_sin_motion()
    scanning_ids = True
    time.sleep(0.2)

    try:
        current_baud = ser.baudrate
        found_ids = scan_servo_ids(ser, range(1, 31))

        if not found_ids and current_baud != 115200:
            print(f"[serial] No servos found at {current_baud} baud, trying 115200...")
            original_baud = current_baud
            ser.baudrate = 115200
            ser.reset_input_buffer()
            found_ids = scan_servo_ids(ser, range(1, 31))
            ser.baudrate = original_baud

        if not found_ids:
            return jsonify({
                'success': False,
                'message': 'No servos detected. Check power/wiring. '
                           f'Current baud: {current_baud}. '
                           'Ensure servos have external power (6-12V) and are connected to the correct port.',
                'found_ids': [],
                'count': 0
            })

        return jsonify({
            'success': True,
            'found_ids': found_ids,
            'count': len(found_ids),
            'baud': current_baud
        })
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})
    finally:
        scanning_ids = False

@app.route('/api/ports')
@login_required
def get_ports():
    ports = serial.tools.list_ports.comports()
    port_list = [{'device': port.device, 'description': port.description} for port in ports]
    return jsonify(port_list)

@app.route('/api/serial/diagnose', methods=['POST'])
@login_required
def serial_diagnose():
    """Run comprehensive serial diagnostics.

    Tests PING at multiple baud rates and provides detailed results.
    Also checks for common hardware issues.

    Args (JSON):
        port: serial port to use (default: currently connected port)
        servo_id: servo ID to test (default: first configured servo ID)
    """
    data = request.json or {}
    test_port = data.get('port')
    test_id = int(data.get('servo_id', servo_ids[0]))

    if test_port:
        result = {'port': test_port, 'tests': [], 'hardware_checks': {}}

        # Test at all standard baud rates used by SCS servos
        baud_rates = [1000000, 115200, 921600, 230400, 500000, 57600, 38400]
        for baud in baud_rates:
            try:
                test_ser = serial.Serial(test_port, baud, timeout=0.15)
                test_ser.reset_input_buffer()
                test_ser.reset_output_buffer()
                time.sleep(0.05)
                success = ping_servo(test_ser, test_id, timeout=0.15)
                test_ser.close()
                result['tests'].append({
                    'baud': baud,
                    'ping_ok': success
                })
            except Exception as e:
                result['tests'].append({
                    'baud': baud,
                    'error': str(e)
                })

        # Check if any baud rate worked
        result['overall_success'] = any(t.get('ping_ok', False) for t in result['tests'])

        if result['overall_success']:
            working_baud = next(t['baud'] for t in result['tests'] if t.get('ping_ok'))
            result['diagnosis'] = (
                f"✓ Servo ID {test_id} responds at {working_baud} baud on {test_port}. "
                f"Set the connection baud rate to {working_baud} for reliable communication."
            )
            result['recommendation'] = f"Update connection baud rate to {working_baud}"
        else:
            result['diagnosis'] = (
                f"✗ No response from servo ID {test_id} at any baud rate on {test_port}. "
                "The serial port is functioning (can open/close at all rates) but no valid SCS responses received.\n\n"
                "POSSIBLE CAUSES (check in order):\n"
                "1. ⚡ POWER: SCS servos require EXTERNAL 6-12V power supply. USB 5V is NOT sufficient.\n"
                "2. 🔌 WIRING: Ensure 4 connections between communication board and servos:\n"
                "   - VCC → Servo VCC (external 6-12V)\n"
                "   - GND → Servo GND\n"
                "   - TX (board) → RX (servo bus)\n"
                "   - RX (board) → TX (servo bus)\n"
                "3. 📍 WRONG PORT: Try the other available serial port.\n"
                "4. 🔧 HALF-DUPLEX: Some setups require a direction control signal (GPIO/RTS) to switch between TX and RX.\n"
                "5. 📡 WRONG BAUD: Servo might be configured for a different baud rate (try reprogramming).\n"
                "6. 🔋 SERVO OFF: Servo power switch may be off."
            )
            result['recommendation'] = 'Check hardware connections and power supply'

        # Hardware checks
        try:
            test_ser = serial.Serial(test_port, 115200, timeout=0.1)
            buffer_level = test_ser.in_waiting
            test_ser.close()
            result['hardware_checks'] = {
                'port_openable': True,
                'input_buffer_empty_on_open': buffer_level == 0,
                'port_device': test_port,
                'note': 'Port can be opened at multiple baud rates. If no servos respond, hardware issue is likely.'
            }
        except Exception as e:
            result['hardware_checks'] = {
                'port_openable': False,
                'error': str(e)
            }

        return jsonify(result)
    else:
        if not connected or not ser:
            return jsonify({'success': False, 'message': 'Not connected and no port specified'})

        result = {
            'port': ser.port,
            'current_baud': ser.baudrate,
            'servo_id': test_id,
            'ping_test': ping_servo(ser, test_id, timeout=0.15),
            'input_buffer_before': ser.in_waiting
        }
        return jsonify(result)

def gen_frames():
    global camera_capture
    start_camera()
    if camera_capture is None:
        yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + b'Camera not available' + b'\r\n'
        return
    
    while camera_capture:
        success, frame = camera_capture.read()
        if not success:
            break

        if face_detector and face_detector.enabled:
            frame = face_detector.detect_and_draw(frame)

        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ret:
            continue
        
        frame_bytes = buffer.tobytes()
        yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n'
    
    stop_camera()

@app.route('/video_feed')
@login_required
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/camera/start', methods=['POST'])
@login_required
def camera_start():
    start_camera()
    return jsonify({'success': True, 'message': 'Camera started'})

@app.route('/api/camera/stop', methods=['POST'])
@login_required
def camera_stop():
    stop_camera()
    return jsonify({'success': True, 'message': 'Camera stopped'})

def _video_record_loop(out_path, max_duration=300):
    """Background thread that writes frames from camera_capture into a video file.

    Runs at ~20 FPS. Stops when video_recording is set to False or max_duration
    seconds have elapsed. Always releases the writer on exit.
    """
    global video_recording, video_writer
    start_time = time.time()
    try:
        while video_recording:
            if camera_capture is None or not camera_capture.isOpened():
                break
            with camera_lock:
                ret, frame = camera_capture.read()
            if not ret:
                break
            if face_detector and face_detector.enabled:
                frame = face_detector.detect_and_draw(frame)
            if video_writer and video_writer.isOpened():
                video_writer.write(frame)
            elapsed = time.time() - start_time
            if elapsed >= max_duration:
                break
            time.sleep(0.05)
    except Exception as e:
        print(f"Video record loop error: {e}")
    finally:
        with video_recording_lock:
            if video_writer and video_writer.isOpened():
                video_writer.release()
                print(f"Video clip saved: {out_path}")
            video_writer = None
            video_recording = False

@app.route('/api/camera/record/start', methods=['POST'])
@login_required
def camera_record_start():
    global video_recording, video_writer, video_record_start, video_record_thread
    data = request.json or {}
    duration = max(1, min(300, int(data.get('duration', 30))))

    if video_recording:
        return jsonify({'success': False, 'message': 'Already recording'})

    if camera_capture is None or not camera_capture.isOpened():
        return jsonify({'success': False, 'message': 'Camera not active. Start camera first.'})

    os.makedirs(VIDEO_CLIPS_DIR, exist_ok=True)
    filename = f"clip_{int(time.time())}.mp4"
    out_path = os.path.join(VIDEO_CLIPS_DIR, filename)

    fps = 20.0
    width = int(camera_capture.get(cv2.CAP_PROP_FRAME_WIDTH) or 640)
    height = int(camera_capture.get(cv2.CAP_PROP_FRAME_HEIGHT) or 480)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_writer = cv2.VideoWriter(out_path, fourcc, fps, (width, height))

    video_recording = True
    video_record_start = time.time()
    video_record_thread = threading.Thread(
        target=_video_record_loop, args=(out_path, duration), daemon=True
    )
    video_record_thread.start()

    return jsonify({
        'success': True,
        'message': f'Recording started ({duration}s)',
        'filename': filename
    })

@app.route('/api/camera/record/stop', methods=['POST'])
@login_required
def camera_record_stop():
    global video_recording
    video_recording = False
    if video_record_thread and video_record_thread.is_alive():
        video_record_thread.join(timeout=2.0)
    return jsonify({'success': True, 'message': 'Recording stopped'})

@app.route('/api/camera/record/status')
@login_required
def camera_record_status():
    duration = round(time.time() - video_record_start, 1) if video_recording else 0
    return jsonify({
        'recording': video_recording,
        'elapsed': duration,
        'filename': os.path.basename(getattr(video_writer, 'filename', '')) if video_writer else None
    })

@app.route('/api/camera/clips')
@login_required
def camera_clips():
    clips = []
    if os.path.isdir(VIDEO_CLIPS_DIR):
        for fname in sorted(os.listdir(VIDEO_CLIPS_DIR), reverse=True):
            fpath = os.path.join(VIDEO_CLIPS_DIR, fname)
            if os.path.isfile(fpath) and not fname.startswith('.'):
                stat = os.stat(fpath)
                clips.append({
                    'filename': fname,
                    'size': stat.st_size,
                    'modified': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(stat.st_mtime))
                })
    return jsonify({'success': True, 'clips': clips})

@app.route('/api/camera/clip/<filename>')
@login_required
def camera_clip_download(filename):
    safe = os.path.basename(filename)
    path = os.path.join(VIDEO_CLIPS_DIR, safe)
    if not os.path.exists(path):
        return jsonify({'success': False, 'message': 'Not found'}), 404
    return send_file(path, mimetype='video/mp4', as_attachment=False, download_name=safe)

@app.route('/api/camera/clip/<filename>/delete', methods=['POST'])
@login_required
def camera_clip_delete(filename):
    safe = os.path.basename(filename)
    path = os.path.join(VIDEO_CLIPS_DIR, safe)
    if os.path.exists(path):
        os.remove(path)
        return jsonify({'success': True, 'message': f'Deleted {safe}'})
    return jsonify({'success': False, 'message': 'Not found'}), 404

@app.route('/api/face/detection', methods=['POST'])
@login_required
def face_detection_set():
    data = request.json or {}
    enabled = bool(data.get('enabled', False))
    if face_detector is None:
        return jsonify({'success': False, 'message': 'Face detector not available'}), 500
    if enabled:
        face_detector.enable()
    else:
        face_detector.disable()
    return jsonify({
        'success': True,
        'enabled': face_detector.enabled,
        'method': face_detector.method if face_detector else 'unavailable'
    })

@app.route('/api/face/detection/status')
@login_required
def face_detection_status():
    if face_detector is None:
        return jsonify({'available': False, 'enabled': False, 'method': 'unavailable', 'faces': []})
    faces = face_detector.get_faces()
    return jsonify({
        'available': True,
        'enabled': face_detector.enabled,
        'method': face_detector.method,
        'faces': [
            {'x': x, 'y': y, 'w': w, 'h': h}
            for (x, y, w, h) in faces
        ]
    })

llm_config = {
    'voice': 'male',
    'model': 'gpt-4',
    'prompt': 'You are a helpful robot assistant. You control a robot arm with 3 servo motors. Respond to user commands about controlling the robot.'
}

@app.route('/api/system_status')
@login_required
def system_status():
    import datetime
    
    now = datetime.datetime.now()
    system_time = now.strftime('%H:%M:%S')
    system_date = now.strftime('%Y-%m-%d %A')
    
    battery_percent = 0
    try:
        with open('/sys/class/power_supply/BAT0/capacity', 'r') as f:
            battery_percent = int(f.read().strip())
    except:
        battery_percent = -1
    
    temperature = 0
    try:
        with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
            temperature = int(f.read().strip()) / 1000
    except:
        temperature = -1
    
    motor_status = []
    for servo_id in servo_ids:
        motor_status.append({
            'id': servo_id,
            'position': current_positions[servo_id],
            'connected': connected
        })
    
    return jsonify({
        'time': system_time,
        'date': system_date,
        'battery': {
            'percent': battery_percent,
            'status': 'charging' if battery_percent == -1 else ('full' if battery_percent >= 95 else 'normal')
        },
        'temperature': {
            'value': temperature,
            'unit': 'C'
        },
        'motors': motor_status
    })

@app.route('/api/llm/config', methods=['GET'])
@login_required
def get_llm_config():
    return jsonify({'success': True, 'config': llm_config})

@app.route('/api/llm/config', methods=['POST'])
@login_required
def update_llm_config():
    global llm_config
    data = request.json
    
    if 'voice' in data:
        llm_config['voice'] = data['voice']
    if 'model' in data:
        llm_config['model'] = data['model']
    if 'prompt' in data:
        llm_config['prompt'] = data['prompt']
    
    return jsonify({'success': True, 'message': 'LLM configuration updated', 'config': llm_config})

# ==================== Audio Test Panel ====================
# Falcon (鹰隼) sound clips are stored as real WAV files under static/sounds/.
# Microphone recordings are saved to a separate temp directory.
SOUNDS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'static', 'sounds')
RECORDING_DIR = '/tmp/animal_sounds'
# Keep AUDIO_DIR as alias for recording paths (used by /api/audio/record, /api/audio/wav)
AUDIO_DIR = RECORDING_DIR
ANIMAL_SOUNDS = {
    'cry':     {'name': 'Falcon Cry',     'emoji': '🦅', 'description': '鹰隼鸣叫 ~5秒',  'duration': 5.0},
    'cackle':  {'name': 'Falcon Cackle',  'emoji': '🦅', 'description': '鹰隼咯咯声 ~3秒', 'duration': 3.0},
    'whistle': {'name': 'Falcon Whistle', 'emoji': '🦅', 'description': '鹰隼哨声 ~8秒',  'duration': 8.0},
}
audio_lock = threading.Lock()
audio_state = {
    'operation': None,           # 'play' | 'record' | None
    'animal': None,
    'process': None,
    'record_file': None,         # in-flight recording target
    'last_record_file': None,    # last completed recording (for playback)
    'last_record_at': 0.0,       # timestamp when last recording completed
    'started_at': 0.0,
    'duration': 0.0,
}

def _detect_audio_devices():
    """Auto-detect USB capture/playback ALSA devices by parsing aplay -l / arecord -l.

    Returns (mic_device, speaker_device) as ALSA PCM strings like 'plughw:3,0'.
    Falls back to 'default' for either that can't be detected. Skips internal
    Tegra APE/HDA cards since they don't have real mic/speaker jacks.
    """
    card_re = re.compile(
        r'card\s+(\d+):\s+(\w+)\s+\[([^\]]+)\].*device\s+(\d+):\s+([^\[]+)\s+\[([^\]]+)\]'
    )

    def list_devices(cmd):
        try:
            out = subprocess.check_output(cmd, stderr=subprocess.STDOUT).decode('utf-8', errors='ignore')
        except Exception as e:
            print(f"[audio] {cmd} failed: {e}")
            return []
        cards = []
        for m in card_re.finditer(out):
            card_num, card_name, card_desc, dev_num, dev_type, dev_desc = m.groups()
            # Skip internal Tegra cards (no real analog jacks)
            if card_name in ('APE', 'HDA'):
                continue
            is_usb = ('USB' in dev_desc.upper()) or ('USB' in card_desc.upper())
            cards.append((is_usb, int(card_num), int(dev_num), card_name, card_desc))
        # Prefer USB devices, then by card number
        cards.sort(key=lambda c: (not c[0], c[1], c[2]))
        return cards

    mic_cards = list_devices(['arecord', '-l'])
    speaker_cards = list_devices(['aplay', '-l'])

    mic = f'plughw:{mic_cards[0][1]},{mic_cards[0][2]}' if mic_cards else 'default'
    spk = f'plughw:{speaker_cards[0][1]},{speaker_cards[0][2]}' if speaker_cards else 'default'

    mic_label = mic_cards[0][4] if mic_cards else '(default, no USB mic detected)'
    spk_label = speaker_cards[0][4] if speaker_cards else '(default, no USB speaker detected)'
    print(f"[audio] Mic device:    {mic}  [{mic_label}]")
    print(f"[audio] Speaker dev:  {spk}  [{spk_label}]")
    return mic, spk, mic_label, spk_label

MIC_DEVICE, SPEAKER_DEVICE, MIC_LABEL, SPEAKER_LABEL = _detect_audio_devices()

# Extract card number from "plughw:N,M" for amixer control
_speaker_match = re.match(r'\w+:(\d+),\d+', SPEAKER_DEVICE)
SPEAKER_CARD_NUM = int(_speaker_match.group(1)) if _speaker_match else None

def generate_animal_sound(animal, filepath):
    """Synthesize a short falcon (鹰隼) sound clip and save it as a WAV file."""
    sample_rate = 22050
    duration = 1.2
    n_samples = int(sample_rate * duration)

    samples = [0.0] * n_samples
    if animal == 'cry':
        # Falcon cry: high-pitched (3kHz -> 2kHz) piercing descending scream
        # with strong vibrato + 2nd harmonic, sharp attack, long fade.
        for i in range(n_samples):
            t = i / sample_rate
            phase = t / duration
            # Frequency: starts ~3 kHz, drops to ~2 kHz over the call
            freq = 3000 - 1000 * phase
            # Amplitude envelope: sharp attack (5%), sustain, fade out (last 25%)
            if phase < 0.05:
                env = phase / 0.05
            elif phase > 0.75:
                env = max(0.0, 1 - (phase - 0.75) / 0.25)
            else:
                env = 1.0
            # Strong, fast vibrato for the raptor scream character
            vibrato = 1 + 0.05 * math.sin(2 * math.pi * 18 * t)
            # Fundamental + 2nd harmonic for piercing quality
            fundamental = 0.35 * env * math.sin(2 * math.pi * freq * vibrato * t)
            harmonic = 0.10 * env * math.sin(2 * math.pi * 2 * freq * vibrato * t)
            samples[i] = fundamental + harmonic
    elif animal == 'cackle':
        # Falcon cackle: rapid staccato chattering at ~1.7-2 kHz
        # 9 short bursts (~60 ms each) with decreasing amplitude.
        burst_count = 9
        burst_dur = 0.06
        for i in range(n_samples):
            t = i / sample_rate
            val = 0.0
            for k in range(burst_count):
                start = 0.05 + k * (burst_dur + 0.04)
                rel = t - start
                if 0 <= rel < burst_dur:
                    # Quick attack, exponential decay within each burst
                    env = (1 - math.exp(-50 * rel)) * math.exp(-15 * rel)
                    # Decreasing amplitude across bursts
                    burst_amp = 0.40 * (1 - 0.6 * (k / (burst_count - 1)))
                    freq = 1700 + 300 * math.sin(2 * math.pi * 25 * rel)
                    val += burst_amp * env * math.sin(2 * math.pi * freq * rel)
            samples[i] = val
    elif animal == 'whistle':
        # Falcon whistle: smooth descending whistle 4 kHz -> 2.5 kHz
        # Fast attack, smooth long decay, ~0.8 s.
        wh_duration = 0.8
        wh_n = int(sample_rate * wh_duration)
        for i in range(wh_n):
            t = i / sample_rate
            phase = t / wh_duration
            # Smooth descending frequency
            freq = 4000 - 1500 * phase
            # Smooth attack (10%) and long decay (last 50%)
            if phase < 0.1:
                env = phase / 0.1
            elif phase > 0.5:
                env = max(0.0, 1 - (phase - 0.5) / 0.5)
            else:
                env = 1.0
            # Pure sine, no vibrato — clean whistle
            samples[i] = 0.30 * env * math.sin(2 * math.pi * freq * t)
        # Truncate to the actual whistle length so the WAV is exactly 0.8s
        samples = samples[:wh_n]

    with wave.open(filepath, 'w') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(sample_rate)
        for s in samples:
            v = max(-1.0, min(1.0, s))
            wf.writeframesraw(struct.pack('<h', int(v * 32767)))

def ensure_animal_sounds():
    """Ensure falcon sound WAV files exist.

    Uses real audio files from static/sounds/ if present.
    Falls back to synthetic generation into SOUNDS_DIR if real files are missing.
    """
    os.makedirs(SOUNDS_DIR, exist_ok=True)
    os.makedirs(RECORDING_DIR, exist_ok=True)
    for animal in ANIMAL_SOUNDS:
        path = os.path.join(SOUNDS_DIR, f'{animal}.wav')
        if os.path.exists(path):
            print(f"[audio] Real sound file found: {path}")
        else:
            print(f"[audio] Real sound missing for '{animal}', generating synthetic fallback -> {path}")
            generate_animal_sound(animal, path)

def normalize_wav(path):
    """Normalize a recording's peak amplitude to ~80% of full scale so quiet
    recordings (low mic gain, far from speaker) are still audible on playback.

    Reads the WAV, finds the absolute peak across all channels, scales all
    samples so that peak maps to 0.8 * 32767, and writes the file back in
    place. Skips silent recordings (peak < 100) to avoid amplifying noise.
    """
    try:
        with wave.open(path, 'rb') as w:
            nch = w.getnchannels()
            sw = w.getsampwidth()
            rate = w.getframerate()
            frames = w.readframes(w.getnframes())
        if sw != 2:
            return  # Only handle 16-bit PCM (arecord -f cd output)
        count = len(frames) // (sw * nch)
        if count == 0:
            return
        vals = struct.unpack('<' + 'h' * (count * nch), frames)
        peak = max(abs(v) for v in vals)
        if peak < 100:
            # Effectively silent — don't amplify noise
            return
        target = int(0.8 * 32767)
        scale = target / peak
        norm = [max(-32768, min(32767, int(v * scale))) for v in vals]
        out = struct.pack('<' + 'h' * len(norm), *norm)
        with wave.open(path, 'wb') as w:
            w.setnchannels(nch)
            w.setsampwidth(sw)
            w.setframerate(rate)
            w.writeframes(out)
        print(f"[audio] Normalized {os.path.basename(path)}: peak {peak} -> {target}")
    except Exception as e:
        print(f"[audio] normalize_wav failed for {path}: {e}")

def stop_audio_internal():
    """Stop any running audio playback or recording.

    If a recording was in progress, promotes the (possibly partial) file to
    last_record_file so the frontend can still offer playback after Stop.
    """
    with audio_lock:
        proc = audio_state.get('process')
        if proc and proc.poll() is None:
            try:
                proc.terminate()
                try:
                    proc.wait(timeout=1.0)
                except Exception:
                    try:
                        proc.kill()
                    except Exception:
                        pass
            except Exception:
                pass
        # If we were recording, promote the file so the user can still play it
        if audio_state.get('operation') == 'record' and audio_state.get('record_file'):
            rec_path = audio_state['record_file']
            if os.path.exists(rec_path):
                normalize_wav(rec_path)
                audio_state['last_record_file'] = rec_path
                audio_state['last_record_at'] = time.time()
        audio_state['operation'] = None
        audio_state['animal'] = None
        audio_state['process'] = None
        audio_state['record_file'] = None
        audio_state['started_at'] = 0.0
        audio_state['duration'] = 0.0

@app.route('/api/audio/animals')
@login_required
def audio_animals():
    ensure_animal_sounds()
    return jsonify({'success': True, 'animals': ANIMAL_SOUNDS})

@app.route('/api/audio/play/<animal>', methods=['POST'])
@login_required
def audio_play(animal):
    if animal not in ANIMAL_SOUNDS:
        return jsonify({'success': False, 'message': 'Unknown animal sound'})
    ensure_animal_sounds()
    wav_path = os.path.join(SOUNDS_DIR, f'{animal}.wav')
    if not os.path.exists(wav_path):
        return jsonify({'success': False, 'message': 'WAV file missing'})
    stop_audio_internal()
    try:
        proc = subprocess.Popen(['aplay', '-q', '-D', SPEAKER_DEVICE, wav_path])
        with audio_lock:
            audio_state['operation'] = 'play'
            audio_state['animal'] = animal
            audio_state['process'] = proc
            audio_state['started_at'] = time.time()
            audio_state['duration'] = float(ANIMAL_SOUNDS[animal].get('duration', 1.0))
        return jsonify({'success': True, 'message': f'Playing {ANIMAL_SOUNDS[animal]["name"]}'})
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})

@app.route('/api/audio/record', methods=['POST'])
@login_required
def audio_record():
    data = request.json or {}
    duration = max(1, min(30, int(data.get('duration', 3))))
    stop_audio_internal()
    rec_path = os.path.join(AUDIO_DIR, f'recording_{int(time.time())}.wav')
    try:
        proc = subprocess.Popen([
            'arecord', '-D', MIC_DEVICE, '-f', 'cd', '-d', str(duration), rec_path
        ])
        with audio_lock:
            audio_state['operation'] = 'record'
            audio_state['animal'] = None
            audio_state['process'] = proc
            audio_state['record_file'] = rec_path
            audio_state['started_at'] = time.time()
            audio_state['duration'] = float(duration)
        return jsonify({
            'success': True,
            'message': f'Recording {duration}s',
            'duration': duration,
            'record_file': rec_path
        })
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})

@app.route('/api/audio/stop', methods=['POST'])
@login_required
def audio_stop():
    stop_audio_internal()
    return jsonify({'success': True, 'message': 'Audio stopped'})

@app.route('/api/audio/status')
@login_required
def audio_status():
    with audio_lock:
        proc = audio_state.get('process')
        running = proc is not None and proc.poll() is None
        # Detect natural completion of a recording: when the arecord process
        # exits on its own (after -d seconds), promote record_file to
        # last_record_file so the frontend can offer playback.
        if (not running
                and audio_state.get('operation') == 'record'
                and audio_state.get('record_file')):
            rec_path = audio_state['record_file']
            normalize_wav(rec_path)
            audio_state['last_record_file'] = rec_path
            audio_state['last_record_at'] = time.time()
            audio_state['record_file'] = None
            audio_state['operation'] = None
        return jsonify({
            'success': True,
            'running': running,
            'operation': audio_state.get('operation') if running else None,
            'animal': audio_state.get('animal') if running else None,
            'elapsed': (time.time() - audio_state.get('started_at', 0.0)) if running else 0.0,
            'duration': audio_state.get('duration', 0.0) if running else 0.0,
            'last_record_file': audio_state.get('last_record_file'),
            'last_record_at': audio_state.get('last_record_at', 0.0),
            'mic_device': MIC_DEVICE,
            'mic_label': MIC_LABEL,
            'speaker_device': SPEAKER_DEVICE,
            'speaker_label': SPEAKER_LABEL,
        })

@app.route('/api/audio/wav/<filename>')
@login_required
def audio_wav(filename):
    """Serve a generated or recorded WAV file by basename."""
    safe = os.path.basename(filename)
    path = os.path.join(AUDIO_DIR, safe)
    if not os.path.exists(path):
        return jsonify({'success': False, 'message': 'Not found'}), 404
    return send_file(path, mimetype='audio/wav')

@app.route('/api/audio/volume', methods=['GET', 'POST'])
@login_required
def audio_volume():
    """Get or set the speaker (PCM) output volume via amixer.

    GET  -> {success, volume: 0-100, card, mixer: 'PCM'}
    POST -> {success, volume}  (body: {"volume": 0-100})
    """
    if SPEAKER_CARD_NUM is None:
        return jsonify({'success': False, 'message': 'No speaker card detected'})
    if request.method == 'GET':
        try:
            out = subprocess.check_output(
                ['amixer', '-c', str(SPEAKER_CARD_NUM), 'sget', 'PCM'],
                stderr=subprocess.STDOUT
            ).decode('utf-8', errors='ignore')
            m = re.search(r'\[(\d+)%\]', out)
            vol = int(m.group(1)) if m else 0
            return jsonify({
                'success': True, 'volume': vol,
                'card': SPEAKER_CARD_NUM, 'mixer': 'PCM'
            })
        except Exception as e:
            return jsonify({'success': False, 'message': str(e)})
    else:
        vol = max(0, min(100, int(request.json.get('volume', 50))))
        try:
            subprocess.run(
                ['amixer', '-c', str(SPEAKER_CARD_NUM), 'set', 'PCM', f'{vol}%'],
                check=True, capture_output=True
            )
            return jsonify({'success': True, 'volume': vol})
        except Exception as e:
            return jsonify({'success': False, 'message': str(e)})

# Generate the demo animal sounds at server startup so the panel is ready.
ensure_animal_sounds()

with app.app_context():
    db.create_all()
    
    if not User.query.filter_by(username='admin').first():
        admin = User(
            username='admin',
            password_hash=hash_password('admin123'),
            phone='+966123456789',
            role='admin',
            is_verified=True
        )
        db.session.add(admin)
        db.session.commit()
        print("Admin user created: username=admin, password=admin123")

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
