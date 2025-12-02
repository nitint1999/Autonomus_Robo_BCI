from flask import Flask, render_template_string, Response, jsonify, request
import cv2
import RPi.GPIO as GPIO
import time
import threading
import sys
import pandas as pd
import joblib

# Make sure you have adafruit-circuitpython-servokit installed:
# pip3 install adafruit-circuitpython-servokit
try:
    from adafruit_servokit import ServoKit
except ImportError:
    print("Warning: adafruit-circuitpython-servokit not found. Servo control may fail.")
    # Create a mock class if ServoKit is not available to allow the script to run
    class MockServo:
        def set_pulse_width_range(self, *args): pass
        @property
        def angle(self): return 90
        @angle.setter
        def angle(self, value): pass
    class MockKit:
        def __init__(self): self.servo = [MockServo()] * 16
    ServoKit = MockKit

# -------------------------------------------------------------
# FIX: Initialize the Flask app instance immediately after imports
app = Flask(__name__)
# -------------------------------------------------------------

# --- Global State, Thread Setup, and LOCK ---
current_state = "Stopped"
is_moving = False 
monitor_thread = None
radar_thread = None
is_radar_running = True 
radar_data = [] 
state_lock = threading.Lock() 
SERVO_DELAY_S = 0.03 
current_angle = 90 
ULTRASONIC_AVOID_DISTANCE_CM = 20 # NEW: Threshold for US obstacle avoidance

# Speed Constants (Defaults)
INITIAL_LINEAR_SPEED = 20      
INITIAL_TURN_SPEED = 20      

# DYNAMIC SPEED VARIABLES (Controlled by Sliders)
current_duty_cycle = INITIAL_LINEAR_SPEED    
current_turn_duty_cycle = INITIAL_TURN_SPEED 
PWM_FREQ = 100

# --- SEIZURE DETECTION GLOBALS ---
SEIZURE_LED_PIN_BCM = 27  # BCM pin 27 (Physical Pin 13)
IS_SEIZURE_DETECTED = False
SEIZURE_THREAD = None
IS_SEIZURE_MONITORING = True 
MODEL_PATH = '/home/naveen/Desktop/LED/decision_tree_model.joblib' 
DATA_PATH = '/home/naveen/Desktop/Final/project/Seizure_detection.xlsx'
TEST_ROW_INDEX = 12 
# -----------------------------------

# --- Motor Pin Setup (BCM) ---
IN1 = 26
IN2 = 19
IN3 = 13
IN4 = 6
en_a = 12
en_b = 5

# --- Sensor Pin Setup (BCM) ---
LEFT_IR_PIN = 22 
RIGHT_IR_PIN = 16

# NEW ULTRASONIC PINS
FRONT_TRIG_PIN = 20  
FRONT_ECHO_PIN = 21  
LEFT_TRIG_PIN = 24   # Example new BCM pin
LEFT_ECHO_PIN = 25   # Example new BCM pin
RIGHT_TRIG_PIN = 17   # Example new BCM pin
RIGHT_ECHO_PIN = 4   # Example new BCM pin

# Servo motor setup
kit = ServoKit(channels=16)
servo1 = kit.servo[0]
servo1.set_pulse_width_range(500, 2500) 

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Setup GPIO Pins 
# TRIG pins, Motor pins, and LED are outputs
for pin in [IN1, IN2, IN3, IN4, en_a, en_b, FRONT_TRIG_PIN, LEFT_TRIG_PIN, RIGHT_TRIG_PIN, SEIZURE_LED_PIN_BCM]:
    GPIO.setup(pin, GPIO.OUT)

# ECHO pins and IR pins are inputs
for pin in [LEFT_IR_PIN, RIGHT_IR_PIN, FRONT_ECHO_PIN, LEFT_ECHO_PIN, RIGHT_ECHO_PIN]:
    GPIO.setup(pin, GPIO.IN) 

p = GPIO.PWM(en_a, PWM_FREQ)
p.start(0) 

q = GPIO.PWM(en_b, PWM_FREQ)
q.start(0) 

# --- Motor Control Functions (No change) ---
def set_speed(duty_cycle):
    p.ChangeDutyCycle(duty_cycle)
    q.ChangeDutyCycle(duty_cycle)

def forward():
    set_speed(current_duty_cycle)
    GPIO.output(IN1, True)
    GPIO.output(IN2, False)
    GPIO.output(IN3, False) 
    GPIO.output(IN4, True)

def stop_motors():
    set_speed(0)
    for pin in [IN1, IN2, IN3, IN4]:
        GPIO.output(pin, False)

def backward():
    set_speed(current_duty_cycle)
    GPIO.output(IN1, False)
    GPIO.output(IN2, True)
    GPIO.output(IN3, True)
    GPIO.output(IN4, False)

def left():
    set_speed(current_turn_duty_cycle)
    GPIO.output(IN1, True)
    GPIO.output(IN2, False)
    GPIO.output(IN3, False) 
    GPIO.output(IN4, False)
    
def right():
    set_speed(current_turn_duty_cycle)
    GPIO.output(IN1, False)
    GPIO.output(IN2, False)
    GPIO.output(IN3, False)
    GPIO.output(IN4, True)

# --- Generalized Ultrasonic Sensor Function (MODIFIED) ---
def read_distance(TRIG_PIN_IN, ECHO_PIN_IN):
    """Measures distance for a specific TRIG/ECHO pair."""
    
    # 1. Reset/Clear Trigger
    GPIO.output(TRIG_PIN_IN, False)
    time.sleep(0.000002) 

    # 2. Trigger Pulse
    GPIO.output(TRIG_PIN_IN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN_IN, False)

    pulse_start = time.time()
    pulse_end = time.time()
    
    # 3. Wait for Echo Start (Pulse Start)
    timeout_start = time.time()
    while GPIO.input(ECHO_PIN_IN) == 0 and (time.time() - timeout_start) < 0.05:
        pulse_start = time.time()

    # 4. Wait for Echo End (Pulse End)
    timeout_start = time.time()
    while GPIO.input(ECHO_PIN_IN) == 1 and (time.time() - timeout_start) < 0.05:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150 # Speed of sound = 343 m/s = 34300 cm/s. Half speed for distance = 17150 cm/s

    distance = round(distance, 2)
    
    # Return 0 for out-of-range/failed readings
    if distance > 400 or distance < 2 or pulse_duration >= 0.05:
        return 0 
        
    return distance

# --- Radar Function (MODIFIED to use FRONT sensor pins) ---
def radar():
    """Runs the servo sweep and collects angle/distance data using the FRONT sensor."""
    global radar_data, is_radar_running, current_angle
    
    servo1.angle = 90
    time.sleep(1) 

    while is_radar_running: 
        current_data = [] 

        # Sweep from 0 to 180 degrees
        for angle in range(0, 181, 5): 
            with state_lock:
                if not is_radar_running: break
            
            servo1.angle = angle
            time.sleep(SERVO_DELAY_S) 
            # Use the FRONT sensor for the radar sweep
            distance = read_distance(FRONT_TRIG_PIN, FRONT_ECHO_PIN)
            current_data.append((angle, distance))
            
            with state_lock:
                current_angle = angle 
            
        if not is_radar_running: break 
            
        # Sweep from 180 to 0 degrees
        for angle in range(180, -1, -5):
            with state_lock:
                if not is_radar_running: break
            
            servo1.angle = angle
            time.sleep(SERVO_DELAY_S) 
            # Use the FRONT sensor for the radar sweep
            distance = read_distance(FRONT_TRIG_PIN, FRONT_ECHO_PIN)
            current_data.append((angle, distance))
            
            with state_lock:
                current_angle = angle 
            
        # Safely update the shared global radar data
        with state_lock:
            if is_radar_running: 
                radar_data = current_data
        
        # Pause before the next full sweep
        time.sleep(0.5)

    servo1.angle = 90
    with state_lock:
        current_angle = 90 
    print("Radar sweep stopped.")

def start_radar_thread():
    """Starts the radar thread if it's not already running."""
    global radar_thread, is_radar_running
    with state_lock:
        if not is_radar_running:
            is_radar_running = True
            radar_thread = threading.Thread(target=radar, daemon=True)
            radar_thread.start()
            print("Radar thread started.")
        else:
            print("Radar thread is already running.")


# --- IR and Ultrasonic Sensor Reading & Monitoring Functions (MODIFIED) ---
def get_sensor_status():
    """Reads and formats status for all 5 sensors (2 IR, 3 US)."""
    left_ir_state = GPIO.input(LEFT_IR_PIN)
    right_ir_state = GPIO.input(RIGHT_IR_PIN)
    
    # Read all three ultrasonic sensors
    front_dist = read_distance(FRONT_TRIG_PIN, FRONT_ECHO_PIN)
    left_dist = read_distance(LEFT_TRIG_PIN, LEFT_ECHO_PIN)
    right_dist = read_distance(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN)
    
    left_status = "DETECTED" if left_ir_state == GPIO.LOW else "Clear"
    right_status = "DETECTED" if right_ir_state == GPIO.LOW else "Clear"
    
    return f"IR L: {left_status} | IR R: {right_status} | US F: {front_dist}cm | US L: {left_dist}cm | US R: {right_dist}cm"

def obstacle_monitor():
    """Continuously monitors IR and Ultrasonic sensors for auto-avoidance."""
    global current_state, is_moving
    
    while True:
        with state_lock:
            moving_status = is_moving

        if moving_status:
            left_ir_state = GPIO.input(LEFT_IR_PIN)
            right_ir_state = GPIO.input(RIGHT_IR_PIN)
            
            front_dist = read_distance(FRONT_TRIG_PIN, FRONT_ECHO_PIN)
            left_dist = read_distance(LEFT_TRIG_PIN, LEFT_ECHO_PIN)
            right_dist = read_distance(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN)
            
            # --- PRIMARY CHECK: STOP if Critical Obstacle Detected ---
            # 1. Front US is too close OR
            # 2. Both IRs detected (e.g., about to fall into a hole/cliff)
            if (front_dist > 0 and front_dist < ULTRASONIC_AVOID_DISTANCE_CM) or \
               (left_ir_state == GPIO.LOW and right_ir_state == GPIO.LOW):
                
                stop_motors()
                with state_lock:
                    current_state = f"🚨 AUTO STOP: Obstacle Front ({front_dist}cm) or Both IRs"
                    is_moving = False
                print(current_state)
                # Continue loop iteration to allow manual control resumption later
                time.sleep(0.05)
                continue
            
            # --- SECONDARY CHECK: Auto Avoidance Turn ---
            
            # Left Obstacle (US or IR) -> Turn Right
            elif (left_dist > 0 and left_dist < ULTRASONIC_AVOID_DISTANCE_CM) or (left_ir_state == GPIO.LOW):
                stop_motors()
                time.sleep(0.05)
                right() # Obstacle left -> turn right
                
                with state_lock:
                    new_state = f"➡️ AUTO AVOID: Obstacle Left. Turned Right ({current_turn_duty_cycle}%)"
                    current_state = new_state
                print(current_state)

                time.sleep(0.3)
                stop_motors()
                forward() # Resume forward movement
            
            # Right Obstacle (US or IR) -> Turn Left
            elif (right_dist > 0 and right_dist < ULTRASONIC_AVOID_DISTANCE_CM) or (right_ir_state == GPIO.LOW):
                stop_motors()
                time.sleep(0.05)
                left()  # Obstacle right -> turn left
                
                with state_lock:
                    new_state = f"⬅️ AUTO AVOID: Obstacle Right. Turned Left ({current_turn_duty_cycle}%)"
                    current_state = new_state
                print(current_state)

                time.sleep(0.3)
                stop_motors()
                forward() # Resume forward movement
                
        time.sleep(0.05) 

# -----------------------------------------------------
# --- SEIZURE DETECTION THREAD FUNCTION (No change) ---
def seizure_detection_monitor():
    """Continuously uses the loaded model and data to check for seizures, toggling the LED on detection."""
    global IS_SEIZURE_DETECTED, IS_SEIZURE_MONITORING, TEST_ROW_INDEX
    
    # 1. Load Model and Data ONCE
    try:
        loaded_model = joblib.load(MODEL_PATH)
        data = pd.read_excel(DATA_PATH)
        df = pd.DataFrame(data)
        df = df.drop(['Time'], axis=1)
        df = df.transpose()
        x_data = df.iloc[:, :-1]
        print("Seizure Detection Model and Data loaded successfully.")
        
        GPIO.output(SEIZURE_LED_PIN_BCM, False) # Ensure LED starts OFF
        
    except FileNotFoundError as e:
        print(f"Error: Required file not found: {e}. Seizure detection disabled.")
        return
    except Exception as e:
        print(f"Error during initialization (Data/Model Load): {e}")
        return
        
    print(f"Seizure Detection Monitor started (LED output on BCM {SEIZURE_LED_PIN_BCM}).")

    while IS_SEIZURE_MONITORING:
        try:
            # Get the current row index from the global variable
            current_n = TEST_ROW_INDEX 
            
            # 2. Acquire Data Point
            # Ensure the row index is within bounds
            if current_n < 0 or current_n >= len(x_data):
                print(f"Warning: Invalid row index {current_n}. Skipping detection cycle.")
                time.sleep(1)
                continue
                
            sample_query_column = x_data.iloc[current_n:current_n+1] 
            
            # 3. Make Prediction
            prediction = loaded_model.predict(sample_query_column)
            seizure_predicted = (prediction[0] == 1)
            
            # 4. Act based on the Prediction (LED Toggling Only)
            if seizure_predicted:
                with state_lock:
                    IS_SEIZURE_DETECTED = True
                    
                # Blinking LED pattern (0.1s ON, 0.3s OFF)
                GPIO.output(SEIZURE_LED_PIN_BCM, True)
                time.sleep(0.1)
                GPIO.output(SEIZURE_LED_PIN_BCM, False)
                time.sleep(0.3)
                print(f"🚨 SEIZURE ALERT: Detected at Row {current_n}! Toggling LED.") 
                
            else:
                with state_lock:
                    IS_SEIZURE_DETECTED = False
                    
                GPIO.output(SEIZURE_LED_PIN_BCM, False)
                time.sleep(1) # Check again after 1 second
                
        except Exception as e:
            print(f"Seizure Detection Error during loop: {e}")
            time.sleep(5) 

    print("Seizure Detection Monitor stopped.")
# -----------------------------------------------------


# --- Initialize and Start Threads (No change) ---
monitor_thread = threading.Thread(target=obstacle_monitor, daemon=True)
monitor_thread.start()

start_radar_thread()

# --- Initialize and Start NEW Seizure Detection Thread ---
SEIZURE_THREAD = threading.Thread(target=seizure_detection_monitor, daemon=True)
SEIZURE_THREAD.start()
# ---------------------------------------------------------

# --- Camera Setup (No change) ---
try:
    camera = cv2.VideoCapture(0)
    
    if not camera.isOpened():
        camera = cv2.VideoCapture(1)
        if not camera.isOpened():
             print("Warning: Could not open camera. Continuing without video feed.")
             camera = None
except cv2.error as e:
    print(f"OpenCV Error: {e}")
    camera = None
except Exception as e:
    print(f"General Camera Error: {e}")
    camera = None

def gen_frames():
    if not camera:
        return
    
    while True:
        success, frame = camera.read()
        if not success:
            break
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

# --------------------------------------------------------------------------------------------------------------------------------------
# --- Flask Routes (No functional change to routes, only status update in index) ---
@app.route("/")
def index():
    sensor_status = get_sensor_status()
    with state_lock:
        current_state_locked = current_state
        current_duty_cycle_locked = current_duty_cycle
        current_turn_duty_cycle_locked = current_turn_duty_cycle
        seizure_status = IS_SEIZURE_DETECTED 
        test_row_index_locked = TEST_ROW_INDEX
        
    return render_template_string(html, 
                                 current_state=current_state_locked, 
                                 sensor_status=sensor_status,
                                 LINEAR_SPEED=current_duty_cycle_locked, 
                                 TURN_SPEED=current_turn_duty_cycle_locked,
                                 IS_SEIZURE_DETECTED=seizure_status,
                                 TEST_ROW_INDEX=test_row_index_locked) 

# ... (Speed, Index, Radar, Stop/Start, Video feed routes remain unchanged) ...

@app.route("/set_linear_speed", methods=['POST'])
def set_linear_motor_speed():
    global current_duty_cycle
    try:
        data = request.json
        new_speed = int(data.get('speed', current_duty_cycle)) 
        
        new_speed = max(0, min(100, new_speed))
        
        with state_lock:
            current_duty_cycle = new_speed
            
            if "Forward" in current_state or "Backward" in current_state:
                set_speed(current_duty_cycle)
                
        return jsonify({"success": True, "new_speed": current_duty_cycle})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 400

@app.route("/set_turn_speed", methods=['POST'])
def set_turn_motor_speed():
    global current_turn_duty_cycle
    try:
        data = request.json
        new_speed = int(data.get('speed', current_turn_duty_cycle)) 
        
        new_speed = max(0, min(100, new_speed))
        
        with state_lock:
            current_turn_duty_cycle = new_speed
            
            if "Turning Left" in current_state or "Turning Right" in current_state or "AUTO AVOID" in current_state:
                pass 
                
        return jsonify({"success": True, "new_speed": current_turn_duty_cycle})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 400

@app.route("/set_test_row_index", methods=['POST'])
def set_test_row():
    global TEST_ROW_INDEX
    try:
        data = request.json
        new_index = int(data.get('row_index', TEST_ROW_INDEX)) 
        
        new_index = max(0, new_index)
        
        with state_lock:
            TEST_ROW_INDEX = new_index
            
        print(f"✅ TEST_ROW_INDEX updated to: {TEST_ROW_INDEX}") 
            
        return jsonify({"success": True, "new_index": TEST_ROW_INDEX})
    except ValueError:
        return jsonify({"success": False, "error": "Invalid index format (must be an integer)."}), 400
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 400


@app.route("/radar_data")
def get_radar_data():
    """Returns the current angle/distance data and instantaneous angle."""
    with state_lock:
        data = list(radar_data) 
        angle = current_angle 
    return jsonify({
        "data": data,
        "current_angle": angle
    })

@app.route("/status")
def get_status_json():
    """Returns the current state, sensor status, and running statuses as JSON, including seizure status and test index."""
    with state_lock:
        state = current_state
        moving = is_moving
        radar_running = is_radar_running 
        linear_speed = current_duty_cycle
        turn_speed = current_turn_duty_cycle 
        seizure_status = IS_SEIZURE_DETECTED 
        test_row_index = TEST_ROW_INDEX 
    
    sensor_data_string = get_sensor_status()
    
    return jsonify({
        "state": state,
        "is_moving": moving,
        "sensor_status": sensor_data_string,
        "is_radar_running": radar_running,
        "linear_speed": linear_speed,
        "turn_speed": turn_speed,
        "is_seizure_detected": seizure_status,
        "test_row_index": test_row_index
    })

@app.route("/stop_radar", methods=['POST'])
def stop_radar_route():
    global is_radar_running, radar_data
    with state_lock:
        is_radar_running = False
        radar_data = [] 
        servo1.angle = 90 
    return index()

@app.route("/start_radar", methods=['POST'])
def start_radar_route():
    start_radar_thread()
    return index()

@app.route("/forward", methods=['POST'])
def go_forward():
    global current_state, is_moving
    forward()
    with state_lock:
        current_state = f"Moving Forward (Speed: {current_duty_cycle}%)"
        is_moving = True 
    return index()

@app.route("/backward", methods=['POST'])
def go_backward():
    global current_state, is_moving
    backward()
    with state_lock:
        current_state = f"Moving Backward (Speed: {current_duty_cycle}%)"
        is_moving = True 
    return index()

@app.route("/left", methods=['POST'])
def go_left():
    global current_state, is_moving
    left()
    with state_lock:
        current_state = f"Turning Left (Speed: {current_turn_duty_cycle}%)"
        is_moving = True 
    return index()

@app.route("/right", methods=['POST'])
def go_right():
    global current_state, is_moving
    right()
    with state_lock:
        current_state = f"Turning Right (Speed: {current_turn_duty_cycle}%)"
        is_moving = True 
    return index()

@app.route("/stop", methods=['POST'])
def go_stop():
    global current_state, is_moving
    stop_motors()
    with state_lock:
        current_state = "Stopped"
        is_moving = False
    return index()

@app.route("/video_feed")
def video_feed():
    return Response(gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')
# --------------------------------------------------------------------------------------------------------------------------------------

# --- HTML Template (No change needed here, as the JS automatically polls the new sensor_status string) ---
html = """
<!doctype html>
<html lang="en">
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>4WD Robot Control & Radar</title>
    <script src="https://cdn.tailwindcss.com"></script>
    <script src="https://cdn.jsdelivr.net/npm/chart.js@3.7.1/dist/chart.min.js"></script>
    <style>
        body { font-family: 'Inter', sans-serif; }
        form { display: inline-block; margin: 0.5rem; }
        .radar-container {
            position: relative;
            width: 100%;
            padding-bottom: 50%; 
            height: 0;
            background-color: #1a1a1a;
            border-radius: 0.5rem;
            overflow: hidden; 
        }
        .radar-container canvas {
            position: absolute;
            width: 100% !important;
            height: 100% !important;
        }
        .chart-grid-green .chartjs-render-monitor {
            background-color: #1a1a1a !important; 
        }
        .range-lg {
            -webkit-appearance: none;
            height: 8px;
            background: #d3d3d3;
            outline: none;
            opacity: 0.7;
            -webkit-transition: .2s;
            transition: opacity .2s;
        }
        .range-lg::-webkit-slider-thumb {
            -webkit-appearance: none;
            appearance: none;
            width: 25px;
            height: 25px;
            border-radius: 50%;
            background: #4F46E5;
            cursor: pointer;
            border: 2px solid white;
        }
    </style>
</head>
<body class="bg-gray-100 p-4 sm:p-8">

    <div class="max-w-4xl mx-auto bg-white shadow-xl rounded-xl p-6">
        <h1 class="text-3xl font-bold text-gray-800 mb-4 text-center">
            4WD Robot Controller & Radar
        </h1>
        <p class="text-sm text-gray-500 mb-6 text-center">
            Linear Speed: <span id="linear-speed-display-summary">{{ LINEAR_SPEED }}%</span> | Turn Speed: <span id="turn-speed-display-summary">{{ TURN_SPEED }}%</span>
        </p>
        
        <div class="grid grid-cols-1 mb-6">
            <div class="border-4 border-gray-200 rounded-lg overflow-hidden">
                <h3 class="text-lg font-semibold p-2 bg-gray-200 text-center">Camera Feed</h3>
                <img id="video-feed" src="/video_feed" class="w-full h-auto object-cover" onerror="this.src='https://placehold.co/640x480/E0E7FF/4338CA?text=Camera+Feed+Unavailable';" alt="Robot Camera Feed">
            </div>
        </div>

        <div class="mb-6 space-y-2">
            <h3 class="text-lg font-semibold p-2 bg-blue-50 rounded-lg shadow-inner">
                Movement Status: <span id="movement-status" class="font-normal text-blue-700">{{ current_state }}</span>
            </h3> 
            <h3 class="text-lg font-semibold p-2 bg-green-50 rounded-lg shadow-inner">
                Sensor Status: <span id="sensor-status" class="font-normal text-green-700">{{ sensor_status }}</span>
            </h3>
            
            <h3 class="text-lg font-semibold p-2 rounded-lg shadow-inner" id="seizure-status-box">
                🧠 Seizure Monitor: <span id="seizure-status-display" class="font-normal">Monitoring...</span>
            </h3>
            
            <div class="p-4 border border-gray-300 rounded-lg bg-yellow-50">
                <label for="test-row-index-input" class="block text-md font-semibold text-gray-800 mb-2">
                    Test Row Index (0 to N): 
                </label>
                <div class="flex space-x-2">
                    <input type="number" id="test-row-index-input" 
                               value="{{ TEST_ROW_INDEX }}" min="0" 
                               class="flex-grow p-2 border border-gray-400 rounded-md shadow-sm focus:ring-blue-500 focus:border-blue-500"
                               placeholder="Enter row number">
                    <button id="set-index-btn" class="bg-indigo-500 hover:bg-indigo-600 text-white font-bold py-2 px-4 rounded-md transition duration-150">
                        Set Index
                    </button>
                </div>
                <p class="text-sm text-gray-600 mt-1">Current Active Index: <span id="current-test-index">{{ TEST_ROW_INDEX }}</span></p>
            </div>
            </div>

        <div class="flex flex-col items-center space-y-4 mb-8 border-b pb-8">

            <div class="w-full mb-4 p-4 border border-gray-300 rounded-lg bg-gray-50">
                <label for="linear-speed-slider" class="block text-xl font-semibold text-gray-800 mb-2 text-center">
                    Linear Speed (Fwd/Bwd): <span id="linear-speed-display">{{ LINEAR_SPEED }}%</span>
                </label>
                <input type="range" 
                         id="linear-speed-slider" 
                         min="0" 
                         max="100" 
                         value="{{ LINEAR_SPEED }}" 
                         class="w-full h-3 bg-gray-300 rounded-lg appearance-none cursor-pointer range-lg">
                <p class="text-sm text-gray-500 mt-1 flex justify-between">
                    <span>0% (Stop)</span>
                    <span>100% (Max)</span>
                </p>
            </div>
            
            <div class="w-full mb-6 p-4 border border-gray-300 rounded-lg bg-gray-50">
                <label for="turn-speed-slider" class="block text-xl font-semibold text-gray-800 mb-2 text-center">
                    Turn Speed (Left/Right): <span id="turn-speed-display">{{ TURN_SPEED }}%</span>
                </label>
                <input type="range" 
                         id="turn-speed-slider" 
                         min="0" 
                         max="100" 
                         value="{{ TURN_SPEED }}" 
                         class="w-full h-3 bg-gray-300 rounded-lg appearance-none cursor-pointer range-lg">
                <p class="text-sm text-gray-500 mt-1 flex justify-between">
                    <span>0% (Stop)</span>
                    <span>100% (Max)</span>
                </p>
            </div>


            <form action="/forward" method="POST" class="w-full">
                <button type="submit" class="w-full bg-blue-500 hover:bg-blue-600 text-white font-bold py-3 px-6 rounded-lg shadow-lg transition duration-150 ease-in-out">
                    ⬆️ Forward
                </button>
            </form>

            <div class="flex justify-center space-x-4 w-full">
                <form action="/left" method="POST" class="w-1/3">
                    <button type="submit" class="w-full bg-yellow-500 hover:bg-yellow-600 text-white font-bold py-3 px-6 rounded-lg shadow-lg transition duration-150 ease-in-out">
                        ⬅️ Left
                    </button>
                </form>
                
                <form action="/stop" method="POST" class="w-1/3">
                    <button type="submit" class="w-full bg-red-800 hover:bg-red-600 text-white font-bold py-3 px-6 rounded-lg shadow-lg transition duration-150 ease-in-out">
                        🛑 Stop Motors
                    </button>
                </form>

                <form action="/right" method="POST" class="w-1/3">
                    <button type="submit" class="w-full bg-yellow-500 hover:bg-yellow-600 text-white font-bold py-3 px-6 rounded-lg shadow-lg transition duration-150 ease-in-out">
                        ➡️ Right
                    </button>
                </form>
            </div>

            <form action="/backward" method="POST" class="w-full">
                <button type="submit" class="w-full bg-blue-500 hover:bg-blue-600 text-white font-bold py-3 px-6 rounded-lg shadow-lg transition duration-150 ease-in-out">
                    ⬇️ Backward
                </button>
            </form>
        </div>
        
        <h2 class="text-2xl font-bold text-gray-700 mb-4 text-center">
            📡 Ultrasonic Radar System
        </h2>
        
        <div class="mb-6 space-y-2">
            <h3 class="text-lg font-semibold p-2 bg-purple-50 rounded-lg shadow-inner">
                Radar Status: <span id="radar-last-update" class="font-normal text-purple-700">Running</span>
            </h3>
            
            <div class="flex justify-center space-x-4 pt-4">
                <form action="/start_radar" method="POST" class="w-1/3">
                    <button type="submit" id="start-radar-btn" class="w-full bg-green-500 hover:bg-green-600 text-white font-bold py-3 px-6 rounded-lg shadow-lg transition duration-150 ease-in-out">
                        ▶️ Start Radar
                    </button>
                </form>
                <form action="/stop_radar" method="POST" class="w-1/3" onsubmit="return confirm('Stop the servo sweep?');">
                    <button type="submit" id="stop-radar-btn" class="w-full bg-red-500 hover:bg-red-600 text-white font-bold py-3 px-6 rounded-lg shadow-lg transition duration-150 ease-in-out">
                        ⏹️ Stop Radar
                    </button>
                </form>
            </div>
        </div>

        <div class="grid grid-cols-1 mb-6">
            <div class="border-4 border-gray-200 rounded-lg p-4 bg-[#1a1a1a] chart-grid-green">
                <h3 class="text-lg font-semibold p-2 bg-gray-700 text-white text-center mb-2 rounded">Ultrasonic Radar Plot (180°)</h3>
                <div class="radar-container">
                    <canvas id="radarChart"></canvas>
                </div>
            </div>
        </div>
    </div>
    
    <script>
        let radarChart;
        let MAX_DISTANCE = 200; 
        let currentScanAngle = 90; 

        // --- Chart.js Plugin to Draw Scanning Line ---
        const scannerLinePlugin = {
            id: 'scannerLine',
            beforeDraw(chart) {
                const ctx = chart.ctx;
                
                if (chart.options.plugins.scannerLine.enabled === false) return;

                if (chart.options.scales.r) {
                    const rScale = chart.scales.r;
                    const center = {
                        x: rScale.xCenter,
                        y: rScale.yCenter
                    };
                    const maxRadius = rScale.drawingArea; 
                    
                    const mappedAngleRad = Math.PI * (1 - (currentScanAngle / 180));
                    
                    const xEnd = center.x + maxRadius * Math.cos(mappedAngleRad);
                    const yEnd = center.y - maxRadius * Math.sin(mappedAngleRad); 

                    ctx.save();
                    ctx.beginPath();
                    ctx.strokeStyle = 'rgba(0, 255, 0, 0.8)'; 
                    ctx.lineWidth = 2;
                    ctx.setLineDash([5, 5]); 
                    ctx.moveTo(center.x, center.y);
                    ctx.lineTo(xEnd, yEnd);
                    ctx.stroke();
                    ctx.restore();
                }
            }
        };

        Chart.register(scannerLinePlugin);


        // --- Speed Slider Control Functions ---
        
        // Function for Linear Speed (Forward/Backward)
        function sendLinearSpeedUpdate(speed) {
            document.getElementById('linear-speed-display').textContent = `${speed}%`;
            document.getElementById('linear-speed-display-summary').textContent = `${speed}%`;

            fetch('/set_linear_speed', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({ speed: speed })
            })
            .then(response => response.json())
            .then(data => {
                if (!data.success) {
                    console.error("Failed to update linear speed:", data.error);
                }
            })
            .catch(error => console.error('Error sending linear speed update:', error));
        }

        // Function for Turn Speed (Left/Right/Auto Avoid)
        function sendTurnSpeedUpdate(speed) {
            document.getElementById('turn-speed-display').textContent = `${speed}%`;
            document.getElementById('turn-speed-display-summary').textContent = `${speed}%`;


            fetch('/set_turn_speed', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({ speed: speed })
            })
            .then(response => response.json())
            .then(data => {
                if (!data.success) {
                    console.error("Failed to update turn speed:", data.error);
                }
            })
            .catch(error => console.error('Error sending turn speed update:', error));
        }

        // --- Index Setting Function ---
        function sendTestIndexUpdate(index) {
            fetch('/set_test_row_index', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({ row_index: index })
            })
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    console.log("Updated Test Row Index:", data.new_index);
                    
                    // 1. Update the display span to confirm 
                    document.getElementById('current-test-index').textContent = data.new_index;
                } else {
                    console.error("Failed to update test row index:", data.error);
                    alert("Error updating index: " + data.error);
                }
            })
            .catch(error => console.error('Error sending index update:', error));
        }

        // --- Radar Plotting Function ---
        function updateRadarPlot(radarData, currentAngle) {
            currentScanAngle = currentAngle;

            if (!radarChart) {
                const ctx = document.getElementById('radarChart').getContext('2d');
                radarChart = new Chart(ctx, {
                    type: 'polarArea', // Using polarArea for simplicity, radar/scatter could also work
                    data: {
                        datasets: [{
                            label: 'Obstacles',
                            data: [],
                            backgroundColor: 'rgba(255, 99, 132, 0.7)',
                            borderColor: 'rgba(255, 99, 132, 1)',
                            pointRadius: 6,
                            pointStyle: 'circle',
                            borderWidth: 1
                        }]
                    },
                    options: {
                        responsive: true,
                        maintainAspectRatio: true,
                        aspectRatio: 2, 
                        layout: {
                            padding: 0
                        },
                        scales: {
                            r: {
                                angleLines: {
                                    display: true,
                                    color: 'rgba(0, 255, 0, 0.3)' 
                                },
                                grid: {
                                    color: 'rgba(0, 255, 0, 0.3)', 
                                    circular: true
                                },
                                pointLabels: {
                                    display: false 
                                },
                                ticks: {
                                    display: false, 
                                    max: MAX_DISTANCE, 
                                    stepSize: MAX_DISTANCE / 4,
                                    backdropColor: 'transparent',
                                    color: 'rgba(0, 255, 0, 0.7)'
                                },
                                min: 0,
                                max: MAX_DISTANCE, 
                                beginAtZero: true
                            },
                            // Override the angular axis to show 0 to 180 degrees
                            x: {
                                display: false 
                            },
                            y: {
                                display: false
                            }
                        },
                        plugins: {
                            legend: {
                                display: false
                            },
                            tooltip: {
                                callbacks: {
                                    label: function(context) {
                                        let label = context.dataset.label || '';
                                        if (label) {
                                            label += ': ';
                                        }
                                        label += Math.round(context.parsed.r) + ' cm at ' + context.label + '°';
                                        return label;
                                    }
                                }
                            },
                            scannerLine: {
                                enabled: true 
                            }
                        },
                        // Custom drawing to crop the chart to 180 degrees
                        // This logic is complex and usually done with a custom Chart.js plugin or container clipping
                        // For simplicity, we rely on the points being plotted in the range [0, 180]
                    }
                });
            }

            const points = [];
            const labels = [];
            const radarDataset = radarChart.data.datasets[0];

            radarData.forEach(([angle, distance]) => {
                if (distance > 0) {
                    // Use only angles 0-180 (top half of the polar chart)
                    labels.push(angle.toString());
                    points.push({
                        x: angle, // Angle is usually represented as the 'label' in polar charts, but we map it here
                        r: distance // Distance is the radial value
                    });
                }
            });

            // For polar charts, data array usually contains radial values in the order of the labels.
            // Since we are using an object-based data model for Chart.js scatter/polar, we need to adapt.
            // Reinitializing the chart data structure for compatibility:
            const polarLabels = Array.from({length: 181}, (_, i) => i.toString()); // Labels 0 to 180
            const polarData = new Array(181).fill(null);
            
            radarDataset.data = []; // Clear old data

            radarData.forEach(([angle, distance]) => {
                if (distance > 0 && distance < MAX_DISTANCE) {
                    // Convert the polar (angle, distance) data to Cartesian (x, y) for visualization
                    // Chart.js polar area requires conversion or special plugin for plotting points like this.
                    // Instead of full conversion, we will just pass the radial data for visualization:
                    
                    // We need the data structure to plot points at specific angles/distances.
                    // The easiest way for this type of visualization is a scatter plot on a custom polar scale.
                    // Since Chart.js polarArea is limited, we stick to the basic update:
                    radarDataset.data.push({
                        x: angle, // We treat x as angle
                        y: distance // We treat y as distance (radial value)
                    });
                }
            });
            
            // To properly plot on a polar chart, we need a dataset type that handles (angle, distance) pairs, 
            // which in Chart.js is often a radar chart or a scatter plot on a radial axis.
            // For now, we update the points in a simple way, assuming the angle is the index for simplicity.
            
            // Rebuild the radar chart data for a scatter-like polar effect:
             const newDataset = {
                label: 'Obstacles',
                data: [],
                backgroundColor: (context) => {
                    const distance = context.raw.r;
                    if (distance > 0 && distance < 30) return 'rgba(255, 0, 0, 0.8)'; // Red for very close
                    if (distance >= 30 && distance < 100) return 'rgba(255, 165, 0, 0.8)'; // Orange for close
                    return 'rgba(0, 200, 255, 0.8)'; // Blue for distant
                },
                borderColor: 'white',
                pointRadius: 7,
                pointStyle: 'circle',
                borderWidth: 1,
            };
            
            radarData.forEach(([angle, distance]) => {
                if (distance > 0 && distance < MAX_DISTANCE) {
                    // Use a scatter-like object {r: radial value, angle: angular value}
                    newDataset.data.push({
                        r: distance, 
                        angle: angle // Custom property we need to manually map to the chart space
                    });
                }
            });
            
            // Since Chart.js `polarArea` is designed for categorical data, 
            // this custom plotting of points requires a complex, custom plugin.
            // For this project, we assume a simplified data structure update.
            // Let's ensure the dataset is replaced cleanly for best results:
            radarChart.data.datasets = [newDataset];


            radarChart.update('none'); // Update without animation
        }
        
        // --- Status Polling Function ---
        function updateStatus() {
            fetch('/status')
                .then(response => response.json())
                .then(data => {
                    // Movement Status
                    document.getElementById('movement-status').textContent = data.state;
                    
                    // Sensor Status (Updated to include 3 US sensors)
                    document.getElementById('sensor-status').textContent = data.sensor_status;

                    // Speed Displays
                    document.getElementById('linear-speed-display').textContent = `${data.linear_speed}%`;
                    document.getElementById('linear-speed-slider').value = data.linear_speed;
                    document.getElementById('linear-speed-display-summary').textContent = `${data.linear_speed}%`;

                    document.getElementById('turn-speed-display').textContent = `${data.turn_speed}%`;
                    document.getElementById('turn-speed-slider').value = data.turn_speed;
                    document.getElementById('turn-speed-display-summary').textContent = `${data.turn_speed}%`;

                    // Seizure Status
                    const statusBox = document.getElementById('seizure-status-box');
                    const statusDisplay = document.getElementById('seizure-status-display');
                    document.getElementById('current-test-index').textContent = data.test_row_index; // Update index display

                    if (data.is_seizure_detected) {
                        statusDisplay.textContent = "🚨 SEIZURE DETECTED! 🚨";
                        statusBox.className = "text-lg font-semibold p-2 rounded-lg shadow-inner bg-red-200 text-red-800 animate-pulse";
                    } else {
                        statusDisplay.textContent = "All Clear.";
                        statusBox.className = "text-lg font-semibold p-2 rounded-lg shadow-inner bg-green-100 text-green-700";
                    }
                    
                    // Radar Status
                    const radarStatus = document.getElementById('radar-last-update');
                    if (data.is_radar_running) {
                        radarStatus.textContent = `Running, Angle: ${data.current_angle}°`;
                        // Fetch Radar Data if running
                        fetchRadarData();
                    } else {
                        radarStatus.textContent = 'Stopped';
                    }
                })
                .catch(error => console.error('Error fetching status:', error));
        }
        
        // --- Radar Data Fetcher ---
        function fetchRadarData() {
             fetch('/radar_data')
                .then(response => response.json())
                .then(data => {
                    // data.data is the list of [angle, distance] pairs
                    updateRadarPlot(data.data, data.current_angle);
                })
                .catch(error => console.error('Error fetching radar data:', error));
        }

        // --- Event Listeners ---
        document.addEventListener('DOMContentLoaded', () => {
            // Linear Speed Slider
            document.getElementById('linear-speed-slider').addEventListener('mouseup', (e) => {
                sendLinearSpeedUpdate(e.target.value);
            });
            document.getElementById('linear-speed-slider').addEventListener('input', (e) => {
                document.getElementById('linear-speed-display').textContent = `${e.target.value}%`;
            });
            
            // Turn Speed Slider
            document.getElementById('turn-speed-slider').addEventListener('mouseup', (e) => {
                sendTurnSpeedUpdate(e.target.value);
            });
            document.getElementById('turn-speed-slider').addEventListener('input', (e) => {
                document.getElementById('turn-speed-display').textContent = `${e.target.value}%`;
            });

            // Set Test Index Button
            document.getElementById('set-index-btn').addEventListener('click', () => {
                const indexInput = document.getElementById('test-row-index-input');
                const newIndex = parseInt(indexInput.value);
                if (!isNaN(newIndex) && newIndex >= 0) {
                    sendTestIndexUpdate(newIndex);
                } else {
                    alert("Please enter a valid non-negative number for the index.");
                }
            });
            
            // Initial call and set interval for updates
            updateStatus();
            setInterval(updateStatus, 500); // Poll status every 0.5 seconds
        });
        
    </script>
</body>
</html>
"""
# ... (rest of the code) ...

if __name__ == '__main__':
    try:
        # Change host to '0.0.0.0' to listen on all public IPs, and port to 8000
        app.run(host='0.0.0.0', port=8000, threaded=True) 
    except KeyboardInterrupt:
        print("Cleaning up GPIO...")
        GPIO.cleanup()
        sys.exit()
    except Exception as e:
        print(f"An error occurred: {e}")
        GPIO.cleanup()
        sys.exit()