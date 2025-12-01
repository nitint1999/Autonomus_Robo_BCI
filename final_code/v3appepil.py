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

app = Flask(__name__)

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

# Speed Constants (Defaults)
INITIAL_LINEAR_SPEED = 20    # Initial value for the linear speed slider
INITIAL_TURN_SPEED = 20      # Initial value for the turn speed slider

# DYNAMIC SPEED VARIABLES (Controlled by Sliders)
current_duty_cycle = INITIAL_LINEAR_SPEED    
current_turn_duty_cycle = INITIAL_TURN_SPEED 
PWM_FREQ = 100 

# --- SEIZURE DETECTION GLOBALS ---
SEIZURE_LED_PIN_BCM = 27  # <--- CORRECTED PIN: BCM pin 27 (Physical Pin 13)
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
LEFT_IR_PIN = 20
RIGHT_IR_PIN = 21
TRIG_PIN = 25 
ECHO_PIN = 24 

# Servo motor setup
kit = ServoKit(channels=16)
servo1 = kit.servo[0]
servo1.set_pulse_width_range(500, 2500) 

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Setup GPIO Pins (Includes the SEIZURE_LED_PIN_BCM)
for pin in [IN1, IN2, IN3, IN4, en_a, en_b, TRIG_PIN, SEIZURE_LED_PIN_BCM]:
    GPIO.setup(pin, GPIO.OUT)

for pin in [LEFT_IR_PIN, RIGHT_IR_PIN, ECHO_PIN]:
    GPIO.setup(pin, GPIO.IN) 

p = GPIO.PWM(en_a, PWM_FREQ)
p.start(0) 

q = GPIO.PWM(en_b, PWM_FREQ)
q.start(0) 

# --- Motor Control Functions ---
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

# --- Ultrasonic Sensor Function ---
def read_distance():
    GPIO.output(TRIG_PIN, False)
    time.sleep(0.000002) 

    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)

    pulse_start = time.time()
    pulse_end = time.time()
    
    timeout_start = time.time()
    while GPIO.input(ECHO_PIN) == 0 and (time.time() - timeout_start) < 0.05:
        pulse_start = time.time()

    timeout_start = time.time()
    while GPIO.input(ECHO_PIN) == 1 and (time.time() - timeout_start) < 0.05:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150

    distance = round(distance, 2)
    if distance > 400 or distance < 2:
        return 0 
    
    return distance 

# --- Radar Function ---
def radar():
    """Runs the servo sweep and collects angle/distance data until stopped."""
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
            distance = read_distance()
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
            distance = read_distance()
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


# --- IR Sensor Reading & Monitoring Functions ---
def get_sensor_status():
    left_state = GPIO.input(LEFT_IR_PIN)
    right_state = GPIO.input(RIGHT_IR_PIN)
    
    left_status = "DETECTED" if left_state == GPIO.LOW else "Clear"
    right_status = "DETECTED" if right_state == GPIO.LOW else "Clear"
    
    return f"Left: {left_status} | Right: {right_status}"

def obstacle_monitor():
    global current_state, is_moving
    
    while True:
        with state_lock:
            moving_status = is_moving

        if moving_status:
            left_state = GPIO.input(LEFT_IR_PIN)
            right_state = GPIO.input(RIGHT_IR_PIN)
            
            if left_state == GPIO.LOW and right_state == GPIO.LOW:
                stop_motors()
                with state_lock:
                    current_state = "🚨 AUTO STOP: Obstacle on Both Sides"
                    is_moving = False
                print(current_state)
            
            elif left_state == GPIO.LOW or right_state == GPIO.LOW:
                stop_motors()
                time.sleep(0.05)
                
                # Auto avoidance uses the current turn speed
                if left_state == GPIO.LOW:
                    right() # Obstacle left -> turn right
                    new_state = f"➡️ AUTO AVOID: Turned Right ({current_turn_duty_cycle}%) & Resumed"
                else:
                    left()  # Obstacle right -> turn left
                    new_state = f"⬅️ AUTO AVOID: Turned Left ({current_turn_duty_cycle}%) & Resumed"

                time.sleep(0.3)
                stop_motors()
                forward() # Resume forward movement with current_duty_cycle
                
                with state_lock:
                    current_state = new_state
                print(current_state)
                
        time.sleep(0.05) 

# -----------------------------------------------------
# --- SEIZURE DETECTION THREAD FUNCTION ---
def seizure_detection_monitor():
    """Continuously uses the loaded model and data to check for seizures, toggling the LED on detection."""
    global IS_SEIZURE_DETECTED, IS_SEIZURE_MONITORING
    
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
    current_n = TEST_ROW_INDEX 

    while IS_SEIZURE_MONITORING:
        try:
            # 2. Acquire Data Point (Simulation: Select fixed row 12)
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
                print("🚨 SEIZURE ALERT: Detected! Toggling LED.") 
                
            else:
                with state_lock:
                    IS_SEIZURE_DETECTED = False
                    
                GPIO.output(SEIZURE_LED_PIN_BCM, False)
                time.sleep(1) # Check again after 1 second
                
        except IndexError:
            # If you want to cycle through rows:
            # current_n = (current_n + 1) % len(x_data)
            # If you want to stick to one test row:
            current_n = TEST_ROW_INDEX
            time.sleep(2)
        except Exception as e:
            print(f"Seizure Detection Error during loop: {e}")
            time.sleep(5) 

    print("Seizure Detection Monitor stopped.")
# -----------------------------------------------------


# --- Initialize and Start Threads ---
monitor_thread = threading.Thread(target=obstacle_monitor, daemon=True)
monitor_thread.start()

start_radar_thread()

# --- Initialize and Start NEW Seizure Detection Thread ---
SEIZURE_THREAD = threading.Thread(target=seizure_detection_monitor, daemon=True)
SEIZURE_THREAD.start()
# ---------------------------------------------------------

# --- Camera Setup ---
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

# --- Flask Routes ---
@app.route("/")
def index():
    sensor_status = get_sensor_status()
    with state_lock:
        current_state_locked = current_state
        current_duty_cycle_locked = current_duty_cycle
        current_turn_duty_cycle_locked = current_turn_duty_cycle
        seizure_status = IS_SEIZURE_DETECTED 
        
    return render_template_string(html, 
                                 current_state=current_state_locked, 
                                 sensor_status=sensor_status,
                                 LINEAR_SPEED=current_duty_cycle_locked, 
                                 TURN_SPEED=current_turn_duty_cycle_locked,
                                 IS_SEIZURE_DETECTED=seizure_status)

# Route to handle LINEAR speed changes from the slider
@app.route("/set_linear_speed", methods=['POST'])
def set_linear_motor_speed():
    global current_duty_cycle
    try:
        data = request.json
        new_speed = int(data.get('speed', current_duty_cycle)) 
        
        new_speed = max(0, min(100, new_speed))
        
        with state_lock:
            current_duty_cycle = new_speed
            
            # If the robot is currently moving forward/backward, update the speed immediately
            if "Forward" in current_state or "Backward" in current_state:
                set_speed(current_duty_cycle)
                
        return jsonify({"success": True, "new_speed": current_duty_cycle})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 400

# NEW ROUTE: Route to handle TURN speed changes from the slider
@app.route("/set_turn_speed", methods=['POST'])
def set_turn_motor_speed():
    global current_turn_duty_cycle
    try:
        data = request.json
        new_speed = int(data.get('speed', current_turn_duty_cycle)) 
        
        new_speed = max(0, min(100, new_speed))
        
        with state_lock:
            current_turn_duty_cycle = new_speed
            
            # If the robot is currently turning, update the speed immediately
            if "Turning Left" in current_state or "Turning Right" in current_state or "AUTO AVOID" in current_state:
                pass 
                
        return jsonify({"success": True, "new_speed": current_turn_duty_cycle})
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
    """Returns the current state, sensor status, and running statuses as JSON, including seizure status."""
    with state_lock:
        state = current_state
        moving = is_moving
        radar_running = is_radar_running 
        linear_speed = current_duty_cycle
        turn_speed = current_turn_duty_cycle 
        seizure_status = IS_SEIZURE_DETECTED 
    
    sensor_data_string = get_sensor_status()
    
    return jsonify({
        "state": state,
        "is_moving": moving,
        "sensor_status": sensor_data_string,
        "is_radar_running": radar_running,
        "linear_speed": linear_speed,
        "turn_speed": turn_speed,
        "is_seizure_detected": seizure_status 
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

# --- HTML Template (Includes Seizure Status and JS Updates) ---
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
                IR Sensor Status: <span id="sensor-status" class="font-normal text-green-700">{{ sensor_status }}</span>
            </h3>
            
            <h3 class="text-lg font-semibold p-2 rounded-lg shadow-inner" id="seizure-status-box">
                🧠 Seizure Monitor: <span id="seizure-status-display" class="font-normal">Monitoring...</span>
            </h3>
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
        
        // --- Status Update Function ---
        function updateStatus() {
            fetch('/status')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('movement-status').textContent = data.state;
                    document.getElementById('sensor-status').textContent = data.sensor_status;
                    
                    // Update speed displays...
                    document.getElementById('linear-speed-display').textContent = `${data.linear_speed}%`;
                    document.getElementById('linear-speed-slider').value = data.linear_speed;
                    document.getElementById('linear-speed-display-summary').textContent = `${data.linear_speed}%`;

                    document.getElementById('turn-speed-display').textContent = `${data.turn_speed}%`;
                    document.getElementById('turn-speed-slider').value = data.turn_speed;
                    document.getElementById('turn-speed-display-summary').textContent = `${data.turn_speed}%`;

                    // --- SEIZURE STATUS UPDATE LOGIC ---
                    const seizureDisplay = document.getElementById('seizure-status-display');
                    const seizureBox = document.getElementById('seizure-status-box');
                    
                    if (data.is_seizure_detected) {
                        seizureDisplay.textContent = "🚨 SEIZURE DETECTED! (LED ACTIVE)";
                        // Alert styling: red background, white text
                        seizureBox.className = "text-lg font-semibold p-2 rounded-lg shadow-inner bg-red-600 text-white";
                    } else {
                        seizureDisplay.textContent = "System Clear";
                        // Normal monitoring styling: light yellow background, dark text
                        seizureBox.className = "text-lg font-semibold p-2 rounded-lg shadow-inner bg-yellow-50 text-gray-800";
                    }
                    // --- END SEIZURE STATUS LOGIC ---
                })
                .catch(error => console.error('Error fetching status:', error));
        }

        // --- Point Color Gradient Function ---
        function getPointColor(distance) {
            if (distance === 0 || distance >= MAX_DISTANCE) {
                return 'rgba(0, 0, 0, 0)'; 
            } else if (distance <= 30) {
                return 'rgb(255, 0, 0)'; 
            } else if (distance <= 70) {
                return 'rgb(255, 165, 0)'; 
            } else if (distance <= 120) {
                return 'rgb(255, 255, 0)'; 
            } else {
                return 'rgb(0, 255, 0)'; 
            }
        }


        // --- Radar Plotting Functions ---
        function initializeRadarChart() {
            const ctx = document.getElementById('radarChart').getContext('2d');
            radarChart = new Chart(ctx, {
                type: 'polarArea',
                data: {
                    labels: [], 
                    datasets: [{
                        label: 'Distance (cm)',
                        data: [],
                        borderWidth: 1,
                        pointRadius: 6, 
                        pointHoverRadius: 8,
                        pointStyle: 'circle', 
                    }]
                },
                options: {
                    responsive: true,
                    maintainAspectRatio: false, 
                    backgroundColor: '#1a1a1a', 
                    layout: {
                         padding: {
                             bottom: 0 
                         }
                    },
                    scales: {
                        r: {
                            min: 0,
                            max: MAX_DISTANCE,
                            radius: '100%', 
                            display: true,
                            
                            circumference: 180, // Forces the scale grid to 180 degrees
                            
                            grid: {
                                color: 'rgba(0, 255, 0, 0.4)',
                            },
                            angleLines: {
                                color: 'rgba(0, 255, 0, 0.4)'
                            },
                            ticks: {
                                stepSize: 50,
                                color: 'rgb(0, 255, 0)',
                                backdropColor: 'rgba(26, 26, 26, 0.8)', 
                            },
                            pointLabels: {
                                display: true,
                                color: 'rgb(0, 255, 0)', 
                                font: {
                                    size: 10
                                }
                            },
                            suggestedMin: 0,
                            suggestedMax: 180,
                            startAngle: 180, 
                            endAngle: 0,      
                        }
                    },
                    plugins: {
                        legend: { display: false },
                        tooltip: {
                            backgroundColor: 'rgba(0, 0, 0, 0.8)', 
                            titleColor: 'rgb(0, 255, 0)',
                            bodyColor: 'rgb(255, 255, 255)',
                            callbacks: {
                                label: (context) => {
                                    return `Distance: ${context.raw} cm`;
                                }
                            }
                        },
                        scannerLine: {
                            enabled: true
                        }
                    },
                    animation: false, 
                }
            });
        }

        function updateRadarPlot() {
            fetch('/radar_data')
                .then(response => response.json())
                .then(result => {
                    const radarStatusElement = document.getElementById('radar-last-update');
                    const data = result.data || [];
                    
                    if (result.current_angle !== undefined) {
                        currentScanAngle = result.current_angle;
                    }

                    if (data.length > 0) {
                        const labels = [];
                        const distances = [];
                        const pointColors = [];
                        
                        data.forEach(item => {
                            const distance = item[1];
                            labels.push(item[0].toString());
                            distances.push(distance);
                            pointColors.push(getPointColor(distance));
                        });

                        radarChart.data.labels = labels;
                        radarChart.data.datasets[0].data = distances;
                        radarChart.data.datasets[0].pointBackgroundColor = pointColors;
                        radarChart.data.datasets[0].pointBorderColor = 'rgba(255, 255, 255, 0.5)'; 
                        radarChart.data.datasets[0].backgroundColor = 'rgba(0, 255, 0, 0.05)'; 
                        radarChart.data.datasets[0].borderColor = 'rgb(0, 255, 0)';

                        radarChart.update('none'); 
                    }
                })
                .catch(error => console.error('Error fetching radar data:', error));
        }


        // --- Event Listeners and Initializers ---

        window.onload = function() {
            initializeRadarChart();

            // Set up event listener for the Linear Speed Slider
            document.getElementById('linear-speed-slider').addEventListener('input', function() {
                sendLinearSpeedUpdate(this.value);
            });
            
            // Set up event listener for the Turn Speed Slider
            document.getElementById('turn-speed-slider').addEventListener('input', function() {
                sendTurnSpeedUpdate(this.value);
            });

            // Start periodic updates for Status and Radar
            setInterval(updateStatus, 500); // Update status every 0.5 seconds
            setInterval(updateRadarPlot, 100); // Update radar plot frequently
        };

    </script>
</body>
</html>
"""

if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=8000, threaded=True, debug=False)
    except KeyboardInterrupt:
        print("Stopping robot and cleaning up GPIO...")
    finally:
        stop_motors()
        GPIO.cleanup()
        print("GPIO cleanup complete.")
        sys.exit(0)