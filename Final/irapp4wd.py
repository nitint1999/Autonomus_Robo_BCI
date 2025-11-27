from flask import Flask, render_template_string, Response
import cv2
import RPi.GPIO as GPIO
import time
import threading

app = Flask(__name__)

# --- Global State and Thread Setup ---
current_state = "Stopped"
is_moving = False  # Flag to tell the monitor thread when to check sensors
monitor_thread = None 

# --- Motor Pin Setup ---
IN1 = 26
IN2 = 19
IN3 = 13
IN4 = 6
en_a = 12
en_b = 5

# --- IR Sensor Pin Setup (BCM) ---
LEFT_IR_PIN = 20
RIGHT_IR_PIN = 21

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Setup GPIO Pins
for pin in [IN1, IN2, IN3, IN4, en_a, en_b, LEFT_IR_PIN, RIGHT_IR_PIN]:
    if pin in [IN1, IN2, IN3, IN4, en_a, en_b]:
        GPIO.setup(pin, GPIO.OUT)
    else:
        GPIO.setup(pin, GPIO.IN) 

# Start PWM on enable pins
p = GPIO.PWM(en_a, 100)
p.start(20)

q = GPIO.PWM(en_b, 100)
q.start(20)

# --- Motor Control Functions ---
def forward():
    GPIO.output(IN1, True)
    GPIO.output(IN4, True)
    GPIO.output(IN2, False)
    GPIO.output(IN3, False)

def stop_motors():
    for pin in [IN1, IN2, IN3, IN4]:
        GPIO.output(pin, False)

def backward():
    GPIO.output(IN2, True)
    GPIO.output(IN3, True)
    GPIO.output(IN1, False)
    GPIO.output(IN4, False)

def right():
    GPIO.output(IN1, False)
    GPIO.output(IN2, False)
    GPIO.output(IN3, False)
    GPIO.output(IN4, True)
    
def left():
    GPIO.output(IN1, True)
    GPIO.output(IN2, False)
    GPIO.output(IN3, False)
    GPIO.output(IN4, False)

# --- IR Sensor Reading & Monitoring Functions ---
def check_for_obstacle():
    """Returns True if any obstacle is detected (LOW signal on either pin)."""
    left_state = GPIO.input(LEFT_IR_PIN)
    right_state = GPIO.input(RIGHT_IR_PIN)
    return left_state == GPIO.LOW or right_state == GPIO.LOW

def get_sensor_status():
    """Reads the state of both IR sensors for display."""
    left_state = GPIO.input(LEFT_IR_PIN)
    right_state = GPIO.input(RIGHT_IR_PIN)
    
    left_status = "DETECTED" if left_state == GPIO.LOW else "Clear"
    right_status = "DETECTED" if right_state == GPIO.LOW else "Clear"
    
    return f"Left: {left_status} | Right: {right_status}"

# **The Continuous Monitoring Thread Function**
def obstacle_monitor():
    global current_state, is_moving
    
    while True:
        # Check only if a movement command is active
        if is_moving:
            if check_for_obstacle():
                stop_motors()
                # Update the state flags and variables
                is_moving = False
                current_state = "🚨 AUTO STOP: Obstacle Detected!"
                print(current_state)
            
        time.sleep(0.05) # Check every 50ms

# --- Initialize and Start the Monitoring Thread ---
monitor_thread = threading.Thread(target=obstacle_monitor, daemon=True)
monitor_thread.start()

# --- Camera Setup ---
camera = cv2.VideoCapture(0)

def gen_frames():
    while True:
        success, frame = camera.read()
        if not success:
            break
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

# --- HTML Template (Unchanged) ---
html = """
<!doctype html>
<title>4WD Control</title>
<h1>4WD Control with Camera and IR Sensors</h1>
<img src="/video_feed" width="640" height="480"><br>
<h3>Movement Status: {{ current_state }}</h3>  
<h3>Sensor Status: {{ sensor_status }}</h3>
<form action="/forward"><button>Forward</button></form>
<form action="/backward"><button>Backward</button></form>
<form action="/left"><button>Left</button></form>
<form action="/right"><button>Right</button></form>
<form action="/stop"><button>Stop</button></form>
"""

# --- Flask Routes (All movement routes set is_moving=True) ---
@app.route("/")
def index():
    sensor_status = get_sensor_status()
    return render_template_string(html, current_state=current_state, sensor_status=sensor_status)

@app.route("/video_feed")
def video_feed():
    return Response(gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/forward")
def go_forward():
    global current_state, is_moving
    forward()
    current_state = "Moving Forward"
    is_moving = True  # Start continuous checking
    print(current_state)
    return index()

@app.route("/backward")
def go_backward():
    global current_state, is_moving
    backward()
    current_state = "Moving Backward"
    is_moving = True  # Start continuous checking
    print(current_state)
    return index()

@app.route("/left")
def go_left():
    global current_state, is_moving
    left()
    current_state = "Turning Left"
    is_moving = True  # Start continuous checking
    print(current_state)
    return index()

@app.route("/right")
def go_right():
    global current_state, is_moving
    right()
    current_state = "Turning Right"
    is_moving = True  # Start continuous checking
    print(current_state)
    return index()

@app.route("/stop")
def go_stop():
    global current_state, is_moving
    stop_motors()
    current_state = "Stopped"
    is_moving = False # Stop continuous checking
    print(current_state)
    return index()

if __name__ == "__main__":
    try:
        # Run Flask with threading enabled
        app.run(host="0.0.0.0", port=5000, threaded=True) 
    finally:
        GPIO.cleanup()