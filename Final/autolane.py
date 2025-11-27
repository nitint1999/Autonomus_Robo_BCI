from flask import Flask, render_template_string, Response
import cv2
import RPi.GPIO as GPIO
import time
import threading
import sys 

app = Flask(__name__)

# --- Global State and Thread Setup ---
current_state = "Stopped"
# Flag indicates the robot is operating autonomously/manually and needs monitoring
is_moving = False 
monitor_thread = None 

# --- Motor Pin Setup (BCM) ---
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

# Setup GPIO Pins: Outputs for Motors, Inputs for Sensors
for pin in [IN1, IN2, IN3, IN4, en_a, en_b]:
    GPIO.setup(pin, GPIO.OUT)

GPIO.setup(LEFT_IR_PIN, GPIO.IN) 
GPIO.setup(RIGHT_IR_PIN, GPIO.IN) 

# Start PWM on enable pins
PWM_FREQ = 100 # Hz
DUTY_CYCLE = 20 # % (Adjust for speed control)
p = GPIO.PWM(en_a, PWM_FREQ)
p.start(DUTY_CYCLE)

q = GPIO.PWM(en_b, PWM_FREQ)
q.start(DUTY_CYCLE)

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
    # Pivot turn or turn right movement
    GPIO.output(IN1, False)
    GPIO.output(IN2, False)
    GPIO.output(IN3, False)
    GPIO.output(IN4, True)
    
def left():
    # Pivot turn or turn left movement
    GPIO.output(IN1, True)
    GPIO.output(IN2, False)
    GPIO.output(IN3, False)
    GPIO.output(IN4, False)

# --- IR Sensor Reading & Monitoring Functions ---

def get_sensor_status():
    """Reads the state of both IR sensors for display."""
    left_state = GPIO.input(LEFT_IR_PIN)
    right_state = GPIO.input(RIGHT_IR_PIN)
    
    # LOW (0) typically means obstacle detected
    left_status = "DETECTED" if left_state == GPIO.LOW else "Clear"
    right_status = "DETECTED" if right_state == GPIO.LOW else "Clear"
    
    return f"Left: {left_status} | Right: {right_status}"

# **The Continuous Monitoring Thread Function for Stabilized Avoidance**
def obstacle_monitor():
    global current_state, is_moving
    
    while True:
        if is_moving:
            left_state = GPIO.input(LEFT_IR_PIN)
            right_state = GPIO.input(RIGHT_IR_PIN)
            
            # Scenario 1: Both Blocked -> STOP (Highest Priority)
            if left_state == GPIO.LOW and right_state == GPIO.LOW:
                stop_motors()
                current_state = "🚨 AUTO STOP: Obstacle on Both Sides"
                is_moving = False  # Stop continuous monitoring until new command
                print(current_state)
            
            # Scenario 2: Left Blocked, Right Clear -> TURN RIGHT
            elif left_state == GPIO.LOW and right_state == GPIO.HIGH:
                # Sequence: Stop -> Pause -> Turn -> Pause -> Resume Forward
                stop_motors()         
                time.sleep(0.05)      
                right()               
                time.sleep(0.3)       # Duration of the avoidance turn
                stop_motors()         
                forward()             # Resume Forward movement
                current_state = "➡️ AUTO AVOID: Turned Right & Resumed"
                print(current_state)

            # Scenario 3: Right Blocked, Left Clear -> TURN LEFT
            elif left_state == GPIO.HIGH and right_state == GPIO.LOW:
                # Sequence: Stop -> Pause -> Turn -> Pause -> Resume Forward
                stop_motors()         
                time.sleep(0.05)      
                left()                
                time.sleep(0.3)       # Duration of the avoidance turn
                stop_motors()         
                forward()             # Resume Forward movement
                current_state = "⬅️ AUTO AVOID: Turned Left & Resumed"
                print(current_state)
            
        time.sleep(0.05) # Check frequency: 50ms (20 times per second)

# --- Initialize and Start the Monitoring Thread ---
# daemon=True ensures the thread exits when the main program (Flask) exits
monitor_thread = threading.Thread(target=obstacle_monitor, daemon=True)
monitor_thread.start()

# --- Camera Setup ---
try:
    camera = cv2.VideoCapture(0)
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
    """Generates frames for the video stream."""
    if not camera:
        # Yield a placeholder frame if camera failed
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

# --- HTML Template ---
html = """
<!doctype html>
<title>4WD Control</title>
<h1>4WD Control with Camera and IR Sensors</h1>
<p>Pinout: Motor Pins (26, 19, 13, 6, 12, 5) | Sensor Pins (BCM 20, 21)</p>
<hr>
<img src="/video_feed" width="640" height="480"><br>
<h3>Movement Status: {{ current_state }}</h3>  
<h3>Sensor Status: {{ sensor_status }}</h3>
<form action="/forward"><button>Forward</button></form>
<form action="/backward"><button>Backward</button></form>
<form action="/left"><button>Left</button></form>
<form action="/right"><button>Right</button></form>
<form action="/stop"><button>Stop</button></form>
"""

# --- Flask Routes ---
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
    current_state = "Moving Forward (Auto-Avoidance ON)"
    is_moving = True  # Start autonomous monitoring
    print(current_state)
    return index()

@app.route("/backward")
def go_backward():
    global current_state, is_moving
    backward()
    current_state = "Moving Backward (Auto-Avoidance ON)"
    is_moving = True  # Start autonomous monitoring
    print(current_state)
    return index()

@app.route("/left")
def go_left():
    global current_state, is_moving
    left()
    current_state = "Turning Left (Auto-Avoidance ON)"
    is_moving = True  # Start autonomous monitoring
    print(current_state)
    return index()

@app.route("/right")
def go_right():
    global current_state, is_moving
    right()
    current_state = "Turning Right (Auto-Avoidance ON)"
    is_moving = True  # Start autonomous monitoring
    print(current_state)
    return index()

@app.route("/stop")
def go_stop():
    global current_state, is_moving
    stop_motors()
    current_state = "Stopped"
    is_moving = False # Stop autonomous monitoring
    print(current_state)
    return index()

if __name__ == "__main__":
    try:
        print("Starting Flask web server on http://0.0.0.0:5000")
        app.run(host="0.0.0.0", port=5000, threaded=True) 
    except KeyboardInterrupt:
        print("\nServer stopped by user.")
    except Exception as e:
        print(f"\nAn error occurred: {e}", file=sys.stderr)
    finally:
        # Cleanup ensures all GPIO pins are reset when the program exits
        GPIO.cleanup()
        if camera:
             camera.release()