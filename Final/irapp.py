from flask import Flask, render_template_string, Response
import cv2
import RPi.GPIO as GPIO
import time

app = Flask(__name__)

#Current 4WD concept

# Pin setup
IN1 = 26
IN2 = 19
IN3 = 13
IN4 = 6
en_a = 12
en_b = 5

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

for pin in [IN1, IN2, IN3, IN4, en_a, en_b]:
    GPIO.setup(pin, GPIO.OUT)

# Start PWM on enable pin
p = GPIO.PWM(en_a, 100)  # 1 kHz frequency
p.start(20)  # Start PWM with 100% duty cycle

# Start PWM on enable pin
q = GPIO.PWM(en_b, 100)  # 1 kHz frequency
q.start(20)  # Start PWM with 100% duty cycle

LEFT_IR_PIN = 21
RIGHT_IR_PIN = 20
# Start the robot


def forward():
    GPIO.output(IN1, True)
    GPIO.output(IN4, True)
    GPIO.output(IN2, False)
    GPIO.output(IN3, False)

    

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
    GPIO.output(IN2, False)
    GPIO.output(IN1, True)
    GPIO.output(IN3, False)
    GPIO.output(IN4, False)

def stop():
    for pin in [IN1, IN2, IN3, IN4]:
        GPIO.output(pin, False)

# Variable to track the current state
current_state = "Stopped"

# Camera setup (USB cam or Pi cam)
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

html = """
<!doctype html>
<title>Car Control</title>
<h1>4WD Control with Camera</h1>
<img src="/video_feed" width="640" height="480"><br>
<p>Current State: {}</p>  <!-- Display the current state -->
<form action="/forward"><button>Forward</button></form>
<form action="/backward"><button>Backward</button></form>
<form action="/left"><button>Left</button></form>
<form action="/right"><button>Right</button></form>
<form action="/stop"><button>Stop</button></form>
"""

@app.route("/")
def index():
    return render_template_string(html.format(current_state))  # Update state displayed

@app.route("/video_feed")
def video_feed():
    return Response(gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/forward")
def go_forward():
    global current_state  # Declare the variable as global to modify it
    forward()
    current_state = "Moving Forward"  # Update the current state
    print(current_state)
    return index()

@app.route("/backward")
def go_backward():
    global current_state
    backward()
    current_state = "Moving Backward"
    print(current_state)
    return index()

@app.route("/left")
def go_left():
    global current_state
    left()
    current_state = "Turning Left"
    print(current_state)
    return index()

@app.route("/right")
def go_right():
    global current_state
    right()
    current_state = "Turning Right"
    print(current_state)
    return index()

@app.route("/stop")
def go_stop():
    global current_state
    stop()
    current_state = "Stopped"
    print(current_state)
    return index()

try:
    move_forward()
    while True:
        left_sensor = GPIO.input(LEFT_IR_PIN)
        right_sensor = GPIO.input(RIGHT_IR_PIN)

        if left_sensor == 1:  # Obstacle detected on the left
            stop()
            right()
            forward()

        elif right_sensor == 1:  # Obstacle detected on the right
            stop()
            left()
            forward()
            
except KeyboardInterrupt:
    print("Exiting program.")

finally:
    stop()

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
    

