from flask import Flask, render_template_string, Response, jsonify
import cv2
import RPi.GPIO as GPIO
import time
import threading
import sys
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
SERVO_DELAY_S = 0.03 # 30 milliseconds delay between angle steps
current_angle = 90 # Instantaneous angle for the scanning line

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

# Ultrasonic Sensor Setup (BCM)
TRIG_PIN = 25 
ECHO_PIN = 24 

# Servo motor setup
kit = ServoKit(channels=16)
servo1 = kit.servo[0]
servo1.set_pulse_width_range(500, 2500) 

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Setup GPIO Pins: Outputs for Motors, Inputs for Sensors
for pin in [IN1, IN2, IN3, IN4, en_a, en_b, TRIG_PIN]:
    GPIO.setup(pin, GPIO.OUT)

for pin in [LEFT_IR_PIN, RIGHT_IR_PIN, ECHO_PIN]:
    GPIO.setup(pin, GPIO.IN) 

# --- Speed Constants (Unchanged) ---
FULL_DUTY_CYCLE = 20 
TURN_DUTY_CYCLE = 15 
PWM_FREQ = 100 

p = GPIO.PWM(en_a, PWM_FREQ)
p.start(0) 

q = GPIO.PWM(en_b, PWM_FREQ)
q.start(0) 

# --- Motor Control Functions ---
def set_speed(duty_cycle):
    p.ChangeDutyCycle(duty_cycle)
    q.ChangeDutyCycle(duty_cycle)

def forward():
    set_speed(FULL_DUTY_CYCLE)
    GPIO.output(IN1, True)
    GPIO.output(IN2, False)
    GPIO.output(IN3, False) 
    GPIO.output(IN4, True)

def stop_motors():
    set_speed(0)
    for pin in [IN1, IN2, IN3, IN4]:
        GPIO.output(pin, False)

def backward():
    set_speed(FULL_DUTY_CYCLE)
    GPIO.output(IN1, False)
    GPIO.output(IN2, True)
    GPIO.output(IN3, True)
    GPIO.output(IN4, False)

def right():
    set_speed(TURN_DUTY_CYCLE)
    GPIO.output(IN1, True)
    GPIO.output(IN2, False)
    GPIO.output(IN3, True) 
    GPIO.output(IN4, False)
    
def left():
    set_speed(TURN_DUTY_CYCLE)
    GPIO.output(IN1, False)
    GPIO.output(IN2, True)
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

# --- Radar Function (Updates current_angle for the scanning line) ---
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
                current_angle = angle # Update instantaneous angle
            
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
                current_angle = angle # Update instantaneous angle
            
        # Safely update the shared global radar data
        with state_lock:
            if is_radar_running: 
                radar_data = current_data
        
        # Pause before the next full sweep
        time.sleep(0.5)

    servo1.angle = 90
    with state_lock:
        current_angle = 90 # Reset angle on stop
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
                
                if left_state == GPIO.LOW:
                    right()
                    new_state = "➡️ AUTO AVOID: Turned Right & Resumed"
                else:
                    left()
                    new_state = "⬅️ AUTO AVOID: Turned Left & Resumed"

                time.sleep(0.3)
                stop_motors()
                forward()
                
                with state_lock:
                    current_state = new_state
                print(current_state)
                
        time.sleep(0.05) 

# --- Initialize and Start Threads ---
monitor_thread = threading.Thread(target=obstacle_monitor, daemon=True)
monitor_thread.start()

# Start the Radar Thread initially
start_radar_thread()

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
        
    return render_template_string(html, 
                                 current_state=current_state_locked, 
                                 sensor_status=sensor_status,
                                 FULL_DUTY_CYCLE=FULL_DUTY_CYCLE,
                                 TURN_DUTY_CYCLE=TURN_DUTY_CYCLE)

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
    """Returns the current state, sensor status, and radar running status as JSON."""
    with state_lock:
        state = current_state
        moving = is_moving
        radar_running = is_radar_running 
    
    sensor_data_string = get_sensor_status()
    
    return jsonify({
        "state": state,
        "is_moving": moving,
        "sensor_status": sensor_data_string,
        "is_radar_running": radar_running 
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
        current_state = f"Moving Forward (Speed: {FULL_DUTY_CYCLE}%)"
        is_moving = True 
    return index()

@app.route("/backward", methods=['POST'])
def go_backward():
    global current_state, is_moving
    backward()
    with state_lock:
        current_state = f"Moving Backward (Speed: {FULL_DUTY_CYCLE}%)"
        is_moving = True 
    return index()

@app.route("/left", methods=['POST'])
def go_left():
    global current_state, is_moving
    left()
    with state_lock:
        current_state = f"Turning Left (Speed: {TURN_DUTY_CYCLE}%)"
        is_moving = True 
    return index()

@app.route("/right", methods=['POST'])
def go_right():
    global current_state, is_moving
    right()
    with state_lock:
        current_state = f"Turning Right (Speed: {TURN_DUTY_CYCLE}%)"
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

# --- HTML Template (Includes all JS/CSS for 180 degree semi-circle radar with scanner line) ---
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
            /* Forces the container height to be exactly half its width for the semi-circle */
            padding-bottom: 50%; 
            height: 0;
            background-color: #1a1a1a;
            border-radius: 0.5rem;
            overflow: hidden; /* CRITICAL: Hides the unwanted bottom half of the circular canvas */
        }
        .radar-container canvas {
            position: absolute;
            width: 100% !important;
            height: 100% !important;
        }
        .chart-grid-green .chartjs-render-monitor {
            background-color: #1a1a1a !important; 
        }
    </style>
</head>
<body class="bg-gray-100 p-4 sm:p-8">

    <div class="max-w-4xl mx-auto bg-white shadow-xl rounded-xl p-6">
        <h1 class="text-3xl font-bold text-gray-800 mb-4 text-center">
            4WD Robot Controller & Radar
        </h1>
        <p class="text-sm text-gray-500 mb-6 text-center">
            Motor Speed: Forward/Backward ({{ FULL_DUTY_CYCLE }}%) | Turns ({{ TURN_DUTY_CYCLE }}%)
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
        </div>

        <div class="flex flex-col items-center space-y-4 mb-8 border-b pb-8">
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

        // --- Chart.js Plugin to Draw Scanning Line (The line that tracks the servo angle) ---
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
                    
                    // Map the 0-180 degree angle to the correct radial position
                    // The formula Math.PI * (1 - (angle/180)) maps: 0->PI, 90->PI/2, 180->0.
                    const mappedAngleRad = Math.PI * (1 - (currentScanAngle / 180));
                    
                    const xEnd = center.x + maxRadius * Math.cos(mappedAngleRad);
                    const yEnd = center.y - maxRadius * Math.sin(mappedAngleRad); // Y axis is inverted

                    ctx.save();
                    ctx.beginPath();
                    ctx.strokeStyle = 'rgba(0, 255, 0, 0.8)'; // Neon green line
                    ctx.lineWidth = 2;
                    ctx.setLineDash([5, 5]); // Dashed line
                    ctx.moveTo(center.x, center.y);
                    ctx.lineTo(xEnd, yEnd);
                    ctx.stroke();
                    ctx.restore();
                }
            }
        };

        Chart.register(scannerLinePlugin);


        // --- Status Update Function ---
        function updateStatus() {
            fetch('/status')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('movement-status').textContent = data.state;
                    document.getElementById('sensor-status').textContent = data.sensor_status;
                    
                    const radarStatusElement = document.getElementById('radar-last-update');
                    const startBtn = document.getElementById('start-radar-btn');
                    const stopBtn = document.getElementById('stop-radar-btn');
                    
                    if (radarChart && radarChart.options.plugins.scannerLine) {
                        radarChart.options.plugins.scannerLine.enabled = data.is_radar_running;
                        if (!data.is_radar_running) {
                            currentScanAngle = 90; 
                            radarChart.update('none'); 
                        }
                    }

                    if (data.is_radar_running) {
                        radarStatusElement.textContent = 'Running';
                        radarStatusElement.classList.remove('text-red-700');
                        radarStatusElement.classList.add('text-purple-700');
                        
                        startBtn.disabled = true;
                        startBtn.classList.replace('bg-green-500', 'bg-gray-400'); 
                        stopBtn.disabled = false;
                        stopBtn.classList.replace('bg-gray-400', 'bg-red-500'); 
                    } else {
                        radarStatusElement.textContent = 'STOPPED (Ready to start)';
                        radarStatusElement.classList.remove('text-purple-700');
                        radarStatusElement.classList.add('text-red-700');
                        
                        startBtn.disabled = false;
                        startBtn.classList.replace('bg-gray-400', 'bg-green-500'); 
                        stopBtn.disabled = true;
                        stopBtn.classList.replace('bg-red-500', 'bg-gray-400'); 
                    }

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


        // --- Radar Plotting Functions (Ensures 180 degree view) ---
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
                    maintainAspectRatio: false, // Disabled to allow CSS to control aspect ratio
                    backgroundColor: '#1a1a1a', 
                    layout: {
                         padding: {
                             bottom: 0 // Prevents extra spacing below the semi-circle
                         }
                    },
                    scales: {
                        r: {
                            min: 0,
                            max: MAX_DISTANCE,
                            radius: '100%', // Forces scale to utilize available horizontal space
                            display: true,
                            
                            // *** CRITICAL CHANGE FOR 180 DEGREE VIEW ***
                            circumference: 180, // Force the scale to only use 180 degrees
                            
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
                            // Define the 180 degree range: 180 (left) to 0 (right)
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
                    
                    // Update the global instantaneous angle
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
                        
                        fetch('/status').then(res => res.json()).then(status => {
                             if (status.is_radar_running) {
                                radarStatusElement.textContent = `Running, last data @ ${new Date().toLocaleTimeString()}`;
                             }
                        });


                    } else {
                        if (radarChart.data.labels.length > 0 || radarChart.options.plugins.scannerLine.enabled) {
                             radarChart.data.labels = [];
                             radarChart.data.datasets[0].data = [];
                             radarChart.update('none'); 
                        }
                    }
                })
                .catch(error => console.error('Error fetching radar data:', error));
        }

        document.addEventListener('DOMContentLoaded', () => {
            initializeRadarChart();
            setInterval(updateStatus, 500); 
            // 50ms polling rate for smooth scanner line movement
            setInterval(updateRadarPlot, 50); 
            updateStatus();
            updateRadarPlot();
        });
    </script>
</body>
</html>
"""

if __name__ == "__main__":
    try:
        print("Starting Flask web server on http://0.0.0.0:5000")
        app.run(host="0.0.0.0", port=5000, threaded=True) 
    except KeyboardInterrupt:
        print("\nServer stopped by user.")
    except Exception as e:
        print(f"\nAn error occurred: {e}", file=sys.stderr)
    finally:
        GPIO.cleanup()
        if camera:
            camera.release()
