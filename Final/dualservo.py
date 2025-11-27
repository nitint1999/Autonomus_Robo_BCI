import time
from adafruit_servokit import ServoKit

# Initialize the ServoKit class for 16 channels
kit = ServoKit(channels=16)

# Define servo motor pins (0 and 1 in this example)
servo1 = kit.servo[0]
servo2 = kit.servo[3]

# Set the range for the servo motors
servo1.set_pulse_width_range(500, 2500)  # Adjust if necessary
servo2.set_pulse_width_range(500, 2500)  # Adjust if necessary

# Function to run servos continuously
def run_servos():
    try:
        while True:
            # Position the servos at their minimum and maximum angles
            for angle in range(0, 180, 1):  # Move from 0 to 180 degrees
                servo1.angle = angle
                servo2.angle = angle
                time.sleep(0.01)  # Adjust as needed for speed
            for angle in range(180, 0, -1):  # Move from 180 to 0 degrees
                servo1.angle = angle
                servo2.angle = angle
                time.sleep(0.01)
    except KeyboardInterrupt:
        # Stop the servos when interrupted
        servo1.angle = 90  # Set to neutral position
        servo2.angle = 90  # Set to neutral position
        print("Stopped servos.")

# Run the function
run_servos()

