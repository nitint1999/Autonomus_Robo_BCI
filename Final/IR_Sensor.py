import RPi.GPIO as GPIO
import time

# --- Pin Definitions (BCM numbering) ---
# Define the GPIO pin for the LEFT sensor
LEFT_IR_PIN = 20
# Define the GPIO pin for the RIGHT sensor
RIGHT_IR_PIN = 21

# --- Setup ---
GPIO.setwarnings(False) 
GPIO.setmode(GPIO.BCM) 

# Set both sensor pins as inputs
GPIO.setup(LEFT_IR_PIN, GPIO.IN) 
GPIO.setup(RIGHT_IR_PIN, GPIO.IN) 

print("Dual IR Sensors Ready... (Ctrl+C to exit)")

# --- Main Loop ---
try:
    while True:
        # Read the state of each sensor
        left_state = GPIO.input(LEFT_IR_PIN)
        right_state = GPIO.input(RIGHT_IR_PIN)

        # Assuming LOW (0) means obstacle detected
        
        # Scenario 1: Obstacle on BOTH sides
        if left_state == GPIO.LOW and right_state == GPIO.LOW:
            print("🔴 Stop! Obstacle Detected on BOTH Left and Right.")
        
        # Scenario 2: Obstacle on LEFT side only
        elif left_state == GPIO.LOW and right_state == GPIO.HIGH:
            print("⬅️ Turning Right! Obstacle Detected on Left.")
            
        # Scenario 3: Obstacle on RIGHT side only
        elif left_state == GPIO.HIGH and right_state == GPIO.LOW:
            print("➡️ Turning Left! Obstacle Detected on Right.")
            
        # Scenario 4: No Obstacle (Clear path)
        else:
            print("✅ Clear Path. Moving Straight.")

        time.sleep(0.1) # Shorter delay for quicker reaction time

# --- Cleanup ---
except KeyboardInterrupt:
    print("\nProgram stopped by User")
    GPIO.cleanup()