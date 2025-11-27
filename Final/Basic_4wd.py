import RPi.GPIO as GPIO
import time

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

def forward():
    GPIO.output(IN1, True)
    GPIO.output(IN4, True)
    GPIO.output(IN2, False)
    GPIO.output(IN3, False)

def reverse():
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
    GPIO.output(IN2, True)
    GPIO.output(IN1, False)
    GPIO.output(IN3, False)
    GPIO.output(IN4, False)

def stop():
    for pin in [IN1, IN2, IN3, IN4]:
        GPIO.output(pin, False)

try:
    while True:
        cmd = input("Enter command (f=forward, b=back, l=left, r=right, s=stop, q=quit): ")

        if cmd == "f":
            forward()
        elif cmd == "b":
            reverse()
        elif cmd == "l":
            left()
        elif cmd == "r":
            right()
        elif cmd == "s":
            stop()
        elif cmd == "q":
            stop()
            break
        else:
            print("Invalid command")

except KeyboardInterrupt:
    pass

finally:
    stop()
    GPIO.cleanup()