import RPi.GPIO as GPIO
import time

# Pin setup
IN1 = 26
IN2 = 19
IN3 = 13
IN4 = 6
en_a=4
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(en_a, GPIO.OUT)
power_a=GPIO.PWM(en_a,100)
power_a.start(60)
GPIO.setwarnings(False)

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
    GPIO.output(IN1, True)
    GPIO.output(IN2, False)
    GPIO.output(IN3, False)
    GPIO.output(IN4, False)

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