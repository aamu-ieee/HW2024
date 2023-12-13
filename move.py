#Library Imports
from gpiozero import Robot, Motor, LineSensor
import RPi.GPIO as GPIO
import threading
import time


"""
    Using inbuilt libraries for robot control
    While easy, could pose challenges down the line with
    controlling sub units or achieveing desired results
"""
# Define motors
left_motor = Motor(forward=4, backward=14)
right_motor = Motor(forward=17, backward=27)

# Create a robot with motors
robot = Robot(left=left_motor, right=right_motor)

# Define line sensors
leftLine_sensor = LineSensor(5)
middleLine_sensor = LineSensor(6)
rightLine_sensor = LineSensor(7)

leftLine_sensor.when_line = robot.left
rightLine_sensor.when_line = robot.right

leftLine_sensor.when_no_line = robot.forward
rightLine_sensor.when_no_line = robot.forward
middleLine_sensor.when_line = robot.forward

speed = 0.65

def motor_speed():
    while True:
        left_detect  = int(leftLine_sensor.value)
        right_detect = int(rightLine_sensor.value)
        middle_detect = int(middleLine_sensor.value)

        ## Stage 1
        if left_detect == 0 and right_detect == 0:
            left_mot = 1
            right_mot = 1
        ## Stage 2
        if left_detect == 0 and right_detect == 1:
            left_mot = -1
        if left_detect == 1 and right_detect == 0:
            right_mot = -1

        yield (right_mot * speed, left_mot * speed)

robot.source = motor_speed()

time.sleep(60)
robot.stop()
robot.source = None
robot.close()
leftLine_sensor.close()
rightLine_sensor.close()


"""
    Manual method of robot control,
    This is preferred because it gives low level control of all
    robotic operations
"""
leftMotor_en = 4  
leftMotor_pin1= 26  
leftMotor_pin2= 21

rightMotor_en = 4  
rightMotor_pin1= 26  
rightMotor_pin2= 21

leftLine_sensor= 19  
middleLine_sensor = 16  
rightLine_sensor = 20

def setup():  
    GPIO.setwarnings(False)  
    GPIO.setmode(GPIO.BCM)  

    GPIO.setup(leftLine_sensor,GPIO.IN)  
    GPIO.setup(middleLine_sensor,GPIO.IN)  
    GPIO.setup(rightLine_sensor,GPIO.IN)  

    GPIO.setup(leftMotor_en, GPIO.OUT)
    GPIO.setup(leftMotor_pin1, GPIO.OUT)
    GPIO.setup(leftMotor_pin2, GPIO.OUT)

    GPIO.setup(rightMotor_en, GPIO.OUT)
    GPIO.setup(rightMotor_pin1, GPIO.OUT)
    GPIO.setup(rightMotor_pin2, GPIO.OUT)    

    try:
        pwm_left = GPIO.PWM(leftMotor_en, 1000)
        pwm_right = GPIO.PWM(rightMotor_en, 1000)
    except:
        print('Unable to set Pulse Width Modulation (PWM)')
        pass

 
class Movement():
    DIST_PER_SEC  = 35.0 # cm/s
    SEC_PER_TURN  = 2.087 # seconds per full turn (depending on the base material, use forwardLeft(), stop() and time.time() to measure the time)
 
    def __init__(self, left_pin1, left_pin2, right_pin1, right_pin2):
        self.MOTOR_LEFT_PIN1  = left_pin1
        self.MOTOR_LEFT_PIN2  = left_pin2
        self.MOTOR_RIGHT_PIN1 = right_pin1
        self.MOTOR_RIGHT_PIN2 = right_pin2
 
        GPIO.setup(self.MOTOR_LEFT_PIN1, GPIO.OUT)
        GPIO.setup(self.MOTOR_LEFT_PIN2, GPIO.OUT)
        GPIO.setup(self.MOTOR_RIGHT_PIN1, GPIO.OUT)
        GPIO.setup(self.MOTOR_RIGHT_PIN2, GPIO.OUT)
 
    def forwardLeft(self):
        GPIO.output(self.MOTOR_LEFT_PIN1, GPIO.LOW)
        GPIO.output(self.MOTOR_LEFT_PIN2, GPIO.HIGH)
        GPIO.output(self.MOTOR_RIGHT_PIN1, GPIO.LOW)
        GPIO.output(self.MOTOR_RIGHT_PIN2, GPIO.LOW)
        return time.time()
 
    def forwardRight(self):
        GPIO.output(self.MOTOR_RIGHT_PIN1, GPIO.LOW)
        GPIO.output(self.MOTOR_RIGHT_PIN2, GPIO.HIGH)
        GPIO.output(self.MOTOR_LEFT_PIN1, GPIO.LOW)
        GPIO.output(self.MOTOR_LEFT_PIN2, GPIO.LOW)
        return time.time()
 
    def forward(self):
        GPIO.output(self.MOTOR_LEFT_PIN1, GPIO.LOW)
        GPIO.output(self.MOTOR_LEFT_PIN2, GPIO.HIGH)
        GPIO.output(self.MOTOR_RIGHT_PIN1, GPIO.LOW)
        GPIO.output(self.MOTOR_RIGHT_PIN2, GPIO.HIGH)
        return time.time()
 
    def backwardLeft(self):
        GPIO.output(self.MOTOR_RIGHT_PIN1, GPIO.HIGH)
        GPIO.output(self.MOTOR_RIGHT_PIN2, GPIO.LOW)
        GPIO.output(self.MOTOR_LEFT_PIN1, GPIO.LOW)
        GPIO.output(self.MOTOR_LEFT_PIN2, GPIO.LOW)
        return time.time()
 
    def backwardRight(self):
        GPIO.output(self.MOTOR_RIGHT_PIN1, GPIO.HIGH)
        GPIO.output(self.MOTOR_RIGHT_PIN2, GPIO.LOW)
        GPIO.output(self.MOTOR_LEFT_PIN1, GPIO.LOW)
        GPIO.output(self.MOTOR_LEFT_PIN2, GPIO.LOW)
        return time.time()
 
    def backward(self):
        GPIO.output(self.MOTOR_LEFT_PIN1, GPIO.HIGH)
        GPIO.output(self.MOTOR_LEFT_PIN2, GPIO.LOW)
        GPIO.output(self.MOTOR_RIGHT_PIN1, GPIO.HIGH)
        GPIO.output(self.MOTOR_RIGHT_PIN2, GPIO.LOW)
        return time.time()
 
    def stop(self):
        GPIO.output(self.MOTOR_LEFT_PIN1, GPIO.LOW)
        GPIO.output(self.MOTOR_LEFT_PIN2, GPIO.LOW)
        GPIO.output(self.MOTOR_RIGHT_PIN1, GPIO.LOW)
        GPIO.output(self.MOTOR_RIGHT_PIN2, GPIO.LOW)
 
    def forwardDistance(self, dist):
        self.forward()
        threading.Timer(dist/self.DIST_PER_SEC, self.stop)
 
    def backwardDistance(self, dist):
        self.backward()
        threading.Timer(dist/self.DIST_PER_SEC, self.stop)


def lineDetection(leftLine_sensor, middleLine_sensor, rightLine_sensor):
    status_right = GPIO.input(leftLine_sensor)  
    status_middle = GPIO.input(middleLine_sensor)  
    status_left = GPIO.input(rightLine_sensor)

    if status_left == 1 and status_middle == 0 and status_right == 0:
        print('Turn Left')
    elif status_left == 0 and status_middle == 1 and status_right == 0:
        print('Go forward')
    elif status_left == 0 and status_middle == 0 and status_right == 1:
        print('Turn Right')
    else:
        print('Line not detected')