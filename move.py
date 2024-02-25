# Library Imports
import time
import RPi.GPIO as GPIO

# Motor pins and directions
# motor_EN_A: Pin7 | motor_EN_B: Pin11
# motor_A: Pin8, Pin10 | motor_B: Pin13, Pin12
Motor_A_EN = 4
Motor_B_EN = 17

Motor_A_Pin1 = 26
Motor_A_Pin2 = 21
Motor_B_Pin1 = 27
Motor_B_Pin2 = 18

Dir_forward = 0
Dir_backward = 1

left_forward = 1
left_backward = 0

right_forward = 0
right_backward = 1

pwn_A = 0
pwm_B = 0

# Define line sensors
leftLine_sensor = 5
middleLine_sensor = 6
rightLine_sensor = 7

def motorStop():
    # Motor stops
    GPIO.output(Motor_A_Pin1, GPIO.LOW)
    GPIO.output(Motor_A_Pin2, GPIO.LOW)
    GPIO.output(Motor_B_Pin1, GPIO.LOW)
    GPIO.output(Motor_B_Pin2, GPIO.LOW)
    GPIO.output(Motor_A_EN, GPIO.LOW)
    GPIO.output(Motor_B_EN, GPIO.LOW)


def setup():
    # Motor initialization
    global pwm_A, pwm_B
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Motor_A_EN, GPIO.OUT)
    GPIO.setup(Motor_B_EN, GPIO.OUT)
    
    GPIO.setup(Motor_A_Pin1, GPIO.OUT)
    GPIO.setup(Motor_A_Pin2, GPIO.OUT)
    GPIO.setup(Motor_B_Pin1, GPIO.OUT)
    GPIO.setup(Motor_B_Pin2, GPIO.OUT)
    
    GPIO.setup(leftLine_sensor, GPIO.IN)
    GPIO.setup(middleLine_sensor, GPIO.IN)
    GPIO.setup(rightLine_sensor, GPIO.IN) 

    motorStop()
    try:
        pwm_A = GPIO.PWM(Motor_A_EN, 1000)
        pwm_B = GPIO.PWM(Motor_B_EN, 1000)
    except:
        pass


def motor_left(status, direction, speed):
    # Motor 2 positive and negative rotation
    if status == 0:  # stop
        GPIO.output(Motor_B_Pin1, GPIO.LOW)
        GPIO.output(Motor_B_Pin2, GPIO.LOW)
        GPIO.output(Motor_B_EN, GPIO.LOW)
    else:
        if direction == Dir_backward:
            GPIO.output(Motor_B_Pin1, GPIO.HIGH)
            GPIO.output(Motor_B_Pin2, GPIO.LOW)
            pwm_B.start(100)
            pwm_B.ChangeDutyCycle(speed)
        elif direction == Dir_forward:
            GPIO.output(Motor_B_Pin1, GPIO.LOW)
            GPIO.output(Motor_B_Pin2, GPIO.HIGH)
            pwm_B.start(0)
            pwm_B.ChangeDutyCycle(speed)


def motor_right(status, direction, speed):
    # Motor 1 positive and negative rotation
    if status == 0:  # stop
        GPIO.output(Motor_A_Pin1, GPIO.LOW)
        GPIO.output(Motor_A_Pin2, GPIO.LOW)
        GPIO.output(Motor_A_EN, GPIO.LOW)
    else:
        if direction == Dir_forward:
            GPIO.output(Motor_A_Pin1, GPIO.HIGH)
            GPIO.output(Motor_A_Pin2, GPIO.LOW)
            pwm_A.start(100)
            pwm_A.ChangeDutyCycle(speed)
        elif direction == Dir_backward:
            GPIO.output(Motor_A_Pin1, GPIO.LOW)
            GPIO.output(Motor_A_Pin2, GPIO.HIGH)
            pwm_A.start(0)
            pwm_A.ChangeDutyCycle(speed)


def move(speed, direction, turn, radius=0.6):    
    # Move the robot based on the specified speed, direction, turn, and optional radius
    # Speed is in the range [0, 100], direction can be 'forward' or 'backward', turn can be 'right', 'left', or 'no'
    # Radius adjusts the turning radius, where 0 < radius <= 1
    
    if direction == 'forward':
        if turn == 'right':
            motor_left(0, left_backward, int(speed*radius))
            motor_right(1, right_forward, speed)
        elif turn == 'left':
            motor_left(1, left_forward, speed)
            motor_right(0, right_backward, int(speed*radius))
        else:
            motor_left(1, left_forward, speed)
            motor_right(1, right_forward, speed)
    elif direction == 'backward':
        if turn == 'right':
            motor_left(0, left_forward, int(speed*radius))
            motor_right(1, right_backward, speed)
        elif turn == 'left':
            motor_left(1, left_backward, speed)
            motor_right(0, right_forward, int(speed*radius))
        else:
            motor_left(1, left_backward, speed)
            motor_right(1, right_backward, speed)
    elif direction == 'no':
        if turn == 'right':
            motor_left(1, left_backward, speed)
            motor_right(1, right_forward, speed)
        elif turn == 'left':
            motor_left(1, left_forward, speed)
            motor_right(1, right_backward, speed)
        else:
            motorStop()
    else:
        pass


def destroy():
    # Stop the motors and release GPIO resources
    motorStop()
    GPIO.cleanup()


def lineDetection(leftLine_sensor, middleLine_sensor, rightLine_sensor):
    # Detect the position of the line based on sensor readings
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
		

if __name__ == '__main__':
    try:
        speed_set = 60
        setup()
        move(speed_set, 'forward', 'no', 0.8)
        time.sleep(1.3)
        motorStop()
        destroy()
    except KeyboardInterrupt:
        destroy()



