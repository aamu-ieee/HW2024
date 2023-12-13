import time
import RPi.GPIO as GPIO

# Set GPIO mode
GPIO.setmode(GPIO.BCM)

# Define GPIO pins for motor control
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

pwm_A = None
pwm_B = None

def motorStop():
    # Stop both motors by setting all GPIO pins to LOW
    GPIO.output(Motor_A_Pin1, GPIO.LOW)
    GPIO.output(Motor_A_Pin2, GPIO.LOW)
    GPIO.output(Motor_B_Pin1, GPIO.LOW)
    GPIO.output(Motor_B_Pin2, GPIO.LOW)
    GPIO.output(Motor_A_EN, GPIO.LOW)
    GPIO.output(Motor_B_EN, GPIO.LOW)

def setup():
    global pwm_A, pwm_B
    GPIO.setwarnings(False)
    GPIO.setup(Motor_A_EN, GPIO.OUT)
    GPIO.setup(Motor_B_EN, GPIO.OUT)
    GPIO.setup(Motor_A_Pin1, GPIO.OUT)
    GPIO.setup(Motor_A_Pin2, GPIO.OUT)
    GPIO.setup(Motor_B_Pin1, GPIO.OUT)
    GPIO.setup(Motor_B_Pin2, GPIO.OUT)

    motorStop()
    try:
        # Initialize PWM objects for motor speed control
        pwm_A = GPIO.PWM(Motor_A_EN, 1000)
        pwm_B = GPIO.PWM(Motor_B_EN, 1000)
    except:
        pass

def motor_left(status, direction, speed):
    if status == 0:
        # Stop left motor
        GPIO.output(Motor_B_Pin1, GPIO.LOW)
        GPIO.output(Motor_B_Pin2, GPIO.LOW)
        GPIO.output(Motor_B_EN, GPIO.LOW)
    else:
        if direction == Dir_backward:
            # Set left motor direction to backward
            GPIO.output(Motor_B_Pin1, GPIO.HIGH)
            GPIO.output(Motor_B_Pin2, GPIO.LOW)
            pwm_B.start(100)
            pwm_B.ChangeDutyCycle(speed)
        elif direction == Dir_forward:
            # Set left motor direction to forward
            GPIO.output(Motor_B_Pin1, GPIO.LOW)
            GPIO.output(Motor_B_Pin2, GPIO.HIGH)
            pwm_B.start(0)
            pwm_B.ChangeDutyCycle(speed)

def motor_right(status, direction, speed):
    if status == 0:
        # Stop right motor
        GPIO.output(Motor_A_Pin1, GPIO.LOW)
        GPIO.output(Motor_A_Pin2, GPIO.LOW)
        GPIO.output(Motor_A_EN, GPIO.LOW)
    else:
        if direction == Dir_forward:
            # Set right motor direction to forward
            GPIO.output(Motor_A_Pin1, GPIO.HIGH)
            GPIO.output(Motor_A_Pin2, GPIO.LOW)
            pwm_A.start(100)
            pwm_A.ChangeDutyCycle(speed)
        elif direction == Dir_backward:
            # Set right motor direction to backward
            GPIO.output(Motor_A_Pin1, GPIO.LOW)
            GPIO.output(Motor_A_Pin2, GPIO.HIGH)
            pwm_A.start(0)
            pwm_A.ChangeDutyCycle(speed)

def move(speed, direction, turn, radius=0.6):
    if direction == 'forward':
        if turn == 'right':
            # Turn right by adjusting left motor speed and direction
            motor_left(0, left_backward, int(speed * radius))
            motor_right(1, right_forward, speed)
        elif turn == 'left':
            # Turn left by adjusting right motor speed and direction
            motor_left(1, left_forward, speed)
            motor_right(0, right_backward, int(speed * radius))
        else:
            # Move forward with both motors at the same speed
            motor_left(1, left_forward, speed)
            motor_right(1, right_forward, speed)
    elif direction == 'backward':
        if turn == 'right':
            # Turn right while moving backward by adjusting left motor speed and direction
            motor_left(0, left_forward, int(speed * radius))
            motor_right(1, right_backward, speed)
        elif turn == 'left':
            # Turn left while moving backward by adjusting right motor speed and direction
            motor_left(1, left_backward, speed)
            motor_right(0, right_forward, int(speed * radius))
        else:
            # Move backward with both motors at the same speed
            motor_left(1, left_backward, speed)
            motor_right(1, right_backward, speed)
    elif direction == 'no':
        if turn == 'right':
            # Turn right in place by adjusting motor directions
            motor_left(1, left_backward, speed)
            motor_right(1, right_forward, speed)
        elif turn == 'left':
            # Turn left in place by adjusting motor directions
            motor_left(1, left_forward, speed)
            motor_right(1, right_backward, speed)
        else:
            # Stop both motors
            motorStop()
    else:
        pass

def destroy():
    # Stop both motors and clean up GPIO pins
    motorStop()
    GPIO.cleanup()

if __name__ == '__main__':
    try:
        speed_set = 60
        setup()
        # Move forward for 1.3 seconds and then stop, testing motor control
        move(speed_set, 'forward', 'no', 0.8)
        time.sleep(1.3)
        motorStop()
        destroy()
    except KeyboardInterrupt:
        destroy()
