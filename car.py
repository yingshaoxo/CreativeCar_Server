# -*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
from simple_pid import PID
import time

pid = PID(1.3,  # 1,
          0,
          1,
          setpoint=0,
          )
pid.output_limits = (-100, 100)

# Set the GPIO port to BCM encoding mode
GPIO.setmode(GPIO.BCM)

# Ignore warning information
GPIO.setwarnings(False)

# --------------------------------
# --------------------------------
# Motor Controlling
# --------------------------------
# --------------------------------

# Definition of motor pins
IN1 = 14
IN2 = 4
IN3 = 3
IN4 = 2
ENA = 13
ENB = 12


def initiate_motor():
    global pwm_ENA
    global pwm_ENB

    #GPIO.setup(ENA, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(ENA, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(IN1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(IN2, GPIO.OUT, initial=GPIO.LOW)
    #GPIO.setup(ENB, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(ENB, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(IN3, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(IN4, GPIO.OUT, initial=GPIO.LOW)

    # Set the PWM pin and frequency is 2000hz
    pwm_ENA = GPIO.PWM(ENA, 1000)
    pwm_ENB = GPIO.PWM(ENB, 1000)
    pwm_ENA.start(0)
    pwm_ENB.start(0)


def go(speed, delaytime=0):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(speed)
    pwm_ENB.ChangeDutyCycle(speed)
    time.sleep(delaytime)


def back(speed, delaytime=0):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(speed)
    pwm_ENB.ChangeDutyCycle(speed)
    time.sleep(delaytime)


def left(speed1, speed2, delaytime=0):
    time.sleep(delaytime)
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(speed1)
    pwm_ENB.ChangeDutyCycle(speed2)
    time.sleep(delaytime)


def right(speed1, speed2, delaytime=0):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(speed1)
    pwm_ENB.ChangeDutyCycle(speed2)
    time.sleep(delaytime)


"""
def left(speed, delaytime=0):
    time.sleep(delaytime)
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(speed)
    pwm_ENB.ChangeDutyCycle(speed)
    time.sleep(delaytime)


def right(speed, delaytime=0):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(speed)
    pwm_ENB.ChangeDutyCycle(speed)
    time.sleep(delaytime)
"""


def left_rotate(speed, delaytime=0):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(speed)
    pwm_ENB.ChangeDutyCycle(speed)
    time.sleep(delaytime)


def right_rotate(speed, delaytime=0):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(speed)
    pwm_ENB.ChangeDutyCycle(speed)
    time.sleep(delaytime)


def stop(speed=0, delaytime=0):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(0)
    pwm_ENB.ChangeDutyCycle(0)
    time.sleep(delaytime)

# --------------------------------
# --------------------------------
# Line Finding
# --------------------------------
# --------------------------------


# Definition of line finding pins
A_pin = 7
B_pin = 8
C_pin = 10
D_pin = 22
E_pin = 27
F_pin = 17

A = 0
B = 0
C = 0
D = 0
E = 0
F = 0


def initiate_line_finding_module():
    GPIO.setup(A_pin, GPIO.IN)
    GPIO.setup(B_pin, GPIO.IN)
    GPIO.setup(C_pin, GPIO.IN)
    GPIO.setup(D_pin, GPIO.IN)
    GPIO.setup(E_pin, GPIO.IN)
    GPIO.setup(F_pin, GPIO.IN)


def update_ABCDEF():
    global A, B, C, D, E, F
    A = GPIO.input(A_pin)
    B = GPIO.input(B_pin)
    C = GPIO.input(C_pin)
    D = GPIO.input(D_pin)
    E = GPIO.input(E_pin)
    F = GPIO.input(F_pin)

# --------------------------------
# --------------------------------
# Automatic Car
# --------------------------------
# --------------------------------


def constrain(val, min_val, max_val):
    result = min(max_val, max(min_val, val))
    return result


ALL_BLACK = 0
FOLLOWING_LINE = 1
ALL_WHITE = 2

mode = 0
error = 0

initial_MotorPower_for_go_straight = 40 #40 * 3
initial_MotorPower_for_turning = 20  # 20  # the speed that just enough for running your car


class AutoCar:
    def __init__(self):
        initiate_motor()
        initiate_line_finding_module()

    def update_error(self):
        global mode

        global error
        global pid

        global A, B, C, D, E, F
        """
        read line sensors values 

        black = 1;
        white = 0;

        Sensor Array 	Error Value
        0 0 (0&0) 0 1	 4              
        0 0 (0&0) 1 1	 3              
        0 0 (0&0) 1 0	 2              
        0 0 (0&1) 1 0	 1              
        0 0 (0&1) 0 0    0.5

        0 0 (1&1) 0 0	 0              

        0 0 (1&0) 0 0   -0.5
        0 1 (1&0) 0 0	-1              
        0 1 (0&0) 0 0	-2              
        1 1 (0&0) 0 0	-3              
        1 0 (0&0) 0 0	-4              

        1 1 1 1 1        0 full_black
        0 0 0 0 0        0 full_white
        """
        if ((A == 0) and (B == 0) and ((C == 0) and (D == 0)) and (E == 0) and (F == 1)):
            mode = FOLLOWING_LINE
            error = 5
        elif ((A == 0) and (B == 0) and ((C == 0) and (D == 0)) and (E == 1) and (F == 1)):
            mode = FOLLOWING_LINE
            error = 4
        elif ((A == 0) and (B == 0) and ((C == 0) and (D == 0)) and (E == 1) and (F == 0)):
            mode = FOLLOWING_LINE
            error = 3
        elif ((A == 0) and (B == 0) and ((C == 0) and (D == 1)) and (E == 1) and (F == 0)):
            mode = FOLLOWING_LINE
            error = 2
        elif ((A == 0) and (B == 0) and ((C == 0) and (D == 1)) and (E == 0) and (F == 0)):
            mode = FOLLOWING_LINE
            error = 1
        elif ((A == 0) and (B == 0) and ((C == 1) and (D == 1)) and (E == 0) and (F == 0)):
            mode = FOLLOWING_LINE
            error = 0
        elif ((A == 0) and (B == 0) and ((C == 1) and (D == 0)) and (E == 0) and (F == 0)):
            mode = FOLLOWING_LINE
            error = -1
        elif ((A == 0) and (B == 1) and ((C == 1) and (D == 0)) and (E == 0) and (F == 0)):
            mode = FOLLOWING_LINE
            error = -2
        elif ((A == 0) and (B == 1) and ((C == 0) and (D == 0)) and (E == 0) and (F == 0)):
            mode = FOLLOWING_LINE
            error = -3
        elif ((A == 1) and (B == 1) and ((C == 0) and (D == 0)) and (E == 0) and (F == 0)):
            mode = FOLLOWING_LINE
            error = -4
        elif ((A == 1) and (B == 0) and ((C == 0) and (D == 0)) and (E == 0) and (F == 0)):
            mode = FOLLOWING_LINE
            error = -5
        elif ((A == 1) and (B == 1) and ((C == 1) and (D == 1)) and (E == 1) and (F == 1)):
            mode = ALL_BLACK
            error = 0
        elif ((A == 0) and (B == 0) and ((C == 0) and (D == 0)) and (C == 0) and (D == 0)):
            mode = ALL_WHITE
            error = 0

        print(f"got error: {error}")

    def motor_PID_control(self):
        global error
        global pid

        global A, B, C, D, E, F

        pid_value = pid(error)
        print(f"PID value: {pid_value}")

        if (round(pid_value) == 0):
            go(initial_MotorPower_for_go_straight)
            print(f"Go speed: {initial_MotorPower_for_go_straight}")
        elif (pid_value < 0):
            MotorSpeed1 = initial_MotorPower_for_go_straight - abs(pid_value)*3
            MotorSpeed2 = initial_MotorPower_for_go_straight + abs(pid_value)*3
            MotorSpeed1 = constrain(MotorSpeed1, 0, 100)
            MotorSpeed2 = constrain(MotorSpeed2, 0, 100)

            # right_rotate(MotorSpeed)
            right(MotorSpeed1, MotorSpeed2)
            print(f"Right speed: {MotorSpeed1}, {MotorSpeed2}")
        elif (pid_value > 0):
            MotorSpeed1 = initial_MotorPower_for_go_straight + abs(pid_value)*3
            MotorSpeed2 = initial_MotorPower_for_go_straight - abs(pid_value)*3
            MotorSpeed1 = constrain(MotorSpeed1, 0, 100)
            MotorSpeed2 = constrain(MotorSpeed2, 0, 100)

            # left_rotate(MotorSpeed)
            left(MotorSpeed1, MotorSpeed2)
            print(f"Left speed: {MotorSpeed1}, {MotorSpeed2}")

    def start_to_drive(self):
        global mode

        while 1:
            update_ABCDEF()
            self.update_error()

            if mode == ALL_BLACK:
                stop(1)
            elif mode == ALL_WHITE:
                stop(1)
            elif mode == FOLLOWING_LINE:
                self.motor_PID_control()

            # time.sleep(1)
            print("\n\n-------------------------\n\n")


class Car:
    def __init__(self):
        initiate_motor()
        self._get_away_from_obstacles_is_running = False

    def action(self, action_name, speed=50, time_=0):
        if action_name == "go":
            go(speed, time_)
        elif action_name == "back":
            back(speed, time_)
        elif action_name == "left":
            left(speed, time_)
            #left_rotate(speed, time_)
        elif action_name == "right":
            right(speed, time_)
            #right_rotate(speed, time_)
        elif action_name == "stop":
            stop(speed, time_)

    def get_away_from_obstacles(self, ratio=1, always=False):
        pass

    def start_camera(self):
        pass

    def camera_action(self, action_name):
        pass

    def stop_camera(self):
        pass

    def quit(self):
        self.action(action_name="stop")
        pwm_ENA.stop()
        pwm_ENB.stop()
        GPIO.cleanup()


if __name__ == '__main__':
    """
    initiate_motor()
    while 1:
        left(20)
    """

    auto_car = AutoCar()
    auto_car.start_to_drive()

    """
    initiate_motor()
    initiate_line_finding_module()
    while 1:
        update_ABCDEF()
        if (A == 1 and F == 0):
            left_rotate(30)
        elif (A == 0 and F == 1):
            right_rotate(30)
        else:
            go(30)
    """
