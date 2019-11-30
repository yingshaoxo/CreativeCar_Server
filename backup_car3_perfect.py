# -*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
from simple_pid import PID
import time
from datetime import datetime

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
# General Function
# --------------------------------
# --------------------------------


def constrain(val, min_val, max_val):
    result = min(max_val, max(min_val, val))
    return result


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

initial_MotorPower_for_go_straight = 50
ratio = 3
near_tunnel_ratio = 0.8
initial_MotorPower_for_across_tunnel = 40


def initiate_motor():
    global pwm_ENA
    global pwm_ENB

    GPIO.setup(ENA, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(IN1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(IN2, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(ENB, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(IN3, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(IN4, GPIO.OUT, initial=GPIO.LOW)

    # Set the PWM pin and frequency is 1000hz
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
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)

    if speed2 < 0:
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
        speed2 = abs(speed2)
    else:
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)

    pwm_ENA.ChangeDutyCycle(speed1)
    pwm_ENB.ChangeDutyCycle(speed2)
    time.sleep(delaytime)


def right(speed1, speed2, delaytime=0):
    if speed1 < 0:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        speed1 = abs(speed1)
    else:
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)

    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

    pwm_ENA.ChangeDutyCycle(speed1)
    pwm_ENB.ChangeDutyCycle(speed2)
    time.sleep(delaytime)


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
    start_time = datetime.now()

    diff_in_millisecond = 0
    while (diff_in_millisecond < delaytime*1000):
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.LOW)
        pwm_ENA.ChangeDutyCycle(0)
        pwm_ENB.ChangeDutyCycle(0)

        diff = (datetime.now() - start_time)
        diff_in_millisecond = (diff.days * 86400000) + (diff.seconds * 1000) + (diff.microseconds / 1000)

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
    print(f"""
A: {A}
B: {B}
C: {C}
D: {D}
E: {E}
F: {F}
    """)

# --------------------------------
# --------------------------------
# Ultrasonic sensor functions
# --------------------------------
# --------------------------------


Left_Echo = 21
Left_Triger = 20
Right_Echo = 23
Right_Triger = 24


def initiate_ultrasonic_sensor():
    GPIO.setup(Left_Echo, GPIO.IN)
    GPIO.setup(Left_Triger, GPIO.OUT)
    GPIO.setup(Right_Echo, GPIO.IN)
    GPIO.setup(Right_Triger, GPIO.OUT)


def get_left_distance():
    maxTime = 0.04

    GPIO.output(Left_Triger, False)
    time.sleep(0.01)
    GPIO.output(Left_Triger, True)
    time.sleep(0.00001)
    GPIO.output(Left_Triger, False)

    pulse_start = time.time()
    timeout = pulse_start + maxTime
    while GPIO.input(Left_Echo) == 0 and pulse_start < timeout:
        pulse_start = time.time()

    pulse_end = time.time()
    timeout = pulse_end + maxTime
    while GPIO.input(Left_Echo) == 1 and pulse_end < timeout:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17000
    distance = round(distance, 2)

    distance = constrain(distance, 0, 100)

    return distance


def get_right_distance():
    maxTime = 0.04

    GPIO.output(Right_Triger, False)
    time.sleep(0.01)
    GPIO.output(Right_Triger, True)
    time.sleep(0.00001)
    GPIO.output(Right_Triger, False)

    pulse_start = time.time()
    timeout = pulse_start + maxTime
    while GPIO.input(Right_Echo) == 0 and pulse_start < timeout:
        pulse_start = time.time()

    pulse_end = time.time()
    timeout = pulse_end + maxTime
    while GPIO.input(Right_Echo) == 1 and pulse_end < timeout:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17000
    distance = round(distance, 2)

    distance = constrain(distance, 0, 100)

    return distance


def check_if_we_are_in_tunnel():
    left_distance = get_left_distance()
    right_distance = get_right_distance()

    if (left_distance < 50 and right_distance < 50):
        return 1
    else:
        return 0


def adjust_its_position_to_the_center():
    left_distance = get_left_distance()
    right_distance = get_right_distance()

    middle_value = ((left_distance + right_distance) / 2)

    if (left_distance < middle_value):
        right(initial_MotorPower_for_across_tunnel, 0)
    elif (right_distance < middle_value):
        left(0, initial_MotorPower_for_across_tunnel)

# --------------------------------
# --------------------------------
# Automatic Car
# --------------------------------
# --------------------------------


ALL_BLACK = 0
FOLLOWING_LINE = 1
ALL_WHITE = 2

mode = 0
error = 0


class Timer:
    def __init__(self):
        self.seconds = 0
        self.temp_seconds = 0
        self.general_report_in_a_second = 0

        self.full_white_report_in_a_second = 0

        self.beginning = datetime.now()

    def get_real_seconds(self):
        diff = (datetime.now() - self.beginning)
        return diff.seconds

    def get_live_seconds(self):
        diff = (datetime.now() - self.beginning)
        diff_in_millisecond = (diff.days * 86400000) + (diff.seconds * 1000) + (diff.microseconds / 1000)
        return diff_in_millisecond // 200
        """
        diff = (datetime.now() - self.beginning)
        return diff.seconds
        """

    def general_report(self):
        self.general_report_in_a_second += 1

        self.temp_seconds = self.get_live_seconds()
        if (self.temp_seconds > self.seconds):
            self.seconds = self.temp_seconds
            self.general_report_in_a_second = 0
            self.full_white_report_in_a_second = 0
            self.any_black_report_in_a_second = 0

    def a_full_white_report(self):
        self.full_white_report_in_a_second += 1

        self.temp_seconds = self.get_live_seconds()
        if (self.temp_seconds > self.seconds):
            self.seconds = self.temp_seconds
            self.full_white_report_in_a_second = 0

        if (self.general_report_in_a_second > 30 and (self.full_white_report_in_a_second / self.general_report_in_a_second) > 0.8):
            return 1
        else:
            return 0


class Counter:
    def __init__(self):
        self.dict = {}

    def _get_time_difference_in_milliseconds(self, new, old):
        diff = (new - old)
        diff_in_millisecond = (diff.days * 86400000) + (diff.seconds * 1000) + (diff.microseconds / 1000)
        return diff_in_millisecond

    def count(self, name, value, time_inverval, chance_of_showing):
        date_of_now = datetime.now()

        if name in self.dict:
            self.dict[name].update({
                date_of_now: value
            })

            out_of_date_items = []
            for key in self.dict[name]:
                if (self._get_time_difference_in_milliseconds(date_of_now, key) > time_inverval):
                    out_of_date_items.append(key)

            for key in out_of_date_items:
                del self.dict[name][key]

            if out_of_date_items != []:
                all_values = list(self.dict[name].values())
                possibility = all_values.count(value)/len(all_values)

                if possibility >= chance_of_showing:
                    return 1
                else:
                    return 0

        elif name not in self.dict:
            self.dict.update({
                name: {date_of_now: value}
            })

        return 0


class AutoCar:
    def __init__(self):
        initiate_motor()
        initiate_line_finding_module()
        initiate_ultrasonic_sensor()

        self.timer = Timer()
        self.full_white_count = 0

        self.counter = Counter()

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
        0 0 (0&0) 0 1	 5              
        0 0 (0&0) 1 1	 4              
        0 0 (0&0) 1 0	 3              
        0 0 (0&1) 1 0	 2              
        0 0 (0&1) 0 0    1

        0 0 (1&1) 0 0	 0              

        0 0 (1&0) 0 0   -1
        0 1 (1&0) 0 0	-2              
        0 1 (0&0) 0 0	-3              
        1 1 (0&0) 0 0	-4              
        1 0 (0&0) 0 0	-5              

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

        if error != 0:
            print(f"got error: {error}")

    def update_error_by_ultrasonic_sensor(self):
        global error

        left_distance = get_left_distance()
        right_distance = get_right_distance()

        middle_value = ((left_distance + right_distance) / 2)

        if (left_distance < middle_value):
            error = 3
        elif (right_distance < middle_value):
            error = -3

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
            MotorSpeed1 = initial_MotorPower_for_go_straight - abs(pid_value)*ratio
            MotorSpeed2 = initial_MotorPower_for_go_straight + abs(pid_value)*ratio
            MotorSpeed1 = constrain(MotorSpeed1, 0, 100)
            MotorSpeed2 = constrain(MotorSpeed2, 0, 100)

            # right_rotate(MotorSpeed)
            right(MotorSpeed1, MotorSpeed2)
            print(f"Right speed: {MotorSpeed1}, {MotorSpeed2}")
        elif (pid_value > 0):
            MotorSpeed1 = initial_MotorPower_for_go_straight + abs(pid_value)*ratio
            MotorSpeed2 = initial_MotorPower_for_go_straight - abs(pid_value)*ratio
            MotorSpeed1 = constrain(MotorSpeed1, 0, 100)
            MotorSpeed2 = constrain(MotorSpeed2, 0, 100)

            # left_rotate(MotorSpeed)
            left(MotorSpeed1, MotorSpeed2)
            print(f"Left speed: {MotorSpeed1}, {MotorSpeed2}")

    def motor_PID_control_by_ultrasonic_sensor(self):
        global error
        global pid

        global A, B, C, D, E, F

        pid_value = pid(error)
        print(f"PID value: {pid_value}")

        if (round(pid_value) == 0):
            go(initial_MotorPower_for_go_straight)
            print(f"Go speed: {initial_MotorPower_for_go_straight}")
        elif (pid_value < 0):
            MotorSpeed1 = initial_MotorPower_for_across_tunnel - abs(pid_value)
            MotorSpeed2 = initial_MotorPower_for_across_tunnel + abs(pid_value)
            MotorSpeed1 = constrain(MotorSpeed1, 0, 100)
            MotorSpeed2 = constrain(MotorSpeed2, 0, 100)

            right(MotorSpeed1, MotorSpeed2)
            print(f"Right speed: {MotorSpeed1}, {MotorSpeed2}")
        elif (pid_value > 0):
            MotorSpeed1 = initial_MotorPower_for_across_tunnel + abs(pid_value)
            MotorSpeed2 = initial_MotorPower_for_across_tunnel - abs(pid_value)
            MotorSpeed1 = constrain(MotorSpeed1, 0, 100)
            MotorSpeed2 = constrain(MotorSpeed2, 0, 100)

            left(MotorSpeed1, MotorSpeed2)
            print(f"Left speed: {MotorSpeed1}, {MotorSpeed2}")

    def start_to_drive(self):
        global mode
        global A, B, C, D, E, F
        global pid
        global initial_MotorPower_for_go_straight
        global initial_MotorPower_for_across_tunnel
        global ratio, near_tunnel_ratio

        while 1:
            update_ABCDEF()
            self.update_error()

            if self.timer.get_real_seconds() >= 8:
                initial_MotorPower_for_go_straight = initial_MotorPower_for_across_tunnel
                ratio = near_tunnel_ratio

            if mode == ALL_BLACK:
                pass
            elif mode == ALL_WHITE:
                self.timer.general_report()

                if (self.timer.a_full_white_report() == 1 and (check_if_we_are_in_tunnel() == 1 or self.full_white_count == 1)):
                    stop(0, 1)
                    self.full_white_count += 1

                    if (self.full_white_count == 1):
                        print("We are in tunnel!")
                        while 1:
                            self.update_error_by_ultrasonic_sensor()
                            self.motor_PID_control_by_ultrasonic_sensor()

                            we_are_in_tunnel = check_if_we_are_in_tunnel()
                            if we_are_in_tunnel == 1:
                                self.counter.count("tunnel_detection", 1, 100, 0.9)
                            elif we_are_in_tunnel == 0:
                                result = self.counter.count("tunnel_detection", 0, 100, 0.9)
                                if result == 1:
                                    print("We are not in tunnel!")

                                    start_point = datetime.now()
                                    while(self.counter._get_time_difference_in_milliseconds(datetime.now(), start_point) < 500):
                                        back(initial_MotorPower_for_across_tunnel*0.8)
                                    stop(0, 0.4)

                                    find_black_line = 0
                                    left_right_flag = 1
                                    times = 0
                                    while (find_black_line == 0):
                                        if left_right_flag == 1:
                                            timeout = 0.6 * 1000
                                        else:
                                            timeout = 0.6 * 2 * 1000

                                        if left_right_flag == 1:
                                            start_point = datetime.now()
                                            while(self.counter._get_time_difference_in_milliseconds(datetime.now(), start_point) < timeout):
                                                left_rotate(initial_MotorPower_for_across_tunnel*0.8)
                                                update_ABCDEF()
                                                if (C == 1 or D == 1):
                                                    print("We got the black line again!")
                                                    find_black_line = 1
                                                    stop(0, 1)
                                                    break
                                        else:
                                            start_point = datetime.now()
                                            while(self.counter._get_time_difference_in_milliseconds(datetime.now(), start_point) < timeout):
                                                right_rotate(initial_MotorPower_for_across_tunnel*0.8)
                                                update_ABCDEF()
                                                if (C == 1 or D == 1):
                                                    print("We got the black line again!")
                                                    find_black_line = 1
                                                    stop(0, 1)
                                                    break

                                        left_right_flag = left_right_flag * -1

                                        times += 1
                                        if times == 3:
                                            #left_right_flag = 1
                                            #times = 0
                                            break
                                    if (find_black_line == 1):
                                        initial_MotorPower_for_go_straight = initial_MotorPower_for_across_tunnel
                                        ratio = 3
                                        break

                    elif (self.full_white_count == 2):
                        print("Finished!")
                        exit()

                pid.reset()

            elif mode == FOLLOWING_LINE:
                self.timer.general_report()

                self.motor_PID_control()

            # time.sleep(1)
            # print("\n\n-------------------------\n\n")


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
            left_rotate(speed, time_)
        elif action_name == "right":
            right_rotate(speed, time_)
        elif action_name == "stop":
            stop(speed, 0.1)

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
    auto_car = AutoCar()
    auto_car.start_to_drive()

    """
    initiate_ultrasonic_sensor()
    while 1:
        left_distance = get_left_distance()
        right_distance = get_right_distance()
        print(left_distance, right_distance)
    """

    """
    initiate_motor()
    initiate_line_finding_module()
    while 1:
        update_ABCDEF()
        if (A == 1 and F == 0):
            left_rotate(50)
        elif (A == 0 and F == 1):
            right_rotate(50)
        else:
            go(50)
    """
