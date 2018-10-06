#-*- coding:UTF-8 -*-
from auto_everything.base import Terminal
terminal = Terminal()

import RPi.GPIO as GPIO
import threading
import time

#Definition of  motor pin 
IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
ENA = 16
ENB = 13

# Definition of  ultrasonic module pins
EchoPin = 0
TrigPin = 1

# Definition of RGB module pins
LED_R = 22
LED_G = 27
LED_B = 24

# Definition of servo pin
ServoPin = 23
Cam_left_or_right_servo_pin = 11
Cam_up_or_down_servo_pin = 9

# 红外避障引脚定义
AvoidSensorLeft = 12
AvoidSensorRight = 17

# stop signal
stop_signal = False

#Set the GPIO port to BCM encoding mode
GPIO.setmode(GPIO.BCM)

#Ignore warning information
GPIO.setwarnings(False)

#Motor pin initialization operation
def motor_init():
    global pwm_ENA
    global pwm_ENB
    global pwm_servo
    global pwm_cam_up_or_down_servo
    global pwm_cam_left_or_right_servo

    GPIO.setup(ENA,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN1,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN2,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(ENB,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN3,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN4,GPIO.OUT,initial=GPIO.LOW)

    GPIO.setup(EchoPin, GPIO.IN)
    GPIO.setup(TrigPin, GPIO.OUT)
    GPIO.setup(LED_R, GPIO.OUT)
    GPIO.setup(LED_G, GPIO.OUT)
    GPIO.setup(LED_B, GPIO.OUT)
    GPIO.setup(ServoPin, GPIO.OUT)
    GPIO.setup(Cam_left_or_right_servo_pin, GPIO.OUT)
    GPIO.setup(Cam_up_or_down_servo_pin, GPIO.OUT)
    GPIO.setup(AvoidSensorLeft, GPIO.IN)
    GPIO.setup(AvoidSensorRight, GPIO.IN)

    #Set the PWM pin and frequency is 2000hz
    pwm_ENA = GPIO.PWM(ENA, 2000)
    pwm_ENB = GPIO.PWM(ENB, 2000)
    pwm_ENA.start(0)
    pwm_ENB.start(0)

    # 设置舵机的频率和起始占空比
    pwm_servo = GPIO.PWM(ServoPin, 50)
    pwm_servo.start(0)

    pwm_cam_left_or_right_servo = GPIO.PWM(Cam_left_or_right_servo_pin, 50)
    pwm_cam_left_or_right_servo.start(0)

    pwm_cam_up_or_down_servo = GPIO.PWM(Cam_up_or_down_servo_pin, 50)
    pwm_cam_up_or_down_servo.start(0)

#advance
def go(speed, delaytime=0):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(speed)
    pwm_ENB.ChangeDutyCycle(speed)
    time.sleep(delaytime)

#back
def back(speed, delaytime=0):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(speed)
    pwm_ENB.ChangeDutyCycle(speed)
    time.sleep(delaytime)

#turn left in place
def spin_left(speed, delaytime=0):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(speed)
    pwm_ENB.ChangeDutyCycle(speed)
    time.sleep(delaytime)

#turn right in place
def spin_right(speed, delaytime=0):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(speed)
    pwm_ENB.ChangeDutyCycle(speed)
    time.sleep(delaytime)

#brake
def stop(speed=0, delaytime=0):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(0)
    pwm_ENB.ChangeDutyCycle(0)
    time.sleep(delaytime)

# rotate camera horizontally
def rotate_camera_horizontally(degree=90):
    for i in range(18):
        pwm_cam_left_or_right_servo.ChangeDutyCycle(1.80 + 100 * degree/1800)

# rotate camera vertically
def rotate_camera_vertically(degree=90):
    for i in range(18):
        pwm_cam_up_or_down_servo.ChangeDutyCycle(1.80 + 100 * degree/1800)

# 超声波函数
def Distance_test():
    GPIO.output(TrigPin, GPIO.HIGH)
    time.sleep(0.000015)
    GPIO.output(TrigPin, GPIO.LOW)
    while not GPIO.input(EchoPin):
        pass
    t1 = time.time()
    while GPIO.input(EchoPin):
        pass
    t2 = time.time()
    time.sleep(0.01)
    return ((t2 - t1) * 340 / 2) * 100

# 舵机旋转到指定角度
def servo_appointed_detection(pos):
    for i in range(18):
        pwm_servo.ChangeDutyCycle(2.5 + 10 * pos/180)

# get away from obstacles
def servo_color_carstate(ratio=1):
    global stop_signal

    speed = 65 * ratio

    # 开红灯
    GPIO.output(LED_R, GPIO.HIGH)
    GPIO.output(LED_G, GPIO.LOW)
    GPIO.output(LED_B, GPIO.LOW)
    back(20)
    time.sleep(0.08)
    stop()

    stop_signal = True
    # 舵机旋转到0度，即右侧，测距
    servo_appointed_detection(0)
    time.sleep(0.8)
    right_distance = Distance_test()

    # 舵机旋转到180度，即左侧，测距
    servo_appointed_detection(180)
    time.sleep(0.8)
    left_distance = Distance_test()

    # 舵机旋转到90度，即前方，测距
    servo_appointed_detection(90)
    time.sleep(0.8)
    front_distance = Distance_test()
    stop_signal = False

    if left_distance < 30 and right_distance < 30 and front_distance < 30:
        # 亮品红色，掉头
        GPIO.output(LED_R, GPIO.HIGH)
        GPIO.output(LED_G, GPIO.LOW)
        GPIO.output(LED_B, GPIO.HIGH)
        spin_right(speed)
        time.sleep(0.58)
    elif left_distance >= right_distance:
        # 亮蓝色
        GPIO.output(LED_R, GPIO.LOW)
        GPIO.output(LED_G, GPIO.LOW)
        GPIO.output(LED_B, GPIO.HIGH)
        spin_left(speed)
        time.sleep(0.28)
    elif left_distance <= right_distance:
        # 亮品红色，向右转
        GPIO.output(LED_R, GPIO.HIGH)
        GPIO.output(LED_G, GPIO.LOW)
        GPIO.output(LED_B, GPIO.HIGH)
        spin_right(speed)
        time.sleep(0.28)
    stop()

def get_away_from_obstacles():
    did_action = False
    distance = Distance_test()
    #print('distance:', distance)

    if distance < 30:
        servo_color_carstate(ratio=1)
    elif 30 <= distance <= 50:
        # 遇到障碍物,红外避障模块的指示灯亮,端口电平为LOW
        # 未遇到障碍物,红外避障模块的指示灯灭,端口电平为HIGH
        LeftSensorValue = GPIO.input(AvoidSensorLeft)
        RightSensorValue = GPIO.input(AvoidSensorRight)

        while (LeftSensorValue == True and RightSensorValue == False):
            did_action = True
            spin_left(65)  # 右边探测到有障碍物，有信号返回，原地向左转
            time.sleep(0.002)
            LeftSensorValue = GPIO.input(AvoidSensorLeft)
            RightSensorValue = GPIO.input(AvoidSensorRight)
        while (RightSensorValue == True and LeftSensorValue == False):
            did_action = True
            spin_right(65)  # 左边探测到有障碍物，有信号返回，原地向右转
            time.sleep(0.002)
            LeftSensorValue = GPIO.input(AvoidSensorLeft)
            RightSensorValue = GPIO.input(AvoidSensorRight)
        while (RightSensorValue == False and LeftSensorValue == False):
            did_action = True
            spin_right(65)  # 当两侧均检测到障碍物时调用固定方向的避障(原地右转)
            time.sleep(0.002)
            LeftSensorValue = GPIO.input(AvoidSensorLeft)
            RightSensorValue = GPIO.input(AvoidSensorRight)
    elif distance > 50:
        # 遇到障碍物,红外避障模块的指示灯亮,端口电平为LOW
        # 未遇到障碍物,红外避障模块的指示灯灭,端口电平为HIGH
        LeftSensorValue = GPIO.input(AvoidSensorLeft)
        RightSensorValue = GPIO.input(AvoidSensorRight)

        while (LeftSensorValue == True and RightSensorValue == False):
            did_action = True
            spin_left(65)  # 右边探测到有障碍物，有信号返回，原地向左转
            time.sleep(0.002)
            LeftSensorValue = GPIO.input(AvoidSensorLeft)
            RightSensorValue = GPIO.input(AvoidSensorRight)
        while (RightSensorValue == True and LeftSensorValue == False):
            did_action = True
            spin_right(65)  # 左边探测到有障碍物，有信号返回，原地向右转
            time.sleep(0.002)
            LeftSensorValue = GPIO.input(AvoidSensorLeft)
            RightSensorValue = GPIO.input(AvoidSensorRight)
        while (RightSensorValue == False and LeftSensorValue == False):
            did_action = True
            spin_right(65)  # 当两侧均检测到障碍物时调用固定方向的避障(原地右转)
            time.sleep(0.002)
            LeftSensorValue = GPIO.input(AvoidSensorLeft)
            RightSensorValue = GPIO.input(AvoidSensorRight)

            GPIO.output(LED_R, GPIO.LOW)
            GPIO.output(LED_G, GPIO.HIGH)
            GPIO.output(LED_B, GPIO.LOW)

    if did_action:
        stop()


class Car:
    def __init__(self):
        motor_init()
        self._get_away_from_obstacles_is_running = False

    def action(self, action_name, speed=50, time=0):
        global stop_signal

        if stop_signal == False:
            if action_name == "go":
                go(speed, time)
            elif action_name == "back":
                back(speed, time)
            elif action_name == "left":
                spin_left(speed, time)
            elif action_name == "right":
                spin_right(speed, time)
            elif action_name == "stop":
                stop(speed, time)

    def get_away_from_obstacles(self, ratio=1, always=False):
        if always == False:
            servo_color_carstate(ratio)
        else:
            if self._get_away_from_obstacles_is_running == False:
                def func():
                    while 1:
                        time.sleep(0.3)
                        get_away_from_obstacles()
                threading.Thread(target=func).start()
                self._get_away_from_obstacles_is_running = True

    def start_camera(self):
        terminal.run_command("motion -b")

    def camera_action(self, action_name):
        right_angle = 90
        if action_name == "reset":
            rotate_camera_horizontally(right_angle)
            rotate_camera_vertically(right_angle)
        elif action_name == "left":
            rotate_camera_horizontally(0)
            rotate_camera_vertically(right_angle)
        elif action_name == "right":
            rotate_camera_horizontally(180)
            rotate_camera_vertically(right_angle)
        elif action_name == "up":
            rotate_camera_horizontally(right_angle)
            rotate_camera_vertically(180)
        elif action_name == "down":
            rotate_camera_horizontally(right_angle)
            rotate_camera_vertically(70)

    def stop_camera(self):
        terminal.run_command("pkill motion")

    def quit(self):
        self.action(action_name="stop")
        pwm_ENA.stop()
        pwm_ENB.stop()
        GPIO.cleanup() 
