import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)

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

GPIO.setup(A_pin, GPIO.IN)
GPIO.setup(B_pin, GPIO.IN)
GPIO.setup(C_pin, GPIO.IN)
GPIO.setup(D_pin, GPIO.IN)
GPIO.setup(E_pin, GPIO.IN)
GPIO.setup(F_pin, GPIO.IN)


# --------------------------------
# --------------------------------
# Ultrasonic sensor functions
# --------------------------------
# --------------------------------

Left_Echo = 21
Left_Triger = 20
Right_Echo = 23
Right_Triger = 24

GPIO.setup(Left_Echo, GPIO.IN)
GPIO.setup(Left_Triger, GPIO.OUT)
GPIO.setup(Right_Echo, GPIO.IN)
GPIO.setup(Right_Triger, GPIO.OUT)


GPIO.cleanup()
