import RPi.GPIO as GPIO
import time

A_pin = 7
B_pin = 8
C_pin = 10
D_pin = 22
E_pin = 27
F_pin = 17

GPIO.setmode(GPIO.BCM)

GPIO.setup(A_pin, GPIO.IN)
GPIO.setup(B_pin, GPIO.IN)
GPIO.setup(C_pin, GPIO.IN)
GPIO.setup(D_pin, GPIO.IN)
GPIO.setup(E_pin, GPIO.IN)
GPIO.setup(F_pin, GPIO.IN)

while 1:
    print(f"""
A: {GPIO.input(A_pin)}
B: {GPIO.input(B_pin)}
C: {GPIO.input(C_pin)}
D: {GPIO.input(D_pin)}
E: {GPIO.input(E_pin)}
F: {GPIO.input(F_pin)}
    """)
    print(f"\n-------------------\n")
    time.sleep(1)
