import RPi.GPIO as GPIO
from websocket import create_connection
import time
import numpy as np
import json
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
TRIG = 4
ECHO = 17
motor_pin=26
GPIO.setup(TRIG,GPIO.OUT)
GPIO.setup(ECHO,GPIO.IN)
GPIO.setup(motor_pin,GPIO.OUT)

def connect():
    global ws
    ws = create_connection("ws://192.168.9.119:6969")
    print("Connection established")

def readt_the_distance():
    running= True
    while running:
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)
        while GPIO.input(ECHO)==0:
            pulse_start=time.time()
        while GPIO.input(ECHO)==1:
            pulse_end = time.time()
            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150
            distance = round(distance, 2)
        ws.send(json.dumps({'type': 'distance', 'value': distance}))
        running = False

def turn_motor():
    pwm_motor=GPIO.PWM(motor_pin,50)
    pwm_motor.start(2.5)
    min_duty_cycle = 2
    max_duty_cycle = 13
    while True:
        for i in np.arange (min_duty_cycle, max_duty_cycle, 0.2):
            pwm_motor.ChangeDutyCycle(i)
            motor_position = (i - min_duty_cycle) / (max_duty_cycle - min_duty_cycle) * 180
            ws.send(json.dumps({'type': 'degree', 'value': motor_position}))
            readt_the_distance()
            time.sleep(0.3)

def main():
    connect()
    turn_motor()

if __name__=="__main__": 
    main()