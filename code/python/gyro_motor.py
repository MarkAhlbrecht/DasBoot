# gyro motor test
import PWM_Motor
from time import sleep

import numpy as np
import time
import math
import board
import digitalio
import busio
import adafruit_bno055
from IMU import IMU 
from BNO055_IMU import BNO055_IMU


motor = PWM_Motor.PWM_Motor(fwdGPIO=19, revGPIO=26, pwdGPIO=13, deadband=5, scaleFactor=1)
myIMU = BNO055_IMU()

gyro2MotorScale = 200

while True:
    (t,r,a,m) = myIMU.sample()
    gz = float(r[2])
    gy = float(r[1])
    gx = float(r[0])
    driveCmd = gx*gyro2MotorScale
    print("{:.4f} -> {:.0f}".format(gz,driveCmd))
    motor.drive(driveCmd)
    
