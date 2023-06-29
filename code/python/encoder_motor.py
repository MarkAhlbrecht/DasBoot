# Manual Motor Driver with encoder
import PWM_Motor
from time import sleep

import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.IN,pull_up_down=GPIO.PUD_UP)
GPIO.setup(12, GPIO.IN,pull_up_down=GPIO.PUD_UP)
GPIO.setup(16, GPIO.IN,pull_up_down=GPIO.PUD_UP)
GPIO.setup(20, GPIO.IN,pull_up_down=GPIO.PUD_UP)

prevReset = True
prevBtn = True
prevEncA = True
prevEncB = True

resetH2L = False
btnH2L = False
encoderAH2L = False
encoderBH2L = False
frameCnt = 0
resetDBTime = 0
encoderCnt = 0
prevCntTime = 0
debounceTime = 0

motorDelayCnt = 0
motorPeriod = 25


motor = PWM_Motor.PWM_Motor(fwdGPIO=19, revGPIO=26, pwdGPIO=13, deadband=5, scaleFactor=1)

while True:
    time.sleep(0.01)
    frameCnt +=1
    resetBtn = GPIO.input(18)
    encoderBtn = GPIO.input(12)
    encoderA = GPIO.input(16)
    encoderB = GPIO.input(20)

    if (encoderA == False and prevEncA == True and encoderB == True and prevEncB == True):
        
        if (frameCnt > prevCntTime + debounceTime):
            encoderCnt += 1
            #print(encoderCnt)
            prevCntTime = frameCnt
            motorDelayCnt += motorPeriod
    if (encoderB == False and prevEncB == True and encoderA == True and prevEncA == True):
        if (frameCnt > prevCntTime + debounceTime):
            encoderCnt -= 1
            #print(encoderCnt)
            prevCntTime = frameCnt
            motorDelayCnt -= motorPeriod
    
    print(f"{encoderCnt} {motorDelayCnt}")
    if (not(motorDelayCnt == 0)):
        if(motorDelayCnt > 0):
            # run motor
            motor.drive(100)
            motorDelayCnt -=1
        if(motorDelayCnt < 0):
            motor.drive(-100)
            motorDelayCnt +=1
    else:
        motor.drive(0)
    prevReset = resetBtn
    prevEncA = encoderA
    prevEncB = encoderB
    
GPIO.cleanup()