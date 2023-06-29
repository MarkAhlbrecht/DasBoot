#DasBoot Pilot
import RPi.GPIO as GPIO
import time

from adafruit_seesaw import seesaw, rotaryio, digitalio
import board
import busio
#import digitalio
import adafruit_sharpmemorydisplay
import time
import adafruit_seesaw

# from adafruit_seesaw import seesaw, rotaryio

import numpy as np
import time
import math
from math import cos, sin, radians, degrees, sqrt, atan2, asin
import board
import digitalio
import busio
import adafruit_bno055
from IMU import IMU 
from BNO055_IMU import BNO055_IMU
from AHRS import AHRS

import PWM_Motor
from time import sleep

import PID

def euler2C(pitch,roll,yaw):
        C = np.zeros( (3,3) )
        cTheta = cos(pitch)
        sTheta = sin(pitch)
        cPhi = cos(roll)
        sPhi = sin(roll)
        cPsi = cos(yaw)
        sPsi = sin(yaw)
        C[0,0] = cTheta*cPsi
        C[0,1] = -1*cPhi*sPsi + sPhi*sTheta*cPsi
        C[0,2] = sPhi*sPsi + cPhi*sTheta*cPsi
        C[1,0] = cTheta*sPsi
        C[1,1] = cPhi*cPsi + sPhi*sTheta*sPsi
        C[1,2] = -sPhi*cPsi + cPhi*sTheta*sPsi
        C[2,0] = -sTheta
        C[2,1] = sPhi*cTheta
        C[2,2] = cPhi*cTheta
        return C

modes = [ "STANDBY", "AUTO", "TURN", "MANUAL","DATA","2DCAL","3DCAL","EXIT" ]
nModes = 8
targetMode = 0

mode = "DATA"

screenMode = "LCD"
# screenMode = "DATA"


###
### ENCODER SETUP
###
seesaw = seesaw.Seesaw(board.I2C(), addr=0x36)

seesaw_product = (seesaw.get_version() >> 16) & 0xFFFF
#print("Found product {}".format(seesaw_product))
if seesaw_product != 4991:
    print("Wrong firmware loaded?  Expected 4991")

seesaw.pin_mode(24, seesaw.INPUT_PULLUP)
button = adafruit_seesaw.digitalio.DigitalIO(seesaw, 24)
button_held = False

encoder = rotaryio.IncrementalEncoder(seesaw)
last_position = None
#
# Old Setup
#
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
prevEncoderCnt = 0
prevCntTime = 0
debounceTime = 0
encoderBtnCnt = 0
encoderLongPress = 15
encoderShortPress = 5
standbyDelay = 0
uiSamples = 20


###
### LCD Setup
###
# Colors
BLACK = 0
WHITE = 255

# Parameters to Change
BORDER = 3
FONTSIZE = 50

spi = busio.SPI(board.SCK, MOSI=board.MOSI)
scs = digitalio.DigitalInOut(board.D22)  # inverted chip select

display = adafruit_sharpmemorydisplay.SharpMemoryDisplay(spi, scs, 400, 240)
supressDisplay = False
blockDisplay = False
# Clear display.
display.fill(1)
display.show()

###
### AHRS/IMU SETUP
###
myIMU = BNO055_IMU()
# magCalBias = np.array([-22.5, -3.6875, 14.15625+3.7])
# magCalScaleFactor = np.array([0.9871891891891892, 0.997296928327645, 1.063218920557914])
# schooner
# magCalBias = np.array([-25.1875, -9.90625-1.5, 16.625])
# magCalScaleFactor = np.array([0.9729456159822419, 0.972405990016639, 1.0473405017921147])
magCalBias = np.array([-26.65625, -9.34375, 17.28125])
# magScaleErr = (1.0523976661500594, 1.0957015224724223, 0.9976980581639165)
# magScaleF = (1.0/magScaleErr[0], 1.0/magScaleErr[1], 1.0/magScaleErr[2])

# magCalBias = np.array([-19, 2.5, 16.625])
# magCalScaleFactor = np.array([1.0, 1.0, 1.0])

# magCalBias = np.array([0.0, 0.0, 0.0])
magCalScaleFactor = np.array([1.0, 1.0, 1.0])

magCalBiasAdj = np.array([ 1.70891235, -3.69128876, -5.60071556])
myIMU.magCalBias = magCalBias 
myIMU.magCalScaleFactor = magCalScaleFactor

C_orient = np.zeros( (3,3) )
C_orient[0][2] = -1.0
C_orient[1][1] = -1.0
C_orient[2][0] = -1.0
myIMU.setOrientation(C_orient)
C_Device2Sensor = C_orient.transpose()

myAHRS = AHRS(myIMU)

#
# Mag Cal
#
calFirstFrame = 0

horzMag = 17.679
vertMag = 51.9472
fieldMag = 54.789
xMax = -101
yMax = -101
zMax = -101
xMin = 100
yMin = 100
zMin = 100
xScale = 1
yScale = 1
zScale = 1
magErrorData = np.zeros((12,3))
magErrorSamples = np.zeros((12,1))
magErrorRanges = np.arange(-5,+7,1)*30.0 - 15.0
magErrorBound = 5.0
print(magErrorRanges)

prevAttDeg = np.array([0.0, 0.0, 0.0])
prevTime = 0;
turnRateDegF =0

print("Warming Up...")
myAHRS.warmup(100)

print("Aligning ...")
myAHRS.align(100)

###
### Motor Setup
###
motorDelayCnt = 0
motorPeriod = 5

motorDeadband = 10
motor = PWM_Motor.PWM_Motor(fwdGPIO=19, revGPIO=26, pwdGPIO=13, deadband=motorDeadband, scaleFactor=1)

hdgTarget = 180

turnRateSelects = range(-15, 18, 3) #-15 to 15
turnRateSelectIdx = 5

###
### Controller
###
Kp = -5
Ki = -5
Kd = 0
hdgController = PID.PID()
hdgController.SetKp(Kp)
hdgController.SetKi(Ki)
hdgController.SetKd(Kd)
hdgControllerDelay = 50

turnKp = 50
turnKi = 0
turnKd = 0
turnController = PID.PID()
turnController.SetKp(Kp)
turnController.SetKi(Ki)
turnController.SetKd(Kd)

###
### Logging
###
# logFileEnd = str(int(time.time() - 1628000000))
# logFileName = "/home/pi/Projects/DasBoot/data/DBRaw_" + logFileEnd + ".csv"
# logFile=open(logFileName,"w+")




###
### Main Loop
###

while True:

    # Common Pre-Processing
    startPerfTime = time.perf_counter()
    
    frameCnt +=1
    
    ###
    ### UI Control
    ###
    # negate the position to make clockwise rotation positive
    position = -encoder.position

    if position != last_position:
        last_position = position
        print("Position: {}".format(position))

    if not button.value and not button_held:
        button_held = True
        print("Button pressed")

    if button.value and button_held:
        button_held = False
        print("Button released")
    
    encoderCnt = position
    resetBtn = GPIO.input(18)
    #OLD
#     for s in range(uiSamples):
#         #time.sleep(0.01)
#         encoderA = GPIO.input(16)
#         encoderB = GPIO.input(20)
#         resetBtn = GPIO.input(18)
#         encoderBtn = GPIO.input(12)

#         if (encoderBtn == True):
#             if (encoderA == False and prevEncA == True and encoderB == True and prevEncB == True):
#                 
#                 if (frameCnt > prevCntTime + debounceTime):
#                     encoderCnt += 1
#                     #print(encoderCnt)
#                     prevCntTime = frameCnt
#                     break
# 
#             if (encoderB == False and prevEncB == True and encoderA == True and prevEncA == True):
#                 if (frameCnt > prevCntTime + debounceTime):
#                     encoderCnt -= 1
#                     #print(encoderCnt)
#                     prevCntTime = frameCnt
#                     break
#             
#             encoderBtnCnt = 0
#             
#         if (encoderBtn == False):
#                 encoderBtnCnt += 1
#                 break
        
    encoderBtn = button.value    
    if (not button.value):
        encoderBtnCnt += 1
    else:
        encoderBtnCnt =0
            
    if (resetBtn == False):
            mode = "EXIT"
    print(f"{encoderCnt} {encoderBtn} {encoderBtnCnt}")

    if (encoderBtnCnt > encoderLongPress):
        mode = "MODE_SEL"
        

    
    if(standbyDelay > 0):
        standbyDelay -=1
            
    ###
    ### Init Screen
    ###
    if blockDisplay:
        supressDisplay = True
    else:
        display.fill(1)
    
    
    # Update AHRS
    att = myAHRS.update()
    r = myAHRS.gyroRaw
    a = myAHRS.accelRaw
    m = myAHRS.magRaw
    t = myAHRS.elapsedTime
    temp = myIMU.sensor.temperature
    print(f"TEMP {temp}")

    logString = "0,{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12}".format(\
                      t,\
                      att[0],att[1],att[2],\
                      r[0],r[1],r[2],\
                      a[0],a[1],a[2],\
                      m[0],m[1],m[2]) 

#     logFile.write(logString)
#     logFile.write("\r\n")
#   print("RawMag {0}".format(mSensor))

    


    r_degPerSec = np.degrees(r)
    mSensor = np.matmul(m,C_Device2Sensor)
    att_deg = np.degrees(att)
    turnRateDeg = (att_deg[2] - prevAttDeg[2])/(t-prevTime);
    tauTurnRate = 3
    turnRateDegF = turnRateDegF*((tauTurnRate-1)/tauTurnRate) + \
                   (turnRateDeg)*(1/tauTurnRate);
    
    prevAttDeg = att_deg;
    ###
    ### Mode Select
    ###
    if (mode == "MODE_SEL"):
        
#         print(f"Encoder: {encoderCnt}")
#         if (encoderCnt > prevEncoderCnt):
        if (encoderCnt > prevEncoderCnt):
            targetMode +=1
            if(targetMode >= nModes):
                targetMode = 0
        if (encoderCnt < prevEncoderCnt):
            targetMode -=1
            if(targetMode < 0):
                targetMode = nModes-1
        print(f"Mode: {mode} -> TargetMode: {modes[targetMode]}")
        for m in range(nModes):
            if(m==targetMode):
                display.text("==> "+ modes[m], 20, 20+25*m, 0, size=2)
            else:
                display.text(modes[m], 20, 20+25*m, 0, size=2)
            
        if (encoderBtn == False and encoderBtnCnt == 1):
            mode = modes[targetMode]
        
        #
        # Mode specific Initializations
        #
        calFirstFrame = 0
    
    #mode = "STANDBY"        
    elif (mode == "STANDBY"):
#         print(f"Mode: {mode}")

        display.text(mode, 20, 20, 0, size=2)
        if (encoderCnt > prevEncoderCnt):
            hdgTarget += 10
            if(hdgTarget >180):
                hdgTarget -= 360

        if (encoderCnt < prevEncoderCnt):
            hdgTarget -= 10
            if(hdgTarget <-179):
                hdgTarget += 360
        
        hdgTarget360 = hdgTarget
        if(hdgTarget360 < 0):
            hdgTarget360 = 360 + hdgTarget360
        targetString = "  {0:03d}  ".format(int(hdgTarget360))
        
        hdg360 = att_deg[2]
        if(hdg360 < 0):
            hdg360 = 360 + hdg360        
        hdgString = "- {0:03d} -".format(int(hdg360))
        
        print(hdgString)
        print(targetString)
        
        display.text(hdgString, 40, 40, 0, size=8)
        display.text(targetString, 40, 120, 0, size=8)
        
        if (encoderBtn == False and encoderBtnCnt == 1):
            hdgTarget = att_deg[2]
 
        if (encoderBtn == False and encoderBtnCnt > encoderShortPress and standbyDelay == 0):
            mode = "AUTO"
            standbyDelay = 15
 
    ###
    ### AUTO Mode
    ###
    elif (mode == "AUTO"):
#         print(f"Mode: {mode}")
        display.text(mode, 20, 20, 0, size=2)


        if (encoderCnt > prevEncoderCnt):
            hdgTarget += 10
            if(hdgTarget >180):
                hdgTarget -= 360

        if (encoderCnt < prevEncoderCnt):
            hdgTarget -= 10
            if(hdgTarget <-179):
                hdgTarget += 360
        
        hdgTarget360 = hdgTarget
        if(hdgTarget360 < 0):
            hdgTarget360 = 360 + hdgTarget360
        
        
        hdg360 = att_deg[2]
        if(hdg360 < 0):
            hdg360 = 360 + hdg360        
        
 
        error = att_deg[2] - hdgTarget
        error = float(error)
        if error > 180:
            error = error - 360
        elif error <= -180:
            error = error + 360
        
        ctrlOutput = hdgController.GenOut(error)
        ctlPreString = "  "
        ctlPostString = "  "
        if(ctrlOutput > motorDeadband):
            ctlPreString = "=>"
            ctlPostString = "  "
        if(ctrlOutput < -1*motorDeadband):
            ctlPreString = "  "
            ctlPostString = "<="        
        targetString = "  {0:03d}  ".format(int(hdgTarget360))
        hdgString = "{0}{1:03d}{2}".format(ctlPreString,int(hdg360),ctlPostString)
        
#         print(hdgString)
#         print(targetString)
#         print(f"Error {error}")
#         print(f"Drive {ctrlOutput}")
        
        display.text(hdgString, 40, 40, 0, size=8)
        display.text(targetString, 40, 120, 0, size=8)
        if (frameCnt % hdgControllerDelay ==0):
            (drive1,drive2,motorPwm) = motor.drive(ctrlOutput)
        
        if (encoderBtn == False and encoderBtnCnt == 1):
            hdgTarget = att_deg[2]
 
        if (encoderBtn == False and encoderBtnCnt > encoderShortPress and standbyDelay == 0):
            mode = "STANDBY"
            standbyDelay = 15
    
    ###
    ### Turn Mode
    ###
    elif (mode == "TURN"):
        print(f"Mode: {mode}")
        
        if (encoderCnt > prevEncoderCnt):
            turnRateSelectIdx += 1
            if turnRateSelectIdx > 10: turnRateSelectIdx=10

        if (encoderCnt < prevEncoderCnt):
            turnRateSelectIdx -= 1
            if turnRateSelectIdx < 0: turnRateSelectIdx=0

        if (encoderBtn == False and encoderBtnCnt == 1):
            turnRateSelectIdx = 5

        turnRateTarget = turnRateSelects[turnRateSelectIdx]
        error = turnRateDegF - turnRateTarget
        error = float(error)
        
        ctrlOutput = turnController.GenOut(error)
        (drive1,drive2,motorPwm) = motor.drive(ctrlOutput)
        
        attString = "ATT: {0:+4.1f} {1:+5.1f} {2:+5.1f}".format(att_deg[0],att_deg[1],att_deg[2])
        rotString = "ROT: {0:+6.1f} {1:+6.1f} {2:+6.1f}".format(r_degPerSec[0],r_degPerSec[1],r_degPerSec[2])
        accString = "ACC: {0:+6.1f} {1:+6.1f} {2:+6.1f}".format(a[0],a[1],a[2])
        magString = "MAG: {0:+6.1f} {1:+6.1f} {2:+6.1f}".format(m[0],m[1],m[2])
        flgString = "DRV: {0:+4.1f} {1:+4.1f} {2:+4.1f}".format(turnRateTarget,turnRateDegF,ctrlOutput)
        timeString = "TIME: {0:.2f} {1:+4.1f}".format(t,temp)
        
        if screenMode == "DATA":
            print(logString)
        else:
            print("-------------------------------")
            print(attString)
            print(rotString)
            print(accString)
            print(magString)
            print(flgString)
            print(timeString)
        display.text(attString, 5, 10, 0, size=3)          
        display.text(rotString, 10, 65, 0, size=2)
        display.text(accString, 10, 105, 0, size=2)
        display.text(magString, 10, 145, 0, size=2)
        display.text(flgString, 10, 185, 0, size=3)
        display.text(timeString, 10, 220, 0, size=2)
    ###
    ### Manual Mode
    ###        
    elif (mode == "MANUAL"):
        print(f"Mode: {mode}")        
        if (encoderCnt > prevEncoderCnt):
            motorDelayCnt += motorPeriod

        if (encoderCnt < prevEncoderCnt):
            motorDelayCnt -= motorPeriod

        if (encoderBtn == False and encoderBtnCnt == 1):
            motorDelayCnt = 0
        
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

        motorString = "DRV: {0:+4d}".format(motorDelayCnt)
#         print(motorString)
        display.text(motorString, 5, 10, 0, size=3)
        
        if (frameCnt % 4):
            supressDisplay = False
        else:
            supressDisplay = True
        
    elif (mode == "DATA"):

        attString = "ATT: {0:+4.1f} {1:+5.1f} {2:+5.1f}".format(att_deg[0],att_deg[1],att_deg[2])
        rotString = "ROT: {0:+6.1f} {1:+6.1f} {2:+6.1f}".format(r_degPerSec[0],r_degPerSec[1],r_degPerSec[2])
        accString = "ACC: {0:+6.1f} {1:+6.1f} {2:+6.1f}".format(a[0],a[1],a[2])
        magString = "MAG: {0:+6.1f} {1:+6.1f} {2:+6.1f}".format(m[0],m[1],m[2])
        flgString = "FLG: ____ ____ {1:+4.1f}".format(temp,turnRateDegF)
        timeString = "TIME: {0:.2f} {1:+4.1f}".format(t,temp)
        
        if screenMode == "DATA":
            print(logString)
        else:
            print(f"Mode: {mode}")
            print("-------------------------------")
            print(attString)
            print(rotString)
            print(accString)
            print(magString)
            print(flgString)
            print(timeString)
        display.text(attString, 5, 10, 0, size=3)          
        display.text(rotString, 10, 65, 0, size=2)
        display.text(accString, 10, 105, 0, size=2)
        display.text(magString, 10, 145, 0, size=2)
        display.text(flgString, 10, 185, 0, size=2)
        display.text(timeString, 10, 220, 0, size=2)
        
    elif (mode == "2DCAL"):
        print(f"Mode: {mode}")
        attString = "ATT: {0:+4.1f} {1:+5.1f} {2:+5.1f}".format(att_deg[0],att_deg[1],att_deg[2])
        magString = "MAG: {0:+6.1f} {1:+6.1f} {2:+6.1f}".format(m[0],m[1],m[2])
        print(attString)
        print(magString)
        C_Local = euler2C(att[0],att[1],att[2])
        magL = np.matmul(C_Local,m)
        magTrueNav = np.array([horzMag,0,vertMag])
        magL = np.matmul(np.transpose(C_Local),magTrueNav)
        magLocalString = "M_L: {0:+6.1f} {1:+6.1f} {2:+6.1f}".format(magL[0],magL[1],magL[2])
        print(magLocalString)
        magTrueNav = np.array([horzMag,0,vertMag])
        magTrue = np.matmul(np.transpose(C_Local),magTrueNav)
        magTrueString = "M_T: {0:+6.1f} {1:+6.1f} {2:+6.1f}".format(magTrue[0],magTrue[1],magTrue[2])
        print(magTrueString)
#         magError = magL - magTrue
        magError = m - magTrue
        magErrorString = "errM: {0:+6.1f} {1:+6.1f} {2:+6.1f}".format(magError[0],magError[1],magError[2])
        print(magErrorString)
        
        for h in range(0,12):
            if ( (att_deg[2] > magErrorRanges[h]) and (att_deg[2] < (magErrorRanges[h] + magErrorBound)) ):
                if magErrorSamples[h] == 0:
                    magErrorSamples[h] += 1
                    magErrorData[h] = magError
                else:
                    magErrorSamples[h] += 1
                    gain = 1.0/magErrorSamples[h]
                    magErrorData[h] = 0.9*gain*magErrorData[h] + (1.0 - gain)*magError
                
            print(f"{magErrorRanges[h]} {magErrorSamples[h]} {magErrorData[h]}")
            
        meanErrSum = np.sum( magErrorData, axis=0)
        meanErrSamples = np.sum(magErrorSamples, axis=0)
        biasEst = meanErrSum/12
        print(f"Stat: {meanErrSum} {meanErrSamples} {biasEst}")
#         print(magTrue)
#         magLocal = 

#         if( calFirstFrame == 0):
#             horzMag = 17.679
#             vertMag = 51.9472
#             fieldMag = 54.789
#             xMax = -101
#             yMax = -101
#             zMax = -101
#             xMin = 100
#             yMin = 100
#             zMin = 100
#             xScale = 1
#             yScale = 1
#             zScale = 1
#             calFirstFrame = frameCnt
#         
#         if mSensor[0] > xMax:
#             xMax = mSensor[0]
#         if mSensor[1] > yMax:
#             yMax = mSensor[1]
#         if mSensor[2] > zMax:
#             zMax = mSensor[2]    
#         if mSensor[0] < xMin:
#             xMin = mSensor[0]
#         if mSensor[1] < yMin:
#             yMin = mSensor[1]
#         if mSensor[2] < zMin:
#             zMin = mSensor[2]
#       
#         xSpan = xMax-xMin+0.01
#         ySpan = yMax-yMin+0.01
#         zSpan = zMax-zMin+0.01
#         xBiasEst = xMin+xSpan/2
#         yBiasEst = yMin+ySpan/2
#         zBiasEst = zMin+zSpan/2
#         xSFEst = horzMag/(xSpan/2)
#         ySFEst = horzMag/(ySpan/2)
#         zSFEst = vertMag/(zSpan/2)
# 
#         xString = "X   : {0:+6.1f} {1:+6.1f}".format(xMin,xMax)
#         yString = "Y   : {0:+6.1f} {1:+6.1f}".format(yMin,yMax)
#         zString = "Z   : {0:+6.1f} {1:+6.1f}".format(zMin,zMax)
#         biasString = "Bias: {0:+6.2f} {1:+6.2f} {2:+6.2f}".format(xBiasEst,yBiasEst,zBiasEst)
#         sfString = "SF  : {0:+6.2f} {1:+6.2f} {2:+6.2f}".format(xSFEst,ySFEst,zSFEst)
#         hdgString = "mHDG: {0:+6.1f}".format(math.degrees(math.atan2(-1*m[1],m[0])))
#         
#         print(xString)
#         print(yString)
#         print(zString)
#         print(biasString)
#         print(sfString)
#         print(hdgString)
#         
#         display.text(xString, 5, 20, 0, size=2)          
#         display.text(yString, 10, 50, 0, size=2)
#         display.text(zString, 10, 80, 0, size=2)
#         display.text(biasString, 10, 110, 0, size=2)
#         display.text(sfString, 10, 140, 0, size=2)
#         display.text(hdgString, 10, 170, 0, size=2)
        
    elif (mode == "3DCAL"):
        print(f"Mode: {mode}")
        #display.text(mode, 20, 20, 0, size=2)
        if( calFirstFrame == 0):
            horzMag = 17.679
            vertMag = 51.9472
            fieldMag = 54.789
            xMax = -101
            yMax = -101
            zMax = -101
            xMin = 100
            yMin = 100
            zMin = 100
            xScale = 1
            yScale = 1
            zScale = 1
            calFirstFrame = frameCnt
        
        if mSensor[0] > xMax:
            xMax = mSensor[0]
        if mSensor[1] > yMax:
            yMax = mSensor[1]
        if mSensor[2] > zMax:
            zMax = mSensor[2]    
        if mSensor[0] < xMin:
            xMin = mSensor[0]
        if mSensor[1] < yMin:
            yMin = mSensor[1]
        if mSensor[2] < zMin:
            zMin = mSensor[2]
      
        xSpan = xMax-xMin+0.01
        ySpan = yMax-yMin+0.01
        zSpan = zMax-zMin+0.01
        xBiasEst = xMin+xSpan/2
        yBiasEst = yMin+ySpan/2
        zBiasEst = zMin+zSpan/2
        xSFEst = fieldMag/(xSpan/2)
        ySFEst = fieldMag/(ySpan/2)
        zSFEst = fieldMag/(zSpan/2)

        xString = "X   : {0:+6.1f} {1:+6.1f}".format(xMin,xMax)
        yString = "Y   : {0:+6.1f} {1:+6.1f}".format(yMin,yMax)
        zString = "Z   : {0:+6.1f} {1:+6.1f}".format(zMin,zMax)
        biasString = "Bias: {0:+6.2f} {1:+6.2f} {2:+6.2f}".format(xBiasEst,yBiasEst,zBiasEst)
        sfString = "SF  : {0:+6.2f} {1:+6.2f} {2:+6.2f}".format(xSFEst,ySFEst,zSFEst)
        hdgString = "mHDG: {0:+6.1f}".format(math.degrees(math.atan2(-1*m[1],m[0])))
        measString = "meas: {0:+6.2f} {1:+6.2f} {2:+6.2f}".format(mSensor[0],mSensor[1],mSensor[2]) 
        
        print(xString)
        print(yString)
        print(zString)
        print(biasString)
        print(sfString)
        print(hdgString)
        print(measString)
        
        display.text(xString, 5, 20, 0, size=2)          
        display.text(yString, 10, 50, 0, size=2)
        display.text(zString, 10, 80, 0, size=2)
        display.text(biasString, 10, 110, 0, size=2)
        display.text(sfString, 10, 140, 0, size=2)
        display.text(hdgString, 10, 170, 0, size=2)
        display.text(measString, 10, 200, 0, size=2)
        
    else:
        print(f"Mode: {mode}")
        display.text(mode, 20, 20, 0, size=2)
        display.text("EXIT", 20, 50, 0, size=3)
        display.show()
        break   
    
    # Common Post Processing
    # Ecoder
    prevEncoderCnt = encoderCnt
    
    if (mode != "AUTO" and mode != "MANUAL" and mode != "TURN"):
        motor.drive(0)

    # Draw Display
    if (not(supressDisplay)):
      display.show()
      pass
    supressDisplay = False
    
    prevTime = t;
    
    loopTime = time.perf_counter() - startPerfTime
#     print(loopTime)
# Finish
logFile.close()
print("All Done")    