from sense_hat import SenseHat
import numpy as np
import time
import math
from math import cos, sin, radians, degrees, sqrt, atan2, asin
from dials import *
import pygame
from pygame.locals import *
pygame.init()
import sys
import PID
import RPi.GPIO as GPIO
import PWM_Motor

"""

  Heading Control Test 
  Note: Requires sense_hat 2.2.0 or later

"""
controlEnable = True
enableMotor = True
#
# Configuration
#
#####
# Predetermined Mag Calibraton
####
# Sense Hat 1
# xBias = -30.02851629257202
# yBias = 7.201869010925293 +1.4395752679386042
# xSF = 0.6773587707778925
# ySF = 0.7209744740877911

# Sense Hat 2
xBias = 2.701296329498291
yBias = 19.79104995727539
xSF = 0.6955512793747712
ySF = 0.7164366008777275
#
# AHRS Smoothing filter time constant
tau = 1/200

# Controller setup
setTilt = 0
# Kp = -30
# Kd = -3
# Ki = -0
Kp = -10
Ki = -2
Kd = 0
#
# Log file name
#
logFile=open("/home/pi/Projects/DasBoot/data/heading_ctrl_test.csv","w+")

motor = PWM_Motor.PWM_Motor(fwdGPIO=11, revGPIO=12, pwdGPIO=7, deadband=0, scaleFactor=1)





"""
"   c2euler
"   This routine converts euler measurments to C matrix representations
"   Input: pitch roll yaw
"   Output: C matrix
"""
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

"""
"   c2euler
"   This routine converts a C matrix represention to euler representation
"   Input: C matrix
"   Output: pitch roll yaw
"""
def c2euler(C):
    pitch = -1*asin(C[2,0])
    yaw = atan2(C[1,0],C[0,0])
    roll = atan2(C[2,1],C[2,2])
    return (pitch,roll,yaw)

"""
"   ortho_norm
"   This routine ortho normalizes a C matrix
"   Input: C matrix
"   Output: Ortho normed C matrix
"""
def ortho_norm(C):
    x = C[:,0]
    y = C[:,1]
    z = C[:,2]
    
    e = x.transpose()
    e = e.dot(y)
    eScalar = e/2
    
    xOrtho = x - e*y
    yOrtho = y - e*x
    zOrtho = np.cross(xOrtho,yOrtho)
    
    xNorm = 0.5*(3 - xOrtho.transpose().dot(xOrtho))*xOrtho
    yNorm = 0.5*(3 - yOrtho.transpose().dot(yOrtho))*yOrtho
    zNorm = 0.5*(3 - zOrtho.transpose().dot(zOrtho))*zOrtho
    
    Con = np.column_stack( (xNorm,yNorm,zNorm) )
    return Con

"""
" Update the sense hat screen
"""
def update_screen(angle, show_letter = False):
  
  yorig = 3
  xorig = 3
  ca = cos(radians(angle))
  sa = sin(radians(angle))
  sense.clear()
  
  
  for l in range(-10,0):
    x = int(ca*float(l)/2 + xorig)
    y = int(sa*float(l)/2 + yorig)
    if x >= 0 and x <= 7 and y >= 0 and y <= 7:
      sense.set_pixel(x,y,white)
  for l in range(0,10):
    x = int(ca*float(l)/2 + xorig)
    y = int(sa*float(l)/2 + yorig)
    if x >= 0 and x <= 7 and y >= 0 and y <= 7:
      sense.set_pixel(x,y,red)
  sense.set_pixel(xorig,yorig,green)

# """
# " Motor driver
# """
# def drive_motor(omegaMotor):
#   deadBand = 12;
#   Kpwm = 1;
#   if abs(omegaMotor) > deadBand:
#     drive1 = omegaMotor > 0
#     drive2 = not drive1
#     motorPwm = abs(Kpwm*omegaMotor)
#     if motorPwm > 100:
#       motorPwm = 100
#     #
#     # Command Motor Pins
#     #
#   else:
#     drive1 = False
#     drive2 = False
#     motorPwm = 0
#   #
#   # Command Motor Pins
#   #
# 
#   return (drive1,drive2,motorPwm)
  

"""
  Main Program
"""
# Initialization
sense = SenseHat()
sense.clear()

screen = pygame.display.set_mode((640, 480))
screen.fill(0x222222)
pygame.display.set_caption('Autopilot Test Display')
#    
# # Initialise Dials.
horizon = Horizon(320,20)
heading = Heading(20,20)

errorDial = Generic(20,340,100,100)
propDial = Generic(160,340,100,100)
integDial = Generic(270,340,100,100)
derivDial = Generic(380,340,100,100)
motorDial = Generic(520,340,100,100)


horizon.update(screen, 0, 0 )
heading.update(screen, 0, 0 )
errorDial.update(screen, 0 )
propDial.update(screen, 0 )
integDial.update(screen, 0 )
derivDial.update(screen, 0 )
motorDial.update(screen, 0 )


pygame.display.update()

#
# Setup Motor Control Pins
#

# GPIO.setmode(GPIO.BOARD)
# 
# GPIO.setup(11, GPIO.OUT)
# GPIO.setup(12, GPIO.OUT)
# GPIO.setup(7, GPIO.OUT)
# pwm=GPIO.PWM(7, 100)
# pwm.start(0)
# drive1 = 0
# drive2 = 0
# motorPwm = 0
# pwm.ChangeDutyCycle(100)
# GPIO.output(7, True)

#


# Controller
tiltController = PID.PID()
tiltController.SetKp(Kp)
tiltController.SetKi(Ki)
tiltController.SetKd(Kd)

# Time initialization
startTime = time.time()
prevUpdateTime = 0

# Warmup Period
# This allows some of the initial crazy samples to bleed out
print("Warming up ...")
for i in range(1,101):
  gyroRaw = sense.get_gyroscope_raw()
  accelRaw = sense.get_accelerometer_raw()
  magRaw = sense.get_compass_raw()
  temp = sense.get_temperature()
  
#
#   Alignment Period
#   Collect a set of data while the unit is motionless to establish
#   initial level and initial gyro bias
alignAccel = np.array([0,0,0])
alignGyro = np.array([0,0,0])
alignMag = np.array([0,0,0])

print("Aligning ...")
alignSamples = 100
for i in range(1,alignSamples+1):
    elapsedTime = time.time() - startTime  
    gyroRaw = sense.get_gyroscope_raw()
    accelRaw = sense.get_accelerometer_raw()
    magRaw = sense.get_compass_raw()
    temp = sense.get_temperature()
    magRaw["x"] = xSF *(magRaw["x"] - xBias)
    magRaw["y"] = ySF *(magRaw["y"] - yBias)

    logFile.write("0,{0},{x},{y},{z}\r\n".format(elapsedTime,**gyroRaw)) # rad/sec
    logFile.write("1,{0},{x},{y},{z}\r\n".format(elapsedTime,**accelRaw)) # Gs
    logFile.write("2,{0},{x},{y},{z}\r\n".format(elapsedTime,**magRaw)) # microT
    logFile.write("7,{0},{1}\r\n".format(elapsedTime,temp))
    
    alignAccel = alignAccel + \
               1/alignSamples*np.array([accelRaw["x"], accelRaw["y"],accelRaw["z"]])
    alignGyro = alignGyro + \
               1/alignSamples*np.array([gyroRaw["x"], gyroRaw["y"],gyroRaw["z"]])
    alignMag = alignMag + \
               1/alignSamples*np.array([magRaw["x"], magRaw["y"],magRaw["z"]])

# Establish initial prich roll from averaged accel measures
alignPitch = asin(-1*alignAccel[0])
alignRoll = atan2(alignAccel[1],1.0)
Ca = euler2C(alignPitch,alignRoll,0.0)

# Establish initial heading from horizontal mag measuremetns
magL = Ca.dot(alignMag.transpose())
yaw = atan2(-1*magL[1],magL[0])

# Initialize Gyro, Accel, and Filtered to Level transitions
C_AL = euler2C(alignPitch,alignRoll,yaw)
C_GL = euler2C(alignPitch,alignRoll,yaw)
C_FL = euler2C(alignPitch,alignRoll,yaw)

# Initial Gyro bias is the average over sample period
gyroBias = alignGyro

# Print out the results
(p,r,y) = c2euler(C_AL)
print(degrees(p))
print(degrees(r))
print(degrees(y))


# AHRS Loop

r180 = radians(180)
nr180 = radians(-180)
r360 = radians(360)

deltaTheta = np.array([0, 0 , 0])
accumTheta = np.array([0, 0 , 0])
omega = np.array( [0, 0, 0] )
deltaC = np.ones( (3,3) )

sense.clear()
prevUpdateTime = time.time() - startTime
prevTime = time.time() - startTime

runFlag = True
while runFlag:
 
  ######################
  # Fast Loop
  ######################
  elapsedTime = time.time() - startTime
  gyroRaw = sense.get_gyroscope_raw()
  accelRaw = sense.get_accelerometer_raw()
  magRaw = sense.get_compass_raw()
  temp = sense.get_temperature()

  
  logFile.write("0,{0},{x},{y},{z}\r\n".format(elapsedTime,**gyroRaw)) # rad/sec
  logFile.write("1,{0},{x},{y},{z}\r\n".format(elapsedTime,**accelRaw)) # Gs
  logFile.write("2,{0},{x},{y},{z}\r\n".format(elapsedTime,**magRaw)) # microT
  logFile.write("7,{0},{1}\r\n".format(elapsedTime,temp))
  magRaw["x"] = xSF *(magRaw["x"] - xBias)
  magRaw["y"] = ySF *(magRaw["y"] - yBias)
 
  deltaTime = elapsedTime - prevTime
  
#   print("0,{0},{x},{y},{z}".format(elapsedTime,**gyroRaw)) # rad/sec
#   print("1,{0},{x},{y},{z}".format(elapsedTime,**accelRaw)) # Gs
#   print("2,{0},{x},{y},{z}".format(elapsedTime,**magRaw))

# Accumplate Rates
  omega = np.array( (gyroRaw["x"], gyroRaw["y"], gyroRaw["z"]) ) - gyroBias
  deltaTheta = omega*deltaTime # Basic Integration
  accumTheta = accumTheta + deltaTheta 
#   print("Dt {0} {1} {2}".format(degrees(accumTheta[0]),degrees(accumTheta[1]),degrees(accumTheta[2])))

# Form the deltaC matrix from the angular rotations
# Single taylor series with small angle assumption
  deltaC[0,1] = -1*deltaTheta[2]
  deltaC[0,2] = deltaTheta[1]
  deltaC[1,0] = deltaTheta[2]
  deltaC[1,2] = -1*deltaTheta[0]
  deltaC[2,0] = -1*deltaTheta[1]
  deltaC[2,1] = deltaTheta[0]

# Update the gyro and filter solution
  C_GL = C_GL.dot(deltaC)
  C_FL = C_FL.dot(deltaC)

# Determine the Accel Solution
  accelPitch = asin(-1*accelRaw["x"])
  accelRoll = atan2(accelRaw["y"],1.0)
  accelMag = np.array([magRaw["x"], magRaw["y"],magRaw["z"]])
  C_AL = euler2C(accelPitch,accelRoll,0.0)
#   print(Ca)
  magL = C_AL.dot(accelMag.transpose())
#   print(magL)
  yaw = atan2(-1*magL[1],magL[0])
  C_AL = euler2C(accelPitch,accelRoll,yaw)
  
  # Update the filtered solution with the data from the
  # Accel based solution
  (pf,rf,yf) = c2euler(C_FL)
  pf = (1-tau)*pf + tau*accelPitch
  rf = (1-tau)*rf + tau*accelRoll
  ey = yf-yaw
  if ey > r180:
    ey = ey - r360
  elif ey < -r180:
    ey = ey + r360
  yf = (1-tau)*yf + tau*(yf - ey)
  if yf > r180:
      yf = yf-r360
  elif yf < -r180:
      yf = yf+r360
  C_FL = euler2C(pf,rf,yf)
#   print("Att --> {0} {1} {2}".format(degrees(pf),degrees(rf),degrees(yf)))

  # Controller
  error = degrees(yf) - setTilt
  if error > 180:
      error = error - 360
  elif error <= -180:
      error = error + 360
  ctrlOutput = tiltController.GenOut(error)
  dp = tiltController.Cp
  di = tiltController.Ki * tiltController.Ci
  dd = tiltController.Kd * tiltController.Cd

  
  # Motor Drive
  if controlEnable:
    (drive1,drive2,motorPwm) = motor.drive(ctrlOutput)
    
#   if enableMotor:
#     GPIO.output(11, drive1)
#     GPIO.output(12, drive2)
#     pwm.ChangeDutyCycle(motorPwm)

  # Manual Sense Controls
  selection = False
  events = sense.stick.get_events()
  for event in events:
      # Skip releases
      if event.action != "released":
        if event.direction == "left":
          setTilt -= 30
          selection = True
        elif event.direction == "right":
          setTilt += 30
          selection = True
        elif event.direction == "up":
          (drive1,drive2,motorPwm) = motor.drive(25)
          selection = True
        elif event.direction == "down":
          (drive1,drive2,motorPwm) = motor.drive(-25)
          selection = True
        elif event.direction == "middle":
#           runFlag = False
          (drive1,drive2,motorPwm) = motor.drive(0)
          controlEnable = not controlEnable
#           GPIO.output(11, drive1)
#           GPIO.output(12, drive2)
#           pwm.ChangeDutyCycle(motorPwm)
          selection = True

  if setTilt > 180:
    setTilt -= 360
  if setTilt < -180:
    setTilt += 360
#
  prevTime = elapsedTime
  
  print("Att    : {0:+.2f} {1:+.2f} {2:+.2f}".format(degrees(pf),degrees(rf),degrees(yf)))
  print("Ctl In : {0:+.2f} {1:+.2f} {2:+.2f}".format(degrees(yf),setTilt,error))
  print("Ctl Out: {0:+.2f} ->  {1:+.2f} {2:+.2f} {3:+.2f} = {4:+.2f}".format(error,dp,di,dd,ctrlOutput))
  print("Dt     : {0:+.2f}".format(deltaTime*1000))
  
  ###############################
  # Slow Loop
  ###############################
  if (elapsedTime - prevUpdateTime)>1:
#     print("0,{0},{x},{y},{z}".format(elapsedTime,**gyroRaw)) # rad/sec
#     print("1,{0},{x},{y},{z}".format(elapsedTime,**accelRaw)) # Gs
#     print("2,{0},{x},{y},{z}".format(elapsedTime,**magRaw))
#     print(Cg)
    C_GL = ortho_norm(C_GL)
    for event in pygame.event.get():
       if event.type == QUIT:
           print("Exiting....")
#            GPIO.output(11, False)
#            GPIO.output(12, False)
#            GPIO.output(7, False)
#            pwm.stop()
# 
#            GPIO.cleanup()
           motor.close()
           sense.clear()
           sys.exit()   # end program.
           
    horizon.update(screen, degrees(-1*rf), degrees(pf) )
    heading.update(screen, degrees(yf), setTilt)
    errorDial.update(screen, error )
    propDial.update(screen, dp*1.35 )
    integDial.update(screen, di*1.35 )
    derivDial.update(screen, dd*1.35 )
    motorDial.update(screen, ctrlOutput*1.35 )
    pygame.display.update()

    
#     Cf = ortho_norm(Cf)
#     print(Cg)
#     (p,r,y) = c2euler(C_GL)
#     (pf,rf,yf) = c2euler(C_FL)
#     print("Gyro : {0} {1} {2}".format(degrees(p),degrees(r),degrees(y)))
#     print("Accel: {0} {1} {2}".format(degrees(alignPitch),degrees(alignRoll),degrees(yaw)))
#     print("Att    : {0:+.2f} {1:+.2f} {2:+.2f}".format(degrees(pf),degrees(rf),degrees(yf)))
#     print("Ctl In : {0:+.2f} {1:+.2f} {2:+.2f}".format(degrees(yf),setTilt,error))
#     print("Ctl Out: {0:+.2f} ->  {1:+.2f} {2:+.2f} {3:+.2f} = {4:+.2f}".format(error,dp,di,dd,ctrlOutput))

    print("")
    prevUpdateTime = elapsedTime 

print("Exiting....")
# GPIO.output(11, False)
# GPIO.output(12, False)
# GPIO.output(7, False)
# pwm.stop()
motor.close()

GPIO.cleanup()
sense.clear()
sys.exit()   # end program.

