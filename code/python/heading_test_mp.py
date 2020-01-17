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
xBias = 40.59622859954834
yBias = 56.694664001464844
xSF = 0.6992602070692628
ySF = 0.7003758272879376
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



# """
# " Dials Objects
# """
# white = (255, 255, 255) 
# green = (0, 255, 0) 
# blue = (0, 0, 128)
# grey = (50, 50, 50)
# transparent = (255,255,0)
# orange = (255,128,0)
# 
# class Dial:
#    """
#    Generic dial type.
#    """
#    def __init__(self, image, frameImage, x=0, y=0, w=0, h=0):
#        """
#        x,y = coordinates of top left of dial.
#        w,h = Width and Height of dial.
#        """
#        self.x = x 
#        self.y = y
#        self.image = image
#        self.frameImage = frameImage
#        self.dial = pygame.Surface(self.frameImage.get_rect()[2:4])
#        self.dial.fill(0xFFFF00)
#        if(w==0):
#           w = self.frameImage.get_rect()[2]
#        if(h==0):
#           h = self.frameImage.get_rect()[3]
#        self.w = w
#        self.h = h
#        self.pos = self.dial.get_rect()
#        self.pos = self.pos.move(x, y)
# 
#    def position(self, x, y):
#        """
#        Reposition top,left of dial at x,y.
#        """
#        self.x = x 
#        self.y = y
#        self.pos[0] = x 
#        self.pos[1] = y 
# 
#    def position_center(self, x, y):
#        """
#        Reposition centre of dial at x,y.
#        """
#        self.x = x
#        self.y = y
#        self.pos[0] = x - self.pos[2]/2
#        self.pos[1] = y - self.pos[3]/2
# 
#    def rotate(self, image, angle):
#        """
#        Rotate supplied image by "angle" degrees.
#        This rotates round the centre of the image. 
#        If you need to offset the centre, resize the image using self.clip.
#        This is used to rotate dial needles and probably doesn't need to be used externally.
#        """
#        tmpImage = pygame.transform.rotate(image ,angle)
#        imageCentreX = tmpImage.get_rect()[0] + tmpImage.get_rect()[2]/2
#        imageCentreY = tmpImage.get_rect()[1] + tmpImage.get_rect()[3]/2
# 
#        targetWidth = tmpImage.get_rect()[2]
#        targetHeight = tmpImage.get_rect()[3]
# 
#        imageOut = pygame.Surface((targetWidth, targetHeight))
#        imageOut.fill(0xFFFF00)
#        imageOut.set_colorkey(0xFFFF00)
#        imageOut.blit(tmpImage,(0,0), pygame.Rect( imageCentreX-targetWidth/2,imageCentreY-targetHeight/2, targetWidth, targetHeight ) )
#        return imageOut
# 
#    def clip(self, image, x=0, y=0, w=0, h=0, oX=0, oY=0):
#        """
#        Cuts out a part of the needle image at x,y position to the correct size (w,h).
#        This is put on to "imageOut" at an offset of oX,oY if required.
#        This is used to centre dial needles and probably doesn't need to be used externally.       
#        """
#        if(w==0):
#            w = image.get_rect()[2]
#        if(h==0):
#            h = image.get_rect()[3]
#        needleW = w + 2*math.sqrt(oX*oX)
#        needleH = h + 2*math.sqrt(oY*oY)
#        imageOut = pygame.Surface((needleW, needleH))
#        imageOut.fill(0xFFFF00)
#        imageOut.set_colorkey(0xFFFF00)
#        imageOut.blit(image, (needleW/2-w/2+oX, needleH/2-h/2+oY), pygame.Rect(x,y,w,h))
#        return imageOut
# 
#    def overlay(self, image, x, y, r=0):
#        """
#        Overlays one image on top of another using 0xFFFF00 (Yellow) as the overlay colour.
#        """
#        x -= (image.get_rect()[2] - self.dial.get_rect()[2])/2
#        y -= (image.get_rect()[3] - self.dial.get_rect()[3])/2
#        image.set_colorkey(0xFFFF00)
#        self.dial.blit(image, (x,y))
# 
# class Heading(Dial):
#    """
#    Heading dial.
#    """
#    def __init__(self, x=0, y=0, w=0, h=0):
#        """
#        Initialise dial at x,y.
#        Default size of 300px can be overidden using w,h.
#        """
#        self.image = pygame.image.load('resources/HeadingWeel.png').convert()
#        self.frameImage = pygame.image.load('resources/HeadingIndicator_Background.png').convert()
#        self.vehicleImage = pygame.image.load('resources/HeadingIndicator_Boat.png').convert()
#        self.needleImage = pygame.image.load('resources/HeadingIndicator_Needle.png').convert()
#        Dial.__init__(self, self.image, self.frameImage, x, y, w, h)
#        self.font = pygame.font.Font('freesansbold.ttf', 18)
#        
#    def update(self, screen, angleX, setAngle):
#        """
#        Called to update an Artificial horizon dial.
#        "angleX" and "angleY" are the inputs.
#        "screen" is the surface to draw the dial on.
#        """
#        angleX %= 360
# #        angleY %= 360
#        if (angleX > 180):
#            angleX -= 360 
# #        if (angleY > 90)and(angleY < 270):
# #            angleY = 180 - angleY 
# #        elif (angleY > 270):
# #            angleY -= 360
# #        tmpImage = self.clip(self.image, 0, (59-angleY)*720/180, 250, 250)
# # create a text suface object, 
# # on which text is drawn on it.
#        displayAngle = angleX
#        if displayAngle < 0:
#            displayAngle +=360
#        if setAngle < 0:
#            setAngle +=360
#        text = self.font.render("{:03d}".format(int(displayAngle)), True, orange, grey)
#        textSetAngle = self.font.render("{:03d}".format(int(setAngle)), True, green, grey)
# # create a rectangular object for the 
# # text surface object 
#        textRect = text.get_rect()  
#   
# # set the center of the rectangular object. 
#        textRect.center = (150, 150)
# 
#        tmpImage = self.rotate(self.image, angleX)
#        tmpImage2 = self.rotate(self.needleImage, angleX - setAngle)
#        self.overlay(self.frameImage, 0,0)
#        self.overlay(tmpImage, 0, 0)
#        self.overlay(self.vehicleImage, 0,0)
#        self.overlay(tmpImage2, 0, 0)
#        self.overlay(text, 0, 20)
#        self.overlay(textSetAngle, 0, 40)
#        self.dial.set_colorkey(0xFFFF00)
#        screen.blit( pygame.transform.scale(self.dial,(self.w,self.h)), self.pos )
# 
# 
# 
# class Horizon(Dial):
#    """
#    Artificial horizon dial.
#    """
#    def __init__(self, x=0, y=0, w=0, h=0):
#        """
#        Initialise dial at x,y.
#        Default size of 300px can be overidden using w,h.
#        """
#        self.image = pygame.image.load('resources/Horizon_GroundSky.png').convert()
#        self.frameImage = pygame.image.load('resources/Horizon_Background.png').convert()
#        self.maquetteImage = pygame.image.load('resources/Maquette_Avion.png').convert()
#        Dial.__init__(self, self.image, self.frameImage, x, y, w, h)
#    def update(self, screen, angleX, angleY):
#        """
#        Called to update an Artificial horizon dial.
#        "angleX" and "angleY" are the inputs.
#        "screen" is the surface to draw the dial on.
#        """
#        angleX %= 360
#        angleY %= 360
#        if (angleX > 180):
#            angleX -= 360 
#        if (angleY > 90)and(angleY < 270):
#            angleY = 180 - angleY 
#        elif (angleY > 270):
#            angleY -= 360
#        tmpImage = self.clip(self.image, 0, (59-angleY)*720/180, 250, 250)
#        tmpImage = self.rotate(tmpImage, angleX)
#        self.overlay(tmpImage, 0, 0)
#        self.overlay(self.frameImage, 0,0)
#        self.overlay(self.maquetteImage, 0,0)
#        self.dial.set_colorkey(0xFFFF00)
#        screen.blit( pygame.transform.scale(self.dial,(self.w,self.h)), self.pos )
# 
# class TurnCoord(Dial):
#    """
#    Turn Coordinator dial.
#    """
#    def __init__(self, x=0, y=0, w=0, h=0):
#        """
#        Initialise dial at x,y.
#        Default size of 300px can be overidden using w,h.
#        """
#        self.image = pygame.image.load('resources/TurnCoordinatorAircraft.png').convert()
#        self.frameImage = pygame.image.load('resources/TurnCoordinator_Background.png').convert()
#        self.marks = pygame.image.load('resources/TurnCoordinatorMarks.png').convert()
#        self.ball = pygame.image.load('resources/TurnCoordinatorBall.png').convert()
#        Dial.__init__(self, self.image, self.frameImage, x, y, w, h)
#    def update(self, screen, angleX, angleY):
#        """
#        Called to update a Turn Coordinator dial.
#        "angleX" and "angleY" are the inputs.
#        "screen" is the surface to draw the dial on.       
#        """
#        angleX %= 360 
#        angleY %= 360
#        if (angleX > 180):
#            angleX -= 360 
#        if (angleY > 180):
#            angleY -= 360
#        if(angleY > 14): 
#            angleY = 14
#        if(angleY < -14): 
#            angleY = -14
#        tmpImage = self.clip(self.image, 0, 0, 0, 0, 0, -12)
#        tmpImage = self.rotate(tmpImage, angleX)
#        self.overlay(self.frameImage, 0,0)
#        self.overlay(tmpImage, 0, 0)
#        tmpImage = self.clip(self.marks, 0, 0, 0, 0, 0, 0)
#        self.overlay(tmpImage, 0, 80)
#        tmpImage = self.clip(self.ball, 0, 0, 0, 0, 0, 300)
#        tmpImage = self.rotate(tmpImage, angleY)
#        self.overlay(tmpImage, 0, -220)
#        self.dial.set_colorkey(0xFFFF00)
#        screen.blit( pygame.transform.scale(self.dial,(self.w,self.h)), self.pos )
# 
# class Generic(Dial):
#    """
#    Generic Dial. This is built on by other dials.
#    """
#    def __init__(self, x=0, y=0, w=0, h=0):
#        """
#        Initialise dial at x,y.
#        Default size of 300px can be overidden using w,h.       
#        """
#        self.image = pygame.image.load('resources/AirSpeedNeedle.png').convert()
#        self.frameImage = pygame.image.load('resources/Indicator_Background.png').convert()
#        Dial.__init__(self, self.image, self.frameImage, x, y, w, h)
#    def update(self, screen, angleX, iconLayer=0):
#        """
#        Called to update a Generic dial.
#        "angleX" and "angleY" are the inputs.
#        "screen" is the surface to draw the dial on.       
#        """
#        angleX %= 360
#        angleX = 360 - angleX
#        tmpImage = self.clip(self.image, 0, 0, 0, 0, 0, -35)
#        tmpImage = self.rotate(tmpImage, angleX)
#        self.overlay(self.frameImage, 0,0)
#        if iconLayer:
#           self.overlay(iconLayer[0],iconLayer[1],iconLayer[2])
#        self.overlay(tmpImage, 0, 0)
#        self.dial.set_colorkey(0xFFFF00)
#        screen.blit( pygame.transform.scale(self.dial,(self.w,self.h)), self.pos )
# 
# class Battery(Generic):
#    """
#    Battery dial.
#    """
#    def __init__(self, x=0, y=0, w=0, h=0):
#        """
#        Initialise dial at x,y.
#        Default size of 300px can be overidden using w,h.
#        """
#        self.icon = pygame.image.load('resources/battery2.png').convert()
#        Generic.__init__(self, x, y, w, h)
#        self.frameImage = pygame.image.load('resources/ledgend.png').convert()
#    def update(self, screen, angleX):
#        """
#        Called to update a Battery dial.
#        "angleX" is the input.
#        "screen" is the surface to draw the dial on.       
#        """
#        if angleX > 100:
#           angleX = 100
#        elif angleX < 0:
#           angleX = 0
#        angleX *= 2.7
#        angleX -= 135
#        Generic.update(self, screen, angleX, (self.icon, 0, 100))
# 
# class RfSignal(Generic):
#    """
#    RF Signal dial.
#    """
#    def __init__(self, x=0, y=0, w=0, h=0):
#        """
#        Initialise dial at x,y.
#        Default size of 300px can be overidden using w,h.
#        """
#        self.image = pygame.Surface((0,0))
#        self.frameImage = pygame.image.load('resources/RF_Dial_Background.png').convert()
#        Dial.__init__(self, self.image, self.frameImage, x, y, w, h)
#    def update(self, screen, inputA, inputB, scanPos):
#        """
#        "screen" is the surface to draw the dial on.       
#        """
# 
#        top = self.dial.get_rect()[0] +60
#        left = self.dial.get_rect()[1] +30
#        bottom = self.dial.get_rect()[0] + self.dial.get_rect()[2] -60
#        right = self.dial.get_rect()[1] + self.dial.get_rect()[3] -30
#        height = bottom - top
#        middle = height/2 + top
# 
#        scanPos %= right -30
#        scanPos += 30
#        inputA %= 100
#        inputB %= 100
#        inputA = height * inputA / 200
#        inputB = height * inputB / 200
# 
#        pygame.draw.line(self.dial, 0xFFFFFF, (scanPos,top), (scanPos,bottom), 1)
#        pygame.draw.line(self.dial, 0x222222, (scanPos-1,top), (scanPos-1,bottom), 1)
# 
#        pygame.draw.line(self.dial, 0x00FFFF, (scanPos-1,middle-inputA), (scanPos-1,middle),4)
#        pygame.draw.line(self.dial, 0xFF00FF, (scanPos-1,bottom-inputB), (scanPos-1,bottom),4)
#        pygame.draw.line(self.dial, 0xFFFF00, (scanPos-1,middle), (scanPos-1,middle))
# 
#        self.overlay(self.frameImage, 0,0)
# 
#        self.dial.set_colorkey(0xFFFF00)
#        screen.blit( pygame.transform.scale(self.dial,(self.w,self.h)), self.pos )
# 
# 



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

"""
" Motor driver
"""
def drive_motor(omegaMotor):
  deadBand = 12;
  Kpwm = 1;
  if abs(omegaMotor) > deadBand:
    drive1 = omegaMotor > 0
    drive2 = not drive1
    motorPwm = abs(Kpwm*omegaMotor)
    if motorPwm > 100:
      motorPwm = 100
    #
    # Command Motor Pins
    #
  else:
    drive1 = False
    drive2 = False
    motorPwm = 0
  #
  # Command Motor Pins
  #

  return (drive1,drive2,motorPwm)
  

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
GPIO.setmode(GPIO.BOARD)

GPIO.setup(11, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)
GPIO.setup(7, GPIO.OUT)
pwm=GPIO.PWM(7, 100)
pwm.start(0)
drive1 = 0
drive2 = 0
motorPwm = 0
pwm.ChangeDutyCycle(100)
GPIO.output(7, True)

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
    (drive1,drive2,motorPwm) = drive_motor(ctrlOutput)
    
  if enableMotor:
    GPIO.output(11, drive1)
    GPIO.output(12, drive2)
    pwm.ChangeDutyCycle(motorPwm)

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
          (drive1,drive2,motorPwm) = drive_motor(25)
          selection = True
        elif event.direction == "down":
          (drive1,drive2,motorPwm) = drive_motor(-25)
          selection = True
        elif event.direction == "middle":
#           runFlag = False
          (drive1,drive2,motorPwm) = drive_motor(0)
          controlEnable = not controlEnable
          GPIO.output(11, drive1)
          GPIO.output(12, drive2)
          pwm.ChangeDutyCycle(motorPwm)
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
           GPIO.output(11, False)
           GPIO.output(12, False)
           GPIO.output(7, False)
           pwm.stop()

           GPIO.cleanup()
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
GPIO.output(11, False)
GPIO.output(12, False)
GPIO.output(7, False)
pwm.stop()

GPIO.cleanup()
sense.clear()
sys.exit()   # end program.

