#LCD IMU

import numpy as np
import time
import math
import board
import digitalio
import busio
import adafruit_bno055
from IMU import IMU 
from BNO055_IMU import BNO055_IMU
from AHRS import AHRS


import digitalio
import adafruit_sharpmemorydisplay
import time

myIMU = BNO055_IMU()

#MSP
# magCalBias = np.array([-22.5, -3.6875, 14.15625+3.7])
# magCalScaleFactor = np.array([0.9871891891891892, 0.997296928327645, 1.063218920557914])
#BAY
magCalBias = np.array([-25.1875, -9.90625, 16.625])
magCalScaleFactor = np.array([0.9729456159822419, 0.972405990016639, 1.0473405017921147])
myIMU.magCalBias = magCalBias
myIMU.magCalScaleFactor = magCalScaleFactor

C_orient = np.zeros( (3,3) )
C_orient[0][2] = -1.0
C_orient[1][1] = -1.0
C_orient[2][0] = -1.0
myIMU.setOrientation(C_orient)
C_Device2Sensor = C_orient.transpose()

myAHRS = AHRS(myIMU)

# (t,r,a,m) = myIMU.sample()


# Colors
BLACK = 0
WHITE = 255

# Parameters to Change
BORDER = 3
FONTSIZE = 50

spi = busio.SPI(board.SCK, MOSI=board.MOSI)
scs = digitalio.DigitalInOut(board.D22)  # inverted chip select

# display = adafruit_sharpmemorydisplay.SharpMemoryDisplay(spi, scs, 96, 96)
display = adafruit_sharpmemorydisplay.SharpMemoryDisplay(spi, scs, 400, 240)

# Clear display.
display.fill(1)
display.show()

#
# Added Setup
#
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

print("Warming Up...")
myAHRS.warmup(100)

print("Aligning ...")
myAHRS.align(100)


count=0
(t,r,a,m) = myIMU.sample()

while(True):
  
  count=count+1
  (t,r,a,m) = myIMU.sample()


#   att = myAHRS.update()
#   r = myAHRS.gyroRaw
#   a = myAHRS.accelRaw
#   m = myAHRS.magRaw
#   t = myAHRS.elapsedTime
#   print("RawMag {0}".format(mSensor))
  
  r_degPerSec = np.degrees(r)
  mSensor = np.matmul(m,C_Device2Sensor)
  att = np.array([0.0,0.0,0.0])
  att_deg = np.degrees(att)

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
      
  xSpan = xMax-xMin
  ySpan = yMax-yMin
  zSpan = zMax-zMin
  xBiasEst = xMin+xSpan/2
  yBiasEst = yMin+ySpan/2
  zBiasEst = zMin+zSpan/2
#   xSFEst = horzMag/(xSpan/2)
#   ySFEst = horzMag/(ySpan/2)
#   zSFEst = vertMag/(zSpan/2)
  xSFEst = fieldMag/(xSpan/2)
  ySFEst = fieldMag/(ySpan/2)
  zSFEst = fieldMag/(zSpan/2)

  attString = "ATT: {0:+4.1f} {1:+5.1f} {2:+5.1f}".format(att_deg[0],att_deg[1],att_deg[2])
  rotString = "ROT: {0:+6.1f} {1:+6.1f} {2:+6.1f}".format(r_degPerSec[0],r_degPerSec[1],r_degPerSec[2])
  accString = "ACC: {0:+6.1f} {1:+6.1f} {2:+6.1f}".format(a[0],a[1],a[2])
  magString = "MAG: {0:+6.1f} {1:+6.1f} {2:+6.1f}".format(m[0],m[1],m[2])
  flgString = "FLG: ____ ____ ____"
  timeString = "TIME: {0:.2f}".format(t)

  print("-------------------------------")
  print(attString)
  print(rotString)
  print(accString)
  print(magString)
  print(flgString)
  print(timeString)
  print("X   : {0} {1}".format(xMin,xMax))
  print("Y   : {0} {1}".format(yMin,yMax))
  print("Z   : {0} {1}".format(zMin,zMax))
  print("Bias: {0} {1} {2}".format(xBiasEst,yBiasEst,zBiasEst))
  print("SF  : {0} {1} {2}".format(xSFEst,ySFEst,zSFEst))
  print(magString)
  print(math.degrees(math.atan2(-1*m[1],m[0])))
#   print(myIMU.sensor.calibration_status)
#   text = str(count)
#   (font_width, font_height) = counterFont.getsize(text)
#   draw.text(
#      (display.width // 2 - font_width // 2, display.height // 2 - font_height // 2),
#       text,
#       font=counterFont,
#       fill=BLACK,
#   )
  
  #draw.text(
  #    (15, 15),
  #    "DasBoot AP:",
  #    font=font,
  #    fill=BLACK,
  #)

  # Display image
 
  #display.image(image)
  #display.show()
  display.fill(1)
  display.text(attString, 5, 10, 0, size=3)
  
  display.text(rotString, 10, 65, 0, size=2)
  display.text(accString, 10, 105, 0, size=2)
  display.text(magString, 10, 145, 0, size=2)
  display.text(flgString, 10, 185, 0, size=2)
  display.text(timeString,  10, 220, 0, size=2)
  display.show()
  
  #print(count)
  #display.fill(1)
