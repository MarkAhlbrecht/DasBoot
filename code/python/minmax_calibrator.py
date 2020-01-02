from sense_hat import SenseHat
import time
from math import cos, sin, radians, degrees, sqrt, atan2

hasMotor = False
applyCalibration = False
"""

  Min/Max Mag Calibration Routine
  
  Determines the Scale Factor and Bias Error in Mag Measurements
  using the min/max technique.   This requires the IMU to be level
  (pitch/roll = 0).
  
  The IMU needs to be spun in a at least one rotation to collect the
  Min, Max and Scale factor measurements in X and Y.
  
  Pressing the joystick middle position will rested the parameters.
  This will take another measurements.
  
  Once the measurements are know then can be placed back in this script
  and then tested with the parameters.   The xBias, yBias, xSF and ySF
  variables need to be set to the measured values and the applyCalibration
  parameter needs to be set to True
  
  Note: Requires sense_hat 2.2.0 or later

"""
logFile=open("/home/pi/Projects/DasBoot/data/calibrate_raw.csv","w+")

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

sense = SenseHat()
sense.clear()

if applyCalibration:
    xBias = 19.67
    yBias = 33.57
    xSF = 0.71
    ySF = 0.71
else:
    xBias = 0
    yBias = 0
    xSF = 1
    ySF = 1

startTime = time.time()
prevUpdateTime = 0

#
# Added Setup
#
horzMag = 11.8868
xMax = -100
yMax = -100
xMin = 100
yMin = 100
xScale = 1
yScale = 1

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


for i in range(1,101):
  gyroRaw = sense.get_gyroscope_raw()
  accelRaw = sense.get_accelerometer_raw()
  magRaw = sense.get_compass_raw()
  temp = sense.get_temperature()
####
# Main game loop
####
prevUpdateTime = time.time() - startTime
sense.clear()
while True:
 
  elapsedTime = time.time() - startTime
  gyroRaw = sense.get_gyroscope_raw()
  accelRaw = sense.get_accelerometer_raw()
  magRaw = sense.get_compass_raw()
  temp = sense.get_temperature()
  
#   xBias = 0.12429*temp + 11.50899
#   yBias = 0.14081*temp + 21.99626
#   magRaw["x"] = magRaw["x"] - xBias
#   magRaw["y"] = magRaw["y"] - yBias

#   print("0,{0},{x},{y},{z}".format(elapsedTime,**gyroRaw)) # rad/sec
#   print("1,{0},{x},{y},{z}".format(elapsedTime,**accelRaw)) # Gs
#   print("2,{0},{x},{y},{z}".format(elapsedTime,**magRaw))


  logFile.write("0,{0},{x},{y},{z}\r\n".format(elapsedTime,**gyroRaw)) # rad/sec
  logFile.write("1,{0},{x},{y},{z}\r\n".format(elapsedTime,**accelRaw)) # Gs
  logFile.write("2,{0},{x},{y},{z}\r\n".format(elapsedTime,**magRaw)) # microT
  logFile.write("7,{0},{1}\r\n".format(elapsedTime,temp))
  magRaw["x"] = xSF *(magRaw["x"] - xBias)
  magRaw["y"] = ySF *(magRaw["y"] - yBias)

  if magRaw["x"] > xMax:
      xMax = magRaw["x"]
  if magRaw["y"] > yMax:
      yMax = magRaw["y"]
  if magRaw["x"] < xMin:
      xMin = magRaw["x"]
  if magRaw["y"] < yMin:
      yMin = magRaw["y"]
  orientation = sense.get_orientation_degrees()
  if(orientation["pitch"] > 180):
      orientation["pitch"] = orientation["pitch"] - 360
  if(orientation["roll"] > 180):
      orientation["roll"] = orientation["roll"] - 360
      
#   magCalx = xSF *(magRaw["x"] - xBias)
#   magCaly = ySF *(magRaw["y"] - yBias)
#   orientation["yaw"] = degrees(atan2(-1*magCaly,magCalx))
  orientation["yaw"] = degrees(atan2(-1*magRaw["y"],magRaw["x"] ))
  logFile.write("3,{0},{pitch},{roll},{yaw}\r\n".format(elapsedTime,**orientation))
  
  selection = False
  events = sense.stick.get_events()
  for event in events:
      # Skip releases
      
      if event.action != "released":
        if event.direction == "left":
          xMax = -100
          yMax = -100
          xMin = 100
          yMin = 100
          xScale = 1
          yScale = 1
          selection = True
        elif event.direction == "right":
          xMax = -100
          yMax = -100
          xMin = 100
          yMin = 100
          xScale = 1
          yScale = 1
          selection = True
        elif event.direction == "up":
          (drive1,drive2,motorPwm) = drive_motor(25)
          selection = True
        elif event.direction == "down":
          (drive1,drive2,motorPwm) = drive_motor(-25)
          selection = True
        elif event.direction == "middle":
          runFlag = False
          (drive1,drive2,motorPwm) = drive_motor(0)
          selection = True
#         elif event.direction == "right":
#           setTilt += 10
#           selection = True
#         elif event.direction == "up":
#           (drive1,drive2,motorPwm) = drive_motor(25)
#           selection = True
#         elif event.direction == "down":
#           (drive1,drive2,motorPwm) = drive_motor(-25)
#           selection = True
#         elif event.direction == "middle":
#           (drive1,drive2,motorPwm) = drive_motor(0)
#           selection = True
          
  if (elapsedTime - prevUpdateTime)>1.0:
    print("0,{0},{x},{y},{z}".format(elapsedTime,**gyroRaw)) # rad/sec
    print("1,{0},{x},{y},{z}".format(elapsedTime,**accelRaw)) # Gs
    print("2,{0},{x},{y},{z}".format(elapsedTime,**magRaw))
    print("3,{0},{pitch},{roll},{yaw}".format(elapsedTime,**orientation))
    print("7,{0},{1}".format(elapsedTime,temp))
    xSpan = xMax-xMin
    ySpan = yMax-yMin
    xBiasEst = xMin+xSpan/2
    yBiasEst = yMin+ySpan/2
    xSFEst = horzMag/(xSpan/2)
    ySFEst = horzMag/(ySpan/2)
    print("X   : {0} {1}".format(xMin,xMax))
    print("Y   : {0} {1}".format(yMin,yMax))
    print("Bias: {0} {1}".format(xBiasEst,yBiasEst))
    print("SF  : {0} {1}".format(xSFEst,ySFEst))
    print("")
    prevUpdateTime = elapsedTime 


