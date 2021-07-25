import time
import math
import board
import digitalio
import busio
import adafruit_bno055

magBias = (0.0, 0.0, 0.0)
magScaleErr = (1.0, 1.0, 1.0)
#magBias = (-5.03125, -13.0625, -51.3125)
#magBias = (-7.21875, -14.03125, -106.4375)
magBias =     (-4.6875, -13.3125, -51.25)
#magScaleErr = (1.0053287725448068, 0.9222875558319669, 4.0)
magScaleErr = (0.9964946005540792, 0.9364222310171312, 0.13604624865720585)

magScaleF = (1.0/magScaleErr[0], 1.0/magScaleErr[1], 1)
applyCal = True
    
led = digitalio.DigitalInOut(board.G0)
led.direction = digitalio.Direction.OUTPUT

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)
#sensor.mode = adafruit_bno055.COMPASS_MODE
#sensor.mode = adafruit_bno055.IMUPLUS_MODE
sensor.mode = adafruit_bno055.AMG_MODE
sensor.magnet_operation_mode = adafruit_bno055.MAGNET_ACCURACY_MODE

def temperature():
    global last_val  # pylint: disable=global-statement
    result = sensor.temperature
    if abs(result - last_val) == 128:
        result = sensor.temperature
        if abs(result - last_val) == 128:
            return 0b00111111 & result
    last_val = result
    return result

maxX = -100
minX = 100
maxY = -100
minY = 100
maxZ = -100
minZ = 100
doMinMax = True
nowCal = False
print("Sleeping")
time.sleep(10)
print("Starting")

hMagField = 17.687
vMagField = 51.918

while True:
    #led.value = True
    #time.sleep(0.25)
    #led.value = False
    #time.sleep(0.25)
    #led.value = True
    #time.sleep(0.25)
    #led.value = False

    """
    print(
        "Temperature: {} degrees C".format(temperature())
    )  # Uncomment if using a Raspberry Pi
    """
    #print("Euler angle: {}".format(sensor.euler))
    #print("----Accelerometer (m/s^2): {}".format(sensor.acceleration))
    #print("----Gyroscope (rad/sec): {}".format(sensor.gyro))
    (magX, magY, magZ) = (sensor.magnetic[0], sensor.magnetic[1], sensor.magnetic[2])
    print("----Raw Mag      (microteslas): {0:05.3f}  {1:05.3f} {2:05.3f}".format(magX, magY, magZ))
    if applyCal == True:
        magX = (magX - magBias[0])*magScaleF[0]
        magY = (magY - magBias[1])*magScaleF[1]
    print("----Magnetometer (microteslas): {0:05.3f}  {1:05.3f} {2:05.3f}".format(magX, magY, magZ))
    if doMinMax == True:
        if(magX > maxX): maxX = magX
        if(magX < minX): minX = magX
        if(magY > maxY): maxY = magY
        if(magY < minY): minY = magY
        if(magZ > maxZ): maxZ = magZ
        if(magZ < minZ): minZ = magZ
        print("    Mag MinMax    {}".format((minX, maxX, minY, maxY, minZ, maxZ)))
        print("    magBias =     {}".format( ( (maxX+minX)/2, (maxY+minY)/2, (maxZ+minZ)/2 ) ) )
        print("    magScaleErr = {}".format( ( (maxX-minX)/(2*hMagField), (maxY-minY)/(2*hMagField), (maxZ-minZ)/(2*hMagField) ) ) )
        if(nowCal == False and sensor.calibration_status[3] == 3):
            maxX = -100
            minX = 100
            maxY = -100
            minY = 100
            maxZ = -100
            minZ = 100
            nowCal = True
            print("------ MAG CAL ---------------------")
            print("------------------------------------")
    #
    

    #print("Quaternion: {}".format(sensor.quaternion))
    #print("Linear acceleration (m/s^2): {}".format(sensor.linear_acceleration))
    #print("Gravity (m/s^2): {}".format(sensor.gravity))
    #print("----Mode (enum): {}".format(sensor.mode))
    #print("----Cal Status {}".format(sensor.calibration_status))
    #print("     offsets_accelerometer {}".format(sensor.offsets_accelerometer))
    #print("     offsets_gyroscope     {}".format(sensor.offsets_gyroscope))
    #print("     offsets_magnetometer  {}".format(sensor.offsets_magnetometer))
    #print("     radius_accelerometer  {}".format(sensor.radius_accelerometer))
    #print("     radius_magnetometer   {}".format(sensor.radius_magnetometer))
    #print("----Temperature: {} degrees C".format(sensor.temperature))
    print("Heading {}".format(math.degrees(math.atan2(magY,magX))))
    print(" ")

    #time.sleep(1)

