import numpy as np
import time
import math
import board
import digitalio
import busio
import adafruit_bno055
from IMU import IMU 

class BNO055_IMU(IMU):
    
    def __init__(self):
        print("BNO055 Constructor")
        super().__init__(self)
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)
        #sensor.mode = adafruit_bno055.COMPASS_MODE
        #sensor.mode = adafruit_bno055.IMUPLUS_MODE
        self.sensor.mode = adafruit_bno055.AMG_MODE
        self.sensor.magnet_operation_mode = adafruit_bno055.MAGNET_ACCURACY_MODE
        super().initialize()
        pass

    def getRawData(self):
        rawRate = np.array([self.sensor.gyro[0], self.sensor.gyro[1], self.sensor.gyro[2]])
        rawAccel = np.array([self.sensor.acceleration[0], self.sensor.acceleration[1], self.sensor.acceleration[2]])
        rawMag = np.array([self.sensor.magnetic[0], self.sensor.magnetic[1], self.sensor.magnetic[2]])

        return (rawRate,rawAccel,rawMag)

if (__name__ == "__main__"):
    myIMU = BNO055_IMU()
    print(myIMU.timeOffset)

    s=0
    while True:
        (t,r,a,m) = myIMU.sample()
        print(f"Sample {s}")
        print(f"    Time  {t}")
        print(f"    Rate  {r}")
        print(f"    Accel {a}")
        print(f"    Mag   {m}")
        s=s+1


