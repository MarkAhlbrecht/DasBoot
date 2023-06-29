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
        #self.sensor.mode = adafruit_bno055.COMPASS_MODE
        #self.sensor.mode = adafruit_bno055.IMUPLUS_MODE
        self.sensor.mode = adafruit_bno055.AMG_MODE
        #self.sensor.magnet_operation_mode = adafruit_bno055.MAGNET_ACCURACY_MODE
        super().initialize()
        pass

    def getRawData(self):
        maxRetries = 5
        retry = 0;
        gyroReasonableLimit = 17.453 # 1000 deg/sec
        accelReasonableLimt = 40.0 # ~4g
        magReasonableLimit = 100.0 # ~2*mag field
        
        
        while (retry < maxRetries):
            rawRate = np.array([self.sensor.gyro[0], self.sensor.gyro[1], self.sensor.gyro[2]])
            rawAccel = np.array([self.sensor.acceleration[0], self.sensor.acceleration[1], self.sensor.acceleration[2]])
            rawMag = np.array([self.sensor.magnetic[0], self.sensor.magnetic[1], self.sensor.magnetic[2]])
            goodSample = False

            if ( rawRate[0] != None and rawRate[1] != None and rawRate[2] != None and \
                 rawAccel[0] != None and rawAccel[1] != None and rawAccel[2] != None and \
                 rawMag[0] != None and rawMag[1] != None and rawMag[2] != None):
                
                goodSample = True
                goodMag = True
                goodAccel = True
                goodRate = True
                if (abs(rawMag[0])>magReasonableLimit or abs(rawMag[1])>magReasonableLimit  or abs(rawMag[2])>magReasonableLimit):
                    print("-------Mag Anomaly-----")
                    print(rawMag)
                    rawMag = np.array([0.0,0.0,0.0])
                    goodMag = False
            
                if (abs(rawAccel[0])>accelReasonableLimt or abs(rawAccel[1])>accelReasonableLimt  or abs(rawAccel[2])>accelReasonableLimt):
                    print("-------Accel Anomaly-----")
                    print(rawAccel)
                    rawAccel = np.array([0.0,0.0,0.0]) 
                    goodAccel = False
                    
                if (abs(rawRate[0])>gyroReasonableLimit or abs(rawRate[1])>gyroReasonableLimit or abs(rawRate[2])>gyroReasonableLimit):
                    print("-------Gyro Anomaly-----")
                    print(rawRate)
                    rawRate = np.array([0.0,0.0,0.0]) 
                    goodRate = False
                
                if goodSample and goodMag and goodAccel and goodRate:
                    break
            rawRate = np.array([0,0,0])
            rawAccel = np.array([0,0,0])
            rawMag = np.array([0,0,0])
            print("Sample Fail")
            retry = retry + 1       
        
        #print(self.sensor.calibration_status)
        
        return (rawRate,rawAccel,rawMag)

if (__name__ == "__main__"):
    myIMU = BNO055_IMU()
    print(myIMU.timeOffset)

    s=0
    while (s< 500):
        (r,a,m) = myIMU.getRawData();
        #print(f"    Rate  {r}")
        #print(f"    Accel {a}")
        #print(f"    Mag   {m}")
        s=s+1
        
    while True:
        (t,r,a,m) = myIMU.sample()
        print(f"Sample {s}")
        print(f"    Time  {t}")
        print(f"    Rate  {r}")
        print(f"    Accel {a}")
        print(f"    Mag   {m}")
        s=s+1


