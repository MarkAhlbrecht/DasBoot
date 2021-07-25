import numpy as np
import time

"""

    Base IMU Class
      This class is the base class for Inertial Measurement Units (IMUs)
      It handles 3, 6 and 9 Degree of Freedom

      __init__ Constructor
      loadCalibration() - Loads Calibration Coef
      setCalibration(gyroBias, gyroSF, accelBias, accelSF, magBias, magSF)
      (time, rates, accel, mag) = sample() - get a sample
      updateTimeOffset(timeOffset) - offset from system time
      initialize() - initialize the sensor (called by constructor)
      inRunCalUpdate(gyroBias, gyroSF, accelBias, accelSF, magBias, magSF)
                   - Sets in run calibration coefs.  This is applied after
                     orientation.  This can be used for closed loop
                     feedback adjustments during run time
      setOrientation(Orientation C Matrix) - IMU to body orientation
      reset() - reset the IMU
      


"""


class IMU():


    def __init__(self, accels=True, gyros=True, mag=True):

        print ("IMU Constructor")
        self.timeOffset = 0
        self.timeInitialized = False
        self.initialized = False
        self.inRunCalApplied = False
        self.orientationApplied = False

        self.rate = np.array([0.0,0.0,0.0])
        self.accel = np.array([0.0,0.0,0.0])
        self.mag = np.array([0.0,0.0,0.0])
        self.orientation = np.eye( 3 ) # Equals its transpose


        self.loadCalibration()

        self.gyroInRunBias = np.array([0.0,0.0,0.0])
        self.gyroInRunScaleFactor = np.array([1.0,1.0,1.0])

        self.accelInRunBias = np.array([0.0,0.0,0.0])
        self.accelInRunScaleFactor = np.array([1.0,1.0,1.0])

        self.magInRunBias = np.array([0.0,0.0,0.0])
        self.magInRunScaleFactor = np.array([1.0,1.0,1.0])

    def loadCalibration(self):
        
        self.gyroCalBias = np.array([0.0,0.0,0.0])
        self.gyroCalScaleFactor = np.array([1.0,1.0,1.0])

        self.accelCalBias = np.array([0.0,0.0,0.0])
        self.accelCalScaleFactor = np.array([1.0,1.0,1.0])

        self.magCalBias = np.array([0.0,0.0,0.0]) 
        self.magCalScaleFactor = np.array([1.0,1.0,1.0])
        

    def setCalibraton(self, gyroCalBias, gyroCalScaleFactor, \
                       accelCalBias, accelCalScaleFactor, \
                       magCalBias, magCalScaleFactor):
        
        self.gyroCalBias = gyroCalBias
        self.gyroCalScaleFactor = gyroCalScaleFactor

        self.accelCalBias = accelCalBias
        self.accelCalScaleFactor = accelCalScaleFactor

        self.magCalBias = magCalBias
        self.magCalScaleFactor = magCalScaleFactor        
        
    def initialize(self):
        
        self.timeOffset = time.time()
        self.timeInitialized = True
        self.initialized = True

    
    def inRunCalUpdate(self, gyroInRunBias, gyroInRunScaleFactor, \
                       accelInRunBias, accelInRunScaleFactor, \
                       magInRunBias, magInRunScaleFactor):
        
        self.gyroInRunBias = gyroInRunBias
        self.gyroInRunScaleFactor = gyroInRunScaleFactor

        self.accelInRunBias = accelInRunBias
        self.accelInRunScaleFactor = accelInRunScaleFactor

        self.magInRunBias = magInRunBias
        self.magInRunScaleFactor = magInRunScaleFactor
        
        self.inRunCalApplied = True
    
    def getRawData(self):
        #self.rawRate = np.array([0.0,0.0,0.0])
        #self.rawAccel = np.array([0.0,0.0,0.0])
        #self.rawMag = np.array([0.0,0.0,0.0])

        #self.rawRate[0] = self.sampleTime
        #self.rawAccel[1] = self.sampleTime
        #self.rawMag[2] = self.sampleTime
        rawRate = np.array([0.0,0.0,0.0])
        rawAccel = np.array([0.0,0.0,0.0])
        rawMag = np.array([0.0,0.0,0.0])

        rawRate[0] = self.sampleTime
        rawAccel[1] = self.sampleTime
        rawMag[2] = self.sampleTime
        return (rawRate, rawAccel, rawMag)

    def sample(self):
        
        if (not self.initialized):
            self.initialize()
        
        self.sampleTime = time.time() - self.timeOffset
        (self.rawRate,self.rawAccel,self.rawMag) = self.getRawData()
        

        self.rateCal = self.gyroCalScaleFactor * (self.rawRate - self.gyroCalBias )
        self.accelCal = self.accelCalScaleFactor * (self.rawAccel - self.accelCalBias )
        self.magCal = self.magCalScaleFactor * (self.rawMag - self.magCalBias )

        if(self.orientationApplied):
            self.rateCal = np.matmul(self.rateCal,self.orientation)
            self.accelCal = np.matmul(self.accelCal,self.orientation)
            self.magCal = np.matmul(self.magCal,self.orientation)

        if (self.inRunCalApplied):
            print("Appling InCal")
            self.rate = self.gyroInRunScaleFactor * (self.rateCal - self.gyroInRunBias )
            self.accel = self.accelInRunScaleFactor * (self.accelCal - self.accelInRunBias )
            self.mag = self.magInRunScaleFactor * (self.magCal - self.magInRunBias )
            return (self.sampleTime, self.rate, self.accel, self.mag)
        else:
            return (self.sampleTime, self.rateCal, self.accelCal, self.magCal)



    def updateTimeOffset(self,timeOffset):

        self.timeOffset = timeOffset

    def setOrientation(self, orientationCMatrix):
        
        self.orientation = orientationCMatrix
        self.orientationApplied = True

    def reset(self):
        self.initialize()
        self.loadCalibration()


if (__name__ == "__main__"):

    print ("Start IMU Class")
    imu = IMU()
    imu.setOrientation(np.eye(3))
    C_orient = np.zeros( (3,3) )
    C_orient[0][2] = 1.0
    C_orient[1][1] = 1.0
    C_orient[2][0] = 1.0
    imu.setOrientation(C_orient)

    nSamples = 12
    s = 0
    while (s < nSamples):
        (t,r,a,m) = imu.sample()
        print(f"Sample {s}")
        print(f"    Time  {t}")
        print(f"    Rate  {r}")
        print(f"    Accel {a}")
        print(f"    Mag   {m}")

        if (s==4):
            print("Appling Update")
            print(f"{-1*imu.gyroCalBias}")
            print(f"{1/imu.gyroCalScaleFactor}")
            print(f"{-1*imu.accelCalBias}")
            print(f"{1/imu.accelCalScaleFactor}")
            print(f"{-1*imu.magCalBias}")
            print(f"{1/imu.magCalScaleFactor}")
            imu.inRunCalUpdate(-1*imu.gyroCalBias*imu.gyroCalScaleFactor , 1/imu.gyroCalScaleFactor, \
                               -1*imu.accelCalBias*imu.accelCalScaleFactor , 1/imu.accelCalScaleFactor, \
                               -1*imu.magCalBias*imu.magCalScaleFactor , 1/imu.magCalScaleFactor )

        if (s==6):
            imu.updateTimeOffset(time.time())

        if (s==8):
            imu.reset()
            time.sleep(2)
            

        s += 1



