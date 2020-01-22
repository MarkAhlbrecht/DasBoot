from sense_hat import SenseHat
import numpy as np
import time
import math
from math import cos, sin, radians, degrees, sqrt, atan2, asin

class EasyAHRS:   
    
    def __init__(self,senseHatNum=1):
        
        #####
        # Predetermined Mag Calibraton
        ####
        # Sense Hat 1
        # xBias = -30.02851629257202
        # yBias = 7.201869010925293 +1.4395752679386042
        # xSF = 0.6773587707778925
        # ySF = 0.7209744740877911

        # Sense Hat 2
        self.xBias = 2.701296329498291
        self.yBias = 19.79104995727539
        self.xSF = 0.6955512793747712
        self.ySF = 0.7164366008777275
        #
        # AHRS Smoothing filter time constant
        self.tau = 1/200
        
        self.rates = np.array([0.0,0.0,0.0])
        self.accel = np.array([0.0,0.0,0.0])
        self.mag = np.array([0.0,0.0,0.0])
        self.att = np.array([0.0,00.0,0.0])
        
        self.sense = SenseHat()
        self.senseHatNum = senseHatNum
        print("initialized {0}".format(self.senseHatNum))
       
    def warmup(self):
        print("Warming up ...")
        for i in range(1,101):
            gyroRaw = self.sense.get_gyroscope_raw()
            accelRaw = self.sense.get_accelerometer_raw()
            magRaw = self.sense.get_compass_raw()
            temp = self.sense.get_temperature()
        print("Warm up complete")
    
    def align(self, alignSamples=100):
        print("Aligning {0} Samples".format(alignSamples))
        for i in range(1,alignSamples+1):
#             elapsedTime = time.time() - startTime  
            gyroRaw = self.sense.get_gyroscope_raw()
            accelRaw = self.sense.get_accelerometer_raw()
            magRaw = self.sense.get_compass_raw()
            temp = self.sense.get_temperature()
            magRaw["x"] = self.xSF *(magRaw["x"] - self.xBias)
            magRaw["y"] = self.ySF *(magRaw["y"] - self.yBias)
                
            self.accel = self.accel + \
               1/alignSamples*np.array([accelRaw["x"], accelRaw["y"],accelRaw["z"]])
            self.rates = self.rates + \
               1/alignSamples*np.array([gyroRaw["x"], gyroRaw["y"],gyroRaw["z"]])
            self.mag = self.mag + \
               1/alignSamples*np.array([magRaw["x"], magRaw["y"],magRaw["z"]])
            print("Accel: ", self.accel)
            print("Gyro: ", self.rates)
            print("Mag: ", self.mag)
        
        # Establish initial prich roll from averaged accel measures
        self.att[1] = asin(-1*self.accel[0])
        print("Pitch {0}".format(self.att[1]))
        print("Pitch {0}".format(asin(-1*self.accel[0])))
        self.att[0] = atan2(self.accel[1],1.0)
        print(self.att)
        Ca = self.euler2C(self.att[1],self.att[0],0.0)
        print(Ca)
        # Establish initial heading from horizontal mag measuremetns
        magL = Ca.dot(self.mag.transpose())
        self.att[2] = atan2(-1*magL[1],magL[0])

        # Initialize Gyro, Accel, and Filtered to Level transitions
        C_AL = self.euler2C(self.att[1],self.att[0],self.att[2])
        C_GL = self.euler2C(self.att[1],self.att[0],self.att[2])
        C_FL = self.euler2C(self.att[1],self.att[0],self.att[2])

        # Initial Gyro bias is the average over sample period
        self.gyroBias = self.rates
        print("Align Attitude:")
        print(self.att*degrees(1.0))
        print("Align Gyro Bias")
        print(self.gyroBias*degrees(1.0))

    """
    "   c2euler
    "   This routine converts a C matrix represention to euler representation
    "   Input: C matrix
    "   Output: pitch roll yaw
    """
    def euler2C(self,pitch,roll,yaw):
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
    def c2euler(self,C):
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
    def ortho_norm(self,C):
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

    
        