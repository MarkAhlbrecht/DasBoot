import numpy as np
import time
import math
from math import cos, sin, radians, degrees, sqrt, atan2, asin
from IMU import IMU

class AHRS:   
    
    def __init__(self,AhrsIMU,timeInit=0,HW_ID=1):
        
#         print("AHRS Constructor")
        self.IMU = AhrsIMU
        self.rates = np.array([0.0,0.0,0.0])
        self.accel = np.array([0.0,0.0,0.0])
        self.mag = np.array([0.0,0.0,0.0])
        self.att = np.array([0.0,0.0,0.0])
        self.gyroBias = np.array([0.0,0.0,0.0])
        self.prevTime = 0
        self.tau = 1/10
        self.attTau = 1/10
        self.g = 9.81
        self.accumTheta = np.array([0, 0 , 0])
        
        self.HW_ID = HW_ID
        
        if timeInit>0:
            self.startTime = timeInit
        else:
            self.startTime = 0
            
        #print("initialized {0}".format(self.HW_ID))
       
    def warmup(self,warmUpSamples):
        print("Warming up ...")
        for i in range(1,warmUpSamples):
            (t,r,a,m) = self.IMU.sample()
            self.prevTime = t
        print("Warm up complete")
    
    def align(self, alignSamples=100):
        print("Aligning {0} Samples".format(alignSamples))
        for i in range(1,alignSamples+1):
            (self.sampleTime,self.gyroRaw,self.accelRaw,self.magRaw) = self.IMU.sample()
#             print(f"--- Align Sample {i}")
#             print("RawAccel: ", self.accelRaw)
#             print("RawGyro: ", self.gyroRaw)
#             print("RawMag: ", self.magRaw)
            self.prevTime = self.sampleTime
            
            self.accel = self.accel + \
               1/alignSamples*self.accelRaw
            self.rates = self.rates + \
               1/alignSamples*self.gyroRaw
            self.mag = self.mag + \
               1/alignSamples*self.magRaw
            
#             print("Accel: ", self.accel)
#             print("Gyro: ", self.rates)
#             print("Mag: ", self.mag)
        
        # Establish initial prich roll from averaged accel measures
        self.att[1] = asin(self.accel[0]/self.g)  # Pitch
        self.att[0] = -atan2(self.accelRaw[1],-self.accelRaw[2]) # Roll               
        Ca = self.euler2C(self.att[1],self.att[0],0.0)
        
#         print(np.degrees(self.att))
#         print(Ca)

        # Establish initial heading from horizontal mag measuremetns
        magL = Ca.dot(self.mag.transpose())
        self.att[2] = atan2(-1*magL[1],magL[0])

        # Initialize Gyro, Accel, and Filtered to Level transitions
        self.C_AL = self.euler2C(self.att[1],self.att[0],self.att[2])
        self.C_GL = self.euler2C(self.att[1],self.att[0],self.att[2])
        self.C_FL = self.euler2C(self.att[1],self.att[0],self.att[2])

        # Initial Gyro bias is the average over sample period
        self.gyroBias = self.rates
        self.gyroBias = np.array([0.0, 0.0, 0.0])
        print("Align Attitude:")
        print(self.att*degrees(1.0))
        print("Align Gyro Bias")
        print(self.gyroBias*degrees(1.0))
        self.prevTime = self.sampleTime - self.startTime

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

    # Update the Attitude Solution
    def update(self):
        r180 = radians(180)
        nr180 = radians(-180)
        r360 = radians(360)

        deltaTheta = np.array([0, 0 , 0])
        
        omega = np.array( [0, 0, 0] )
        deltaC = np.ones( (3,3) )

        (self.sampleTime,self.gyroRaw,self.accelRaw,self.magRaw) = self.IMU.sample()
        
        self.elapsedTime = self.sampleTime - self.startTime
#         print("AHRS gyroRaw,{0},{1}".format(self.elapsedTime,self.gyroRaw)) # rad/sec
#         print("AHRS accelRaw,{0},{1}".format(self.elapsedTime,self.accelRaw)) # rad/sec
#         print("AHRS magRaw,{0},{1}".format(self.elapsedTime,self.magRaw)) # rad/sec
   
        deltaTime = self.elapsedTime - self.prevTime
  
        # Accumplate Rates
        omega = self.gyroRaw - self.gyroBias
        deltaTheta = omega*deltaTime # Basic Integration
        self.accumTheta = self.accumTheta + deltaTheta
#         print(f"DeltaT {deltaTime}")
#         print("Dt {0} {1} {2}".format(degrees(self.accumTheta[0]), \
#                                       degrees(self.accumTheta[1]), \
#                                       degrees(self.accumTheta[2])))

        # Form the deltaC matrix from the angular rotations
        # Single taylor series with small angle assumption
        deltaC[0,1] = -1*deltaTheta[2]
        deltaC[0,2] = deltaTheta[1]
        deltaC[1,0] = deltaTheta[2]
        deltaC[1,2] = -1*deltaTheta[0]
        deltaC[2,0] = -1*deltaTheta[1]
        deltaC[2,1] = deltaTheta[0]

       # Update the gyro and filter solution
        self.C_GL = self.C_GL.dot(deltaC)
#         (pf,rf,yf) = self.c2euler(self.C_GL)
#         print("=======================>C_GYRO: {0}".format(np.degrees(np.array([pf,rf,yf]))))
        
        
        self.C_FL = self.C_FL.dot(deltaC)
#        self.C_FL = self.ortho_norm(self.C_FL)

#       Determine the Accel Solution
        accelPitch = asin(self.accelRaw[0]/self.g)
#         accelRoll = atan2(self.accelRaw[1],self.g)
        accelRoll = -atan2(self.accelRaw[1],-self.accelRaw[2])
        #print(degrees(accelRoll))
        accelMag = np.array([self.magRaw[0], self.magRaw[1],self.magRaw[2]])
        self.C_AL = self.euler2C(accelPitch,accelRoll,0.0)
        magL = self.C_AL.dot(accelMag.transpose())
        yaw = atan2(-1*magL[1],magL[0])
        self.C_AL = self.euler2C(accelPitch,accelRoll,yaw)
#         print(self.C_AL)
#         print("=======================>C_ACCE: {0}".format(np.degrees(np.array([accelPitch,accelRoll,yaw]))))
#         print("=======================>MAG LV: {0}".format(magL))
       
       # Update the filtered solution with the data from the
       # Accel based solution
        #print(self.C_FL)
        (pf,rf,yf) = self.c2euler(self.C_FL)
        pf = (1-self.attTau)*pf + self.attTau*accelPitch
        rf = (1-self.attTau)*rf + self.attTau*accelRoll
        ey = yf-yaw
        if ey > r180:
            ey = ey - r360
        elif ey < -r180:
            ey = ey + r360
        yf = (1-self.tau)*yf + self.tau*(yf - ey)
        if yf > r180:
            yf = yf-r360
        elif yf < -r180:
            yf = yf+r360
        self.C_FL = self.euler2C(pf,rf,yf)
       
        self.att = np.array([pf,rf,yf])
#         print("=======================>C_FLTR: {0}".format(np.degrees(self.att)))
        self.prevTime = self.elapsedTime
        
        return (self.att)
       
if (__name__ == "__main__"):

    print ("Test AHRS Class")
    myIMU = IMU()
    C_orient = np.zeros( (3,3) )
    C_orient[0][2] = -1.0
    C_orient[1][1] = -1.0
    C_orient[2][0] = -1.0
    myIMU.setOrientation(C_orient)
    
    myAHRS = AHRS(myIMU)
#    imu.setOrientation(np.eye(3))
#     C_orient = np.zeros( (3,3) )
#     C_orient[0][2] = 1.0
#     C_orient[1][1] = 1.0
#     C_orient[2][0] = 1.0
#     imu.setOrientation(C_orient)

    nSamples = 10
    s = 0
    myAHRS.warmup(100)
    myAHRS.align()
    while (s < nSamples):
        att = myAHRS.update()
        print("MyAhrs Att --> {0} {1} {2}".format(degrees(att[0]),degrees(att[1]),degrees(att[2])))
            

        s += 1

