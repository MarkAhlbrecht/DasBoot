from sense_hat import SenseHat
import numpy as np
import time
import math
from math import cos, sin, radians, degrees, sqrt, atan2, asin

class EasyAHRS:
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

    self.rates = np.array([0,0,0])
    self.accel = np.array([0,0,0])
    self.att = np.array([0,0,0])
    def __init__(self,senseHat=1):
        
        self.sense = SenseHat()
        
        