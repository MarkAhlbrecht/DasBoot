# Heading/Turn Controller
#
import PID
import numpy as np

wheelRate = 4.1 # degrees/s - 11s to cover 45 degrees
class TurnRateController:
    
    def __init__(self, rudderAngleInitIn = 0, KtrIn = 2):
        
        self.rudderAngleInit = rudderAngleInitIn  # Initial rudder angle guess
        self.Ktr = KtrIn # Initial TR to RA constant
        
    def update(self,time,setTurnRate,turnRate):
        self.estRudderFunc(turnRate)
        self.setRudderAngle = self.determine_rudder_angle(setTurnRate)
        
    def rudder_angle_est(turnRate, driveTime):
        pass
        
        
if (__name__ == "__main__"):

    nSamples = 100
    dt = 0.25
    k = 1.5
    
    tr = np.ones(nSamples);
    tr[0:10] = 5
    tr[10:20] = -5
    tr[20:30] = 10
    tr[30:40] = 15
    tr[40:50] = 10
    tr[50:60] = 5
    tr[60:70] = 0
    tr[70:80] = -5
    tr[80:90] = -2.5
    tr[90:100] = 0.0
    
    ra = tr/k
    deltaRA = np.diff(ra)
    deltaRA[0] = 0
    deltaRA = np.append(deltaRA,np.zeros(1))
    driveTime = deltaRA / wheelRate
    
    estRA = np.zeros(100)
    estRAinit = False
    estKtr = 2;
    estTRSamples = 0;
    turnRateF = 0;
    
    for n in range(nSamples):
        t = n*0.25
        
        estTRSamples +=1;
        turnRateF = turnRateF*(estTRSamples-1)/estTRSamples + tr[n] * (1/estTRSamples)
        
        if (estRAinit == True):
            estKtr = (turnRateF - prevTR)/(estRA[n] - prevRA)
            print(f" {turnRateF} {prevTR}  {estRA[n]} {prevRA} {estKtr}")
        estRA[n] = turnRateF/estKtr
            
        if(driveTime[n] > 0 or driveTime[n] < 0):
            prevRA = estRA[n]
            prevTR = turnRateF
            estRA[n] = estRA[n] + wheelRate*driveTime[n]
            estRAinit = True
            estTRSamples = 0
            

            
                
            
            
        
        print("{0}, {1:4.2f}, {2:4.2f}, {3:4.2f}, {4:4.2f} {5:4.2f} {6:4.2f}".format(n,t,tr[n],turnRateF,ra[n],deltaRA[n],driveTime[n]))
    