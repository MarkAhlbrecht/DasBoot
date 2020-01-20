# from PWM_Motor import *
import PWM_Motor
from time import sleep


motor = PWM_Motor.PWM_Motor(fwdGPIO=11, revGPIO=12, pwdGPIO=7, deadband=0, scaleFactor=1)

# # CW at full speed
print ("CW at full speed")
motor.drive(100)
sleep(2)

# # CCW at full speed
print("CCW at full speed")
motor.drive(-100)
sleep(2)

# # CW at 50% speed
print("CW at 50% speed")
motor.drive(50)
sleep(2)
# 
# # CCW at 50% speed
print("CCW at 50% speed")
motor.drive(-50)
sleep(2)

# # CW at 25% speed
print("CW at 25% speed")
motor.drive(25)
sleep(2)

# # CCW at 25% speed
print("CCW at 25% speed")
motor.drive(-25)
sleep(2)

# # CW at 15% speed
print("CW at 15% speed")
motor.drive(15)
sleep(2)
# 
# # CCW at 15% speed
print("CCW at 15% speed")
motor.drive(-15)
sleep(2)

print("Closing Motor")
motor.close()
