import RPi.GPIO as GPIO
from time import sleep

#PWM_PIN = 33
#FWD_PIN = 35
#REV_PIN = 37
PWM_PIN = 13
FWD_PIN = 19
REV_PIN = 26

GPIO.setmode(GPIO.BCM)

GPIO.setup(FWD_PIN, GPIO.OUT)
GPIO.setup(REV_PIN, GPIO.OUT)
GPIO.setup(PWM_PIN, GPIO.OUT)

pwm=GPIO.PWM(PWM_PIN, 100)
pwm.start(0)
pwm.ChangeDutyCycle(100)
GPIO.output(PWM_PIN, True)

# CW at full speed
print ("CW at full speed")
GPIO.output(FWD_PIN, True)
GPIO.output(REV_PIN, False)
sleep(2)

# CCW at full speed
print("CCW at full speed")
GPIO.output(REV_PIN, True)
GPIO.output(FWD_PIN, False)
sleep(2)

# change to 50% speed
print("set speed to 50%")
pwm.ChangeDutyCycle(50)
GPIO.output(PWM_PIN, True)

# CW at 50% speed
print("CW at 50% speed")
GPIO.output(FWD_PIN, True)
GPIO.output(REV_PIN, False)
sleep(2)

# CCW at 50% speed
print("CCW at 50% speed")
GPIO.output(REV_PIN, True)
GPIO.output(FWD_PIN, False)
sleep(2)

# change to 25% speed
print("set speed to 25%")
pwm.ChangeDutyCycle(25)
GPIO.output(PWM_PIN, True)

# CW at 25% speed
print("CW at 25% speed")
GPIO.output(FWD_PIN, True)
GPIO.output(REV_PIN, False)
sleep(2)

# CCW at 25% speed
print("CCW at 25% speed")
GPIO.output(REV_PIN, True)
GPIO.output(FWD_PIN, False)
sleep(2)

# change to 15% speed
print("set speed to 15%")
pwm.ChangeDutyCycle(15)
GPIO.output(PWM_PIN, True)

# CW at 15% speed
print("CW at 15% speed")
GPIO.output(FWD_PIN, True)
GPIO.output(REV_PIN, False)
sleep(2)

# CCW at 15% speed
print("CCW at 15% speed")
GPIO.output(REV_PIN, True)
GPIO.output(FWD_PIN, False)
sleep(2)

GPIO.output(PWM_PIN, False)

pwm.stop()

GPIO.cleanup()