import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BOARD)

GPIO.setup(11, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)
GPIO.setup(7, GPIO.OUT)

pwm=GPIO.PWM(7, 100)
pwm.start(0)
pwm.ChangeDutyCycle(100)
GPIO.output(7, True)

# CW at full speed
print ("CW at full speed")
GPIO.output(11, True)
GPIO.output(12, False)
sleep(2)

# CCW at full speed
print("CCW at full speed")
GPIO.output(12, True)
GPIO.output(11, False)
sleep(2)

# change to 50% speed
print("set speed to 50%")
pwm.ChangeDutyCycle(50)
GPIO.output(7, True)

# CW at 50% speed
print("CW at 50% speed")
GPIO.output(11, True)
GPIO.output(12, False)
sleep(2)

# CCW at 50% speed
print("CCW at 50% speed")
GPIO.output(12, True)
GPIO.output(11, False)
sleep(2)

# change to 25% speed
print("set speed to 25%")
pwm.ChangeDutyCycle(25)
GPIO.output(7, True)

# CW at 25% speed
print("CW at 25% speed")
GPIO.output(11, True)
GPIO.output(12, False)
sleep(2)

# CCW at 25% speed
print("CCW at 25% speed")
GPIO.output(12, True)
GPIO.output(11, False)
sleep(2)

# change to 15% speed
print("set speed to 15%")
pwm.ChangeDutyCycle(15)
GPIO.output(7, True)

# CW at 15% speed
print("CW at 15% speed")
GPIO.output(11, True)
GPIO.output(12, False)
sleep(2)

# CCW at 15% speed
print("CCW at 15% speed")
GPIO.output(12, True)
GPIO.output(11, False)
sleep(2)

GPIO.output(7, False)

pwm.stop()

GPIO.cleanup()