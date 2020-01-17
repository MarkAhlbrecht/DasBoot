"""
DC Motor PWM Class
   m = PWM_Motor(fwdGPIO, revGPIO, pwdGPIO, deadband=0)
"""


class PWM_Motor:
    
    def __init__(self, fwdGPIO=11, revGPIO==12, pwdGPIO=7, deadband=0, scaleFactor=1):
        self.fwdGPIOPin = fwdGPIO
        self.revGPIOPin = revGPIO
        self.pwdGPIOPin = pwdGPIO
        self.deadband = deadband
        self.scaleFactor = scaleFactor

        # Set mode of GPIO to BOARD
        # Note - This should be checked for conflict at some point
        GPIO.setmode(GPIO.BOARD)

        GPIO.setup(fwdGPIO, GPIO.OUT)
        GPIO.setup(revGPIO, GPIO.OUT)
        GPIO.setup(pwdGPIO, GPIO.OUT)
        self.pwm=GPIO.PWM(7, 100)
        self.pwm.start(0)
        self.forward = 0
        self.reverse = 0
        self.motorPwm = 0
        pwm.ChangeDutyCycle(100)
        GPIO.output(7, True)
        GPIO.output(fwdGPIO, self.forward)
        GPIO.output(revGPIO, self.reverse)
        pwm.ChangeDutyCycle(motorPwm)

    def drive_motor(omegaMotor):
 
        if abs(omegaMotor) > deadBand:
            drive1 = omegaMotor > 0
            drive2 = not drive1
            motorPwm = abs(Kpwm*omegaMotor)
            if motorPwm > 100:
                motorPwm = 100
            #
            # Command Motor Pins
            #
            else:
                drive1 = False
                drive2 = False
                motorPwm = 0
            #
            # Command Motor Pins
            #

            return (drive1,drive2,motorPwm)



#
# Setup Motor Control Pins
#
GPIO.setmode(GPIO.BOARD)

GPIO.setup(11, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)
GPIO.setup(7, GPIO.OUT)
pwm=GPIO.PWM(7, 100)
pwm.start(0)
drive1 = 0
drive2 = 0
motorPwm = 0
pwm.ChangeDutyCycle(100)
GPIO.output(7, True)


    GPIO.output(11, drive1)
    GPIO.output(12, drive2)
    pwm.ChangeDutyCycle(motorPwm)