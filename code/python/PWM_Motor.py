import RPi.GPIO as GPIO

"""
DC Motor PWM Class
   m = PWM_Motor(fwdGPIO, revGPIO, pwdGPIO, deadband=0)
"""


class PWM_Motor:
    
    def __init__(self, fwdGPIO=11, revGPIO=12, pwdGPIO=7, deadband=0, scaleFactor=1):
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
        self.pwm.ChangeDutyCycle(100)
        GPIO.output(7, True)
        GPIO.output(fwdGPIO, self.forward)
        GPIO.output(revGPIO, self.reverse)
        self.pwm.ChangeDutyCycle(self.motorPwm)

    def drive(self,throttle):
 
        if abs(throttle) > self.deadband:
            self.forward = throttle > 0
            self.reverse = not self.forward
            self.motorPwm = abs(self.scaleFactor*throttle)
            if self.motorPwm > 100:
                self.motorPwm = 100
        else:
            self.forward = False
            self.reverse = False
            self.motorPwm = 0
            #
            # Command Motor Pins
            #
        
        GPIO.output(11, self.forward)
        GPIO.output(12, self.reverse)
        self.pwm.ChangeDutyCycle(self.motorPwm)
        return (self.forward,self.reverse,self.motorPwm)
    
    def close(self):
        self.drive(0)
        GPIO.output(7, False)
        self.pwm.stop()
        GPIO.cleanup()

