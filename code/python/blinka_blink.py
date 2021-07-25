import time
import board
import digitalio
     
led = digitalio.DigitalInOut(board.G0)
led.direction = digitalio.Direction.OUTPUT
     
while True:
    led.value = True
    time.sleep(0.25)
    led.value = False
    time.sleep(0.25)
    led.value = True
    time.sleep(0.25)
    led.value = False
    time.sleep(1.25)