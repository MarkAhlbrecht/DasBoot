import serial
from time import sleep
import pynmea2


with serial.Serial('/dev/serial0', baudrate=9600, timeout=1) as ser:
    # read 10 lines from the serial output
    while True:
        strip = ser.readline()
        #print(strip) #this prints to raw NMEA messages
        if strip.find(b'GGA') > 0:
            msg = pynmea2.parse(strip.decode('utf-8'))
            print(msg.timestamp,msg.latitude,msg.lat_dir,msg.longitude,msg.lon_dir,msg.num_sats)
            print ("time   ", msg.timestamp)
            print ("lat    ", msg.latitude,msg.lat_dir)
            print ("lon    ", msg.longitude,msg.lon_dir)
            print ("num sat", msg.num_sats)
        sleep(0.1)