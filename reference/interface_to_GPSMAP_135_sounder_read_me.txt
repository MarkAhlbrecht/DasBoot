This is what I did to interface to the Garmain GPSMAP 135 Sounder unit.
The Goal was to read NMEA 0183 sentences with a python script.

What I learned.
1) NMEA 0183 is set at 4800 baud. 8N1 (8 bit, no parity, one stop bit)
2) Pin #1 is +V and Pin #2 is Gnd.  Unit powered up with no issues on 12v.
3) Pin 3 on the Garmin GPSMAP 135 is the single serial output.
4) Garmin GPSMap 135 has a simulation mode.  This was nice as I was able
   to verify that NMEA messages where coming out and nicely formed. I was
   also able to verify the 4800 baud rate with the scope.
5) Connecting to Pi with Minicon showed data being transmitted, but it 
   was all garbled.  I tried all combinations of baud, parity, # bits, and
   # of stop bits.  Still garbage!!!!!
6) ***Note*** For some unknown reason, I needed to invert the signal
  to get is to be read by the Raspberry Pi. I used a 7404 Hex 
  inverter to flip the signal.  Then it worked.  Otherwise it was just
  garbage on the screen.
7) I created a python script (simple_serial_with_pynmea_decode_all.py) to
   read all of the GSPMap 135 sentences and decode each message. This
   script uses pynmea2 library to decode the NMEA sentences.
7a) Good resoures for pynmea2 are:
    1) https://github.com/Knio/pynmea2/blob/master/pynmea2/types/talker.py
    2) http://aprs.gids.nl/nmea/
8) Example syntax to: Open serial, read line, parce line, use data.

	with serial.Serial('/dev/serial0', baudrate=4800, timeout=1) as ser:
    	while True:
        	strip = ser.readline()
        	print(strip) #this prints to raw NMEA messages
        	if strip.find(b'GGA') > 0:
            	msg = pynmea2.parse(strip.decode('utf-8'))
            	print ("    Timestamp", msg.timestamp)

Summary,  we need to invert the GPSMap 135 signal. Baud is only 4800