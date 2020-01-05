import serial
from time import sleep
import pynmea2


with serial.Serial('/dev/serial0', baudrate=4800, timeout=1) as ser:
    while True:
        strip = ser.readline()
        print(strip) #this prints to raw NMEA messages
        if strip.find(b'GGA') > 0:
            msg = pynmea2.parse(strip.decode('utf-8'))
            print ("    Timestamp", msg.timestamp)
            print ("    Latitude", msg.lat)
            print ("    Latitude Direction", msg.lat_dir)
            print ("    Longitude", msg.lon)
            print ("    Longitude Direction", msg.lon_dir)
            print ("    GPS Quality Indicator", msg.gps_qual)
            print ("    Number of Satellites in use", msg.num_sats)
            print ("    Horizontal Dilution of Precision", msg.horizontal_dil)
            print ("    Antenna Alt above sea level (mean)", msg.altitude)
            print ("    Units of altitude (meters)", msg.altitude_units)
            print ("    Geoidal Separation", msg.geo_sep)
            print ("    Units of Geoidal Separation (meters)", msg.geo_sep_units)
            print ("    Age of Differential GPS Data (secs)", msg.age_gps_data)
            print ("    Differential Reference Station ID", msg.ref_station_id)
        if strip.find(b'GLL') > 0:
            msg = pynmea2.parse(strip.decode('utf-8'))
            print ("    Latitude", msg.lat)
            print ("    Latitude Direction", msg.lat_dir)
            print ("    Longitude", msg.lon)
            print ("    Longitude Direction", msg.lon_dir)
            print ("    Timestamp", msg.timestamp)
            print ("    Status", msg.status) # contains the "A" or "V" flag
            print ("    FAA mode indicator", msg.faa_mode)
        if strip.find(b'GSA') > 0:
            msg = pynmea2.parse(strip.decode('utf-8'))
            print ("    Mode", msg.mode)
            print ("    Mode fix type", msg.mode_fix_type)
            print ("    SV ID01", msg.sv_id01)
            print ("    SV ID02", msg.sv_id02)
            print ("    SV ID03", msg.sv_id03)
            print ("    SV ID04", msg.sv_id04)
            print ("    SV ID05", msg.sv_id05)
            print ("    SV ID06", msg.sv_id06)
            print ("    SV ID07", msg.sv_id07)
            print ("    SV ID08", msg.sv_id08)
            print ("    SV ID09", msg.sv_id09)
            print ("    SV ID10", msg.sv_id10)
            print ("    SV ID11", msg.sv_id11)
            print ("    SV ID12", msg.sv_id12)
            print ("    PDOP (Dilution of precision)", msg.pdop)
            print ("    HDOP (Horizontal DOP)", msg.hdop)
            print ("    VDOP (Vertical DOP)", msg.vdop)
        if strip.find(b'GSV') > 0:
            msg = pynmea2.parse(strip.decode('utf-8'))
            print ("    Number of messages of type in cycle", msg.num_messages)
            print ("    Message Number", msg.msg_num)
            print ("    Total number of SVs in view", msg.num_sv_in_view)
            print ("    SV PRN number 1", msg.sv_prn_num_1)
            print ("    Elevation in degrees 1", msg.elevation_deg_1) # 90 max
            print ("    Azimuth, deg from true north 1", msg.azimuth_1) # 000 to 159
            print ("    SNR 1", msg.snr_1) # 00-99 dB
            print ("    SV PRN number 2", msg.sv_prn_num_2)
            print ("    Elevation in degrees 2", msg.elevation_deg_2) # 90 max
            print ("    Azimuth, deg from true north 2", msg.azimuth_2) # 000 to 159
            print ("    SNR 2", msg.snr_2) # 00-99 dB
            print ("    SV PRN number 3", msg.sv_prn_num_3)
            print ("    Elevation in degrees 3", msg.elevation_deg_3) # 90 max
            print ("    Azimuth, deg from true north 3", msg.azimuth_3) # 000 to 159
            print ("    SNR 3", msg.snr_3) # 00-99 dB
            print ("    SV PRN number 4", msg.sv_prn_num_4)
            print ("    Elevation in degrees 4", msg.elevation_deg_4) # 90 max
            print ("    Azimuth, deg from true north 4", msg.azimuth_4) # 000 to 159
            print ("    SNR 4", msg.snr_4)
        if strip.find(b'RMB') > 0:
            msg = pynmea2.parse(strip.decode('utf-8'))
            print ("    Status", msg.status) # contains the "A" or "V" flag
            print ("    Cross Track Error", msg.cross_track_error) # nautical miles, 9.9 max
            print ("    Cross Track Error, direction to corrent", msg.cte_correction_dir)
            print ("    Origin Waypoint ID", msg.origin_waypoint_id)
            print ("    Destination Waypoint ID", msg.dest_waypoint_id)
            print ("    Destination Waypoint Latitude", msg.dest_lat)
            print ("    Destination Waypoint Lat Direction", msg.dest_lat_dir)
            print ("    Destination Waypoint Longitude", msg.dest_lon)
            print ("    Destination Waypoint Lon Direction", msg.dest_lon_dir)
            print ("    Range to Destination", msg.dest_range) # Nautical Miles
            print ("    True Bearing to Destination", msg.dest_true_bearing)
            print ("    Velocity Towards Destination", msg.dest_velocity) # Knots
            print ("    Arrival Alarm", msg.arrival_alarm)
        if strip.find(b'RMC') > 0:
            msg = pynmea2.parse(strip.decode('utf-8'))
            print ("    Timestamp", msg.timestamp)
            print ("    Status", msg.status) # contains the "A" or "V" flag
            print ("    Latitude", msg.lat)
            print ("    Latitude Direction", msg.lat_dir)
            print ("    Longitude", msg.lon)
            print ("    Longitude Direction", msg.lon_dir)
            print ("    Speed Over Ground", msg.spd_over_grnd)
            print ("    True Course", msg.true_course)
            print ("    Datestamp", msg.datestamp)
            print ("    Magnetic Variation", msg.mag_variation)
            print ("    Magnetic Variation Direction", msg.mag_var_dir)
        if strip.find(b'WPL') > 0:
            msg = pynmea2.parse(strip.decode('utf-8'))
            print ("    Latitude", msg.lat)
            print ("    Latitude Direction", msg.lat_dir)
            print ("    Longitude", msg.lon)
            print ("    Longitude Direction", msg.lon_dir)
            print ("    Waypoint ID", msg.waypoint_id)
        if strip.find(b'RTE') > 0:
            msg = pynmea2.parse(strip.decode('utf-8'))
            print ("    Number of sentences in sequence", msg.num_in_seq)
            print ("    Sentence Number", msg.sen_num)
            print ("    Start Type", msg.start_type) # The first in the list is either current route or waypoint
            print ("    Name or Number of Active Route", msg.active_route_id)
        if strip.find(b'BOD') > 0:
            msg = pynmea2.parse(strip.decode('utf-8'))
            print ("    Bearing True", msg.bearing_t)
            print ("    Bearing True Type", msg.bearing_t_type)
            print ("    Bearing Magnetic", msg.bearing_mag)
            print ("    Bearing Magnetic Type", msg.bearing_mag_type)
            print ("    Destination", msg.dest)
            print ("    Start", msg.start)
            print ()
            print ("loop")
            print ()
                   
                   
            sleep(0.001)