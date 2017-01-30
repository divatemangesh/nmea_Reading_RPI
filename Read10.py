import time
import serial
import string
from pynmea import nmea
import paho.mqtt.client as paho
import RPi.GPIO as GPIO
#import the GPIO and time package

GPIO.setmode(GPIO.BOARD)
GPIO.setup(7, GPIO.OUT)
# loop through 50 times, on/off for 1 second

#rad =8.910493827139932e-09             
rad =0            
rad_feed=0 
c_rad=0
f_lat_feed=0       
f_lon_feed=0   
status=0
c_lat=3
c_lon=2
f_lat=18.672652777777778
f_lon=73.84744444444443

ser = serial.Serial()
ser.port = "/dev/ttyS0"
ser.baudrate = 9600
ser.timeout = 1
ser.open()
gpgga = nmea.GPGGA()
def Read_serial():
   while(1):
    data = ser.readline()
    if data[0:6] == '$GPGGA':
        ##method for parsing the sentence
        gpgga.parse(data)
        lats = gpgga.latitude
        #print "Latitude values : " + str(lats)

        lat_dir = gpgga.lat_direction
        #print "Latitude direction : " + str(lat_dir)

        longitude = gpgga.longitude
        #print "Longitude values : " + str(longitude)

        long_dir = gpgga.lon_direction
        #print "Longitude direction : " + str(long_dir)

        time_stamp = gpgga.timestamp
        #print "GPS time stamp : " + str(time_stamp)

        alt = gpgga.antenna_altitude
        #print "Antenna altitude : " + str(alt)

        lat_deg = lats[0:2]
        lat_mins = lats[2:4]
        lat_sec0 = lats[5:]
        print(lat_sec0)
        lat_sec1 = float(lat_sec0)*60/100000
        lat_secs =round(lat_sec1,2)
        lon_deg = longitude[0:3]
        lon_mins = longitude[3:5]
        lon_secs = round(float(longitude[6:])*60/100000, 2)
        #lon_str = lon_deg + u'\N{DEGREE SIGN}' + lon_mins + string.printable[68] + str(lon_secs) + string.printable[63]
        # print "Longitude : " + str(lon_str)
        #latitude_string = lat_deg + u'\N{DEGREE SIGN}' + lat_mins + string.printable[68] + str(lat_secs) + string.printable[63]
        #print "Latitude : " + str(latitude_string)
        lat_dd = float(lat_deg)+float(lat_mins)/60+lat_secs/3600
        lon_dd = float(lon_deg)+float(lon_mins)/60+lon_secs/3600
        return lat_dd,lon_dd,lat_deg,lat_mins,lat_secs,lon_deg,lon_mins,lon_secs
        print(lat_dd)
        print(lon_dd)
        break
#Euclidean Distance
def chk_Radius():
    print ("chk_Radius function running")
    c_lat,c_lon,_,_,_,_,_,_ = Read_serial()
    c_rad = round(((f_lat-c_lat)**2 + (f_lon-c_lon)**2)**1/2,5)
    if c_rad<rad:#if
        
        print ("led turning on")
        GPIO.output(7,False)
        print ("led turned on")
        return c_rad,1  #Return  will break the if loop instruction after this will not get exicutted in the same loop
        

    else:
        
        print ("led turning off")
        GPIO.output(7,True)
        print ("led turned off")
        time.sleep(0.3)
        return c_rad,0 #Return  will break the if loop instruction after this will not get exicutted in the same loop

    
def on_publish(client, userdata, mid):
    print("mid: "+str(mid))
    
def mqtt_send():
    client = paho.Client()
    client.on_publish = on_publish
    client.connect('192.168.137.43', 1883)
    client.loop_start()
    c=0.000
    while True:
        c+=0.001
        c_lat,c_lon,_,_,_,_,_,_ = Read_serial()
        (rc, mid) = client.publish('C_LAT', str(c_lat))
        (rc, mid) = client.publish('C_LON', str(c_lon))
        (rc, mid) = client.publish('RAD_FEED',str("rad_feed"))
        (rc, mid) = client.publish('F_LAT_FEED',str( "f_lat_feed"))
        (rc, mid) = client.publish('F_LON_FEED', str("f_lon_feed"))
        (rc, mid) = client.publish('STATUS',str( "status"))
        time.sleep(0.5)

c_lat,c_lon,lat_deg,lat_mins,lat_secs,lon_deg,lon_mins,lon_secs=Read_serial()
c_rad,status = chk_Radius()
chk_Radius()
