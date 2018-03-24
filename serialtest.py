#!/usr/bin/python

import smbus
import math
import os
from gps import *
import threading
import serial
import smbus
import time
import datetime
from decimal import *

#time to wait between loops
sleeptime=0.0
#environmental conditions
g=32.2
Drag=.43
Area=.196
Mass=72

#Text file settings
FORMAT = '%H%M%S'
path = 'Rocketlog.txt'
new_path = '%s_%s' % (datetime.datetime.now().strftime(FORMAT), path)
f=open(new_path, 'a')

#opening serial port for altimeter
altimeter = serial.Serial(
              
               port='/dev/ttyAMA0',
               baudrate = 9600,
               parity=serial.PARITY_NONE,
               stopbits=serial.STOPBITS_ONE,
               bytesize=serial.EIGHTBITS,
               timeout=.15
           )

#setting variables
Alt_proj=0
Alt_current=0
endtime=0
Second_Alt = 0
altcount=0
ready = ''
check=0


#Accelerometer Initialization
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

def read_byte(adr):
    return bus.read_byte_data(address, adr)

def read_word(adr):
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr+1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def dist(a,b):
    return math.sqrt((a*a)+(b*b))

def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)

def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)

bus = smbus.SMBus(1) # or bus = smbus.SMBus(1) for Revision 2 boards
address = 0x68       # This is the address value read via the i2cdetect command

# Now wake the 6050 up as it starts in sleep mode
bus.write_byte_data(address, power_mgmt_1, 0)




#initiate brake variable storage to closed
brakes=open('brakes_control.txt','w')
brakes.write('0')
brakes.close()
#Initiate main loop
while check ==0:
	try:
		ready=int(altimeter.readline())
		check=1
	except ValueError:
		print 'Waiting For Stratologger to boot..'


while True:


    
    #############################################################################################
    #check for stratologger activity
    while True:
          try:
             ready=int(altimeter.readline())
             print 'StratoLogger Online...'
             #Take first time/alt measurement 
             starttime=Decimal((time.time())).quantize(Decimal(10) ** -5)
             try:
                      Alt_current =int(altimeter.readline())
             except ValueError:
                      print 'Error Grabing First Altitude...'
             #Sleep for sleeptime
             time.sleep(sleeptime)  
             #Take second time/alt measurement
             endtime=Decimal((time.time())).quantize(Decimal(10) ** -5)
             try:
                      Second_Alt =int(altimeter.readline())
             except ValueError:
                      print 'Error Grabing Second Altitude...'
             #Calculate velocity 
             V_current = Decimal((abs(Alt_current - Second_Alt))/(abs(starttime-endtime))).quantize(Decimal(10) ** -2)
             #print V_current
             #Calulate density
             Density =0.0000000000004*(int(Second_Alt)**(2))-.00000006769*int(Second_Alt)+0.002374
             #Calculate predicted apogee
             Alt_proj = int(Second_Alt) +int( ((int(V_current)**(2))) / int((2*((int(g)+int(Drag)*((float(Density)*(int(V_current)**(2)))/2*int(Area))/int(Mass))))))
             #Print for debugging 
             timetest = abs(starttime-endtime)
             #print 'velocity: ',V_current,'alt projection: ',Alt_proj,' time calc: ',timetest,'\n'
             brakes=open('brakes_control.txt','r')
             poll = brakes.read()
    
             if Alt_proj >= 30000 and int(poll) != 1:
                brakes=open('brakes_control.txt','w')
                brakes.write('1')
                brake=1
                print 'open command sent'
                brakes.close()
             if Alt_proj <= 30000 and int(poll) != 0:
                brakes=open('brakes_control.txt','w')
                brakes.write('0')
                brake=0
                brakes.close()
                print 'close command sent'

             try:
                 gyro_xout = int(read_word_2c(0x43))
             except Null_xout:
                 gyro_xout = 0
             try:
                 gyro_yout = int(read_word_2c(0x45))
             except Null_yout:
                 gyro_yout = 0
             try:
                 gyro_zout = int(read_word_2c(0x47))
             except Null_zout:
                 gyro_zout = 0

             try:
                 accel_xout = int(read_word_2c(0x3b))
             except Null_xout:
                 accel_xout = 0
             try:
                 accel_yout = int(read_word_2c(0x3d))
             except Null_yout:
                 accel_yout = 0
             try:
                 accel_zout = int(read_word_2c(0x3f))
             except Null_zout:
                 accel_zout = 0
        

             accel_xout_scaled = accel_xout / 16384.0
             accel_yout_scaled = accel_yout / 16384.0
             accel_zout_scaled = accel_zout / 16384.0
             #############################################################################################
             #Setup output to text file
             now=datetime.datetime.now()
             timestamp=now.strftime("%H:%M:%S")
             writestring = str(timestamp)+","+str(Alt_current)+","+str(V_current)+","+str(Alt_proj)+","+str(accel_xout_scaled)+","+str(accel_yout_scaled)+","+str(accel_zout_scaled)+","+"\n"
             f.write(writestring)
          except ValueError:
             print 'Stratologger failed...'
             #Take first time/alt measurement 
             Second_Alt =str('N/A')
             V_current = str('N/A')
             Alt_proj = str('N/A')
             brakes=open('brakes_control.txt','w')
             brakes.write('0')
             brake=0
             print 'close command sent'

             try:
                 gyro_xout = int(read_word_2c(0x43))
             except Null_xout:
                 gyro_xout = 0
             try:
                 gyro_yout = int(read_word_2c(0x45))
             except Null_yout:
                 gyro_yout = 0
             try:
                 gyro_zout = int(read_word_2c(0x47))
             except Null_zout:
                 gyro_zout = 0

             try:
                 accel_xout = int(read_word_2c(0x3b))
             except Null_xout:
                 accel_xout = 0
             try:
                 accel_yout = int(read_word_2c(0x3d))
             except Null_yout:
                 accel_yout = 0
             try:
                 accel_zout = int(read_word_2c(0x3f))
             except Null_zout:
                 accel_zout = 0
             accel_xout_scaled = accel_xout / 16384.0
             accel_yout_scaled = accel_yout / 16384.0
             accel_zout_scaled = accel_zout / 16384.0
             #############################################################################################
             #Setup output to text file
             now=datetime.datetime.now()
             timestamp=now.strftime("%H:%M:%S")
             writestring = str(timestamp)+","+str(Alt_current)+","+str(V_current)+","+str(Alt_proj)+","+str(accel_xout_scaled)+","+str(accel_yout_scaled)+","+str(accel_zout_scaled)+","+"\n"
             f.write(writestring)
    
   
    
     	
    
    


   
                                                                         

