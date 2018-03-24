#!/usr/bin/python

import smbus
import math
import os
from gps import *
from time import *
import time
import threading
import time
import serial
import smbus
import time
import datetime

#Open Log File
f=open('DataInLog.txt','a')




radio = serial.Serial(
              
               port='/dev/ttyAMA0',
               baudrate = 9600,
               parity=serial.PARITY_NONE,
               stopbits=serial.STOPBITS_ONE,
               bytesize=serial.EIGHTBITS,
               timeout=1
           )
           counter=0

while True:      

datain=radio.readline()
datainNoComma = datain.split(",")

if int(datainNoComma[7]) == 1:
  doSomething.on()
  f.write(1 + "\n")
  
if int(datainNoComma[7]) == 0:
  doSomething.off()
  f.write(0 + "\n")

now = datetime.datetime.now()
timestamp = now.strftime("%H:%M")
#Timestamp,Strattoaltitude,GPSAlt,Latitude,Longitude,Xaccel,Yaccel,Zaccel,Xrot,Yrot,Zrot
outstring = str(timestamp)+","+str(datain)+"\n"
f.write(outstring)
    
    
except (KeyboardInterrupt, SystemExit): #when you press ctrl+c
    print "\nKilling Thread..."
    print "Done.\nExiting."    
    
    
    
    
    
f.close()

  


