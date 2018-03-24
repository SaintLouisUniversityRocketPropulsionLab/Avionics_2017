# Raspberry Pi Temperature Logger
# Zagros Robotics, Inc.
# www.zagrosrobotics.com
# 6/25/2013

import smbus
import time
import datetime

#SMBus(0) - Raspberry Pi Model A
#SMBus(1) - Raspberry Pi Model B

bus = smbus.SMBus(1)

#I2C address of sensor
address = 0x48

def temperature():
    rvalue0 = bus.read_word_data(address,0)
    rvalue1 = (rvalue0 & 0xff00) >> 8
    rvalue2 = rvalue0 & 0x00ff
    rvalue = (((rvalue2 * 256) + rvalue1) >> 4 ) *.0625
    #print rvalue1, rvalue2
    return rvalue

print("Temperature Data Logger\n")

while True:
    
    #Open Log File
    f=open('tempdata.txt','a')
    now = datetime.datetime.now()
    timestamp = now.strftime("%Y/%m/%d %H:%M")
    outvalue = temperature()
    outstring = str(timestamp)+"  "+str(outvalue)+" C "+str(outvalue*1.8+32)+" F"+"\n"
    print outstring
    f.write(outstring)
    f.close()

    #log temperature every 60 seconds
    time.sleep(60)
    
