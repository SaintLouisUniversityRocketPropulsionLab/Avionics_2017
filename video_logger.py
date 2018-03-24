import time
import os

gettime = time.strftime("%Y%m%d-%H%M%S")

mydir = "/home/pi/LoggerData/"+gettime
os.makedirs(mydir)

runvid = ("sudo raspivid -o "+mydir+"/video -t 10000 ")
os.system(runvid)
