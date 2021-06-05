import serial
import time
import numpy as np
import csv

t0 = 0
tf = 20
ts = 0.04 #40 Hz

T = np.linspace(t0,tf,int((-t0+tf)/ts)+1)

head=['time','ax','ay','az','gx','gy','gz','mx','my','mz']
with open('/home/dell/Project/csv/datatemp.csv','w') as myfile:
    writer=csv.writer(myfile)
    writer.writerow(head)
    
starttime =time.time()

for i in T:
    ValidData = 0
    while ValidData!=10:
            ser = serial.Serial('/dev/ttyACM0',115200)
            s = ser.readline()
            s = str(s)
            s = s.split(',')
            ValidData = len(s)

    
    t = time.time() - starttime
    ax = float(s[1])
    ay = float(s[2])
    az = float(s[3])
    gx = float(s[4])
    gy = float(s[5])
    gz = float(s[6])
    mx = float(s[7])
    my = float(s[8])
    mz = float(s[9][:(len(s[9])-5)])  #to remove trailing elements
    #mz = float(s[9])
    
    #print(mx,my,mz,ax,ay,az)
    
    #print(mx,my,mz)
    
    #print(mz,mx,my)
    row = [t, ax, ay, az, gx, gy, gz, mx, my, mz]
    
    print(row)
    with open('/home/dell/Project/csv/datatemp.csv','a') as myfile:
        writer=csv.writer(myfile)
        writer.writerow(row)
    #print(ax)
    time.sleep(ts*25)
    
