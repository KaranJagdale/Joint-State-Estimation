import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import serial, time
import math, csv
import datetime

datetime_object = datetime.datetime.now()


alfa = 0.99
A_lp = []
instants = []
g0 = 9.81
n = 200
head=['date-time','mx','my','mz']
with open('/home/dell/Project/csv/mag.csv','a') as myfile:
    writer=csv.writer(myfile)
    writer.writerow(head)
def main():
    smx,smy,smz = 0, 0, 0
    i = 0
    while i<n:
        ser = serial.Serial('/dev/ttyACM0',115200)
        s = ser.readline()
        s = str(s)
        #print(s)
        s = s.split(',')
        mx = float(s[7])
        my = float(s[8])
        mz = float(s[9][:(len(s[9])-5)])
        smx = smx + mx
        smy = smy + my
        smz = smz + mz        
        i = i + 1
        time.sleep(0.1)
    amy,amx,amz = smy/n, smx/n, smz/n
    n_am = math.sqrt(amx*amx + amy*amy + amz*amz)
    m = [amy/n_am, amx/n_am, -amz/n_am]   #In accelerometer frame
    row = [datetime_object, m[0], m[1], m[2]]
    with open('/home/dell/Project/csv/mag.csv','a') as myfile:
        writer=csv.writer(myfile)
        writer.writerow(row)
    
    print('In accelerometer frame : ', m)
if __name__ == '__main__':
    main()
