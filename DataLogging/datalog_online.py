import numpy as np
import matplotlib.pyplot as plt
import scipy.linalg as sc
import math
import time, csv
import serial
from matplotlib.animation import FuncAnimation



# Parameters
x_len = 200         # Number of points to display
y_range = [-1000, 1000]  # Range of possible Y values to display

# Create figure for plotting
fig1 = plt.figure(1)
ax1 = fig1.add_subplot(1, 1, 1)
xs = list(range(0, 200))
ys1 = [0] * x_len
ax1.set_ylim(y_range)
#ax1.set_title(r'$\phi$ (Roll)')

fig2 = plt.figure(2)
ax2 = fig2.add_subplot(1, 1, 1)
xs = list(range(0, 200))
ys2 = [0] * x_len
ax2.set_ylim(y_range)
#ax2.set_title(r'$\theta$ (Pitch)')

fig3 = plt.figure(3)
ax3 = fig3.add_subplot(1, 1, 1)
xs = list(range(0, 200))
ys3 = [0] * x_len
ax3.set_ylim(y_range)
#ax3.set_title(r'$\Psi$ (Yaw)')



# Create a blank line. We will update the line in animate
line1, = ax1.plot(xs, ys1)
line2, = ax2.plot(xs, ys2)
line3, = ax3.plot(xs, ys3)
# Add labels
head=['time','ax','ay','az','gx','gy','gz','mx','my','mz']
with open('/home/dell/Project/csv/dataOnline.csv','w') as myfile:
    writer=csv.writer(myfile)
    writer.writerow(head)

starttime = time.time()

def calculateAng():
    global starttime
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
    with open('/home/dell/Project/csv/dataOnline.csv','a') as myfile:
        writer=csv.writer(myfile)
        writer.writerow(row)
    return row
        

# This function is called periodically from FuncAnimation
def animate1(j, ys1):

    # Read temperature (Celsius) from TMP102
    
    reading = calculateAng() 
    #temp_c = np.random.random()

    # Add y to list
    ys1.append(reading[4])

    # Limit y list to set number of items
    ys1 = ys1[-x_len:]

    # Update line with new Y values
    line1.set_ydata(ys1)

    #print((time.time() - t1))
    time.sleep(0.003)
    return line1,
    
def animate2(j, ys2):

    # Read temperature (Celsius) from TMP102
    
    reading = calculateAng() 
    #temp_c = np.random.random()

    # Add y to list
    
    ys2.append(reading[5])
    
    # Limit y list to set number of items
    ys2 = ys2[-x_len:]

    # Update line with new Y values
    
    line2.set_ydata(ys2)
    time.sleep(0.003)
    #print((time.time() - t1))
    return line2,
    
def animate3(j, ys3):

    # Read temperature (Celsius) from TMP102
    
    reading = calculateAng() 
    #temp_c = np.random.random()

    # Add y to list

    ys3.append(reading[6])
    # Limit y list to set number of items
    ys3 = ys3[-x_len:]

    # Update line with new Y values
    
    line3.set_ydata(ys3)
    #print((time.time() - t1))
    time.sleep(0.003)
    return line3,

# Set up plot to call animate() function periodically
ani1 = FuncAnimation(fig1,
    animate1,
    fargs=(ys1,),
    interval=50,
    blit=True)
    
ani2 = FuncAnimation(fig2,
    animate2,
    fargs=(ys2,),
    interval=50,
    blit=True)
    
ani3 = FuncAnimation(fig3,
    animate3,
    fargs=(ys3,),
    interval=50,
    blit=True)
plt.show()