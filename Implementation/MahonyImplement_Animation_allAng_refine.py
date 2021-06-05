import numpy as np
import matplotlib.pyplot as plt
import scipy.linalg as sc
import math
import time
import serial, csv
from matplotlib.animation import FuncAnimation

Kp = 15  #correction term coefficient 
Ki = 8  #Bias update coefficient
Ka = 0.55  #accelerometer term coefficient in correction
Km = 1-Ka    #magnetometer term coefficient in correction
pi = 3.14159265359
datarate = 20 #rate of data collection in Hz
alfam = 0.995  #parameter for low pass filter for magnetic field readings
alfag = 0.995
ndim = 3       #dimention of space
def Vectocrossm(v):
    vx = np.array([[0, -v[2], v[1]],[v[2], 0, -v[0]],[-v[1], v[0], 0]])
    return vx

def Rcapdot(Rcap, Omegam,bcap,correction):   #arguments should be numpy arrays
    global Kp
    return Vectocrossm(Omegam - bcap) + Kp*Vectocrossm(correction)
    
def bcapdot(correction):
    global Ki
    return -Ki*correction
    
def correction(vg,vgcap,vm,vmcap):
    global Ka, Km
    return Ka*np.cross(vg,vgcap) + Km*np.cross(vm,vmcap)    
    
#taken from https://www.learnopencv.com/rotation-matrix-to-euler-angles/ to see how the answers changes if transpose of rotation matrix is used
def rotationMatrixToEulerAngles(R) :
    
    sy = math.sqrt(R[0,0] * R[0,0] +  R[0,1] * R[0,1])
    
    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        x = math.atan2(R[1,2] , R[2,2])
        y = math.atan2(-R[0,2], sy)
        z = math.atan2(R[1,0], R[0,0])
        z = math.atan2(R[0,1], R[0,0])
    else :
        x = math.atan2(-R[2,1], R[1,1])
        y = math.atan2(-R[0,2], sy)
        z = 0
        

    return np.array([x, y, z])

def rotationMatrixToEulerAnglesT(R) :
    
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    
    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
        
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])

#Initializing CSV file to write the data    
head=['time','ax','ay','az','gx','gy','gz','mx','my','mz']
with open('/home/dell/Project/csv/dataLive.csv','w') as myfile:
    writer=csv.writer(myfile)
    writer.writerow(head)



Rcap = np.matrix([[1, 0, 0],[0, 1, 0],[0, 0, 1]])

with open('/home/dell/Project/csv/Rcap1.csv','w') as myfile:
    writer=csv.writer(myfile)
    writer.writerow([Rcap[0,0],Rcap[0,1],Rcap[0,2]])
    writer.writerow([Rcap[1,0],Rcap[1,1],Rcap[1,2]])
    writer.writerow([Rcap[2,0],Rcap[2,1],Rcap[2,2]])

with open('/home/dell/Project/csv/Wmes.csv','w') as myfile:
    writer=csv.writer(myfile)
    writer.writerow(['time','wx', 'wy', 'wz'])    
bcap = np.array([0.1, 0.1, 0.1])
vlinear = np.array([0, 0, 0])    #calculate linear acceleration if required
#vg0 = np.array([[0], [0], [1]])

#Currently, the inertial frame is local NED frame
vg0 = np.array([[0], [0], [-1]])
#vm0 = np.array([[0], [1], [0]])
#vm0 = np.array([[0], [0.7071], [0.7071]])
#vm0 = np.array([[0.8037342120865882], [-0.48580065705143627], [-0.34352443571010244]])
#vm0 = np.array([[131], [94], [157]])
vm0 = np.array([[0.9962486030739544], [-0.0335061189374648], [0.07978759845328776]])
vm0 = vm0/np.linalg.norm(vm0)


# Parameters
x_len = 200         # Number of points to display
y_range = [-180, 180]  # Range of possible Y values to display

# Create figure for plotting
fig1 = plt.figure(1)
ax1 = fig1.add_subplot(1, 1, 1)
xs = list(range(0, 200))
ys1 = [0] * x_len
ax1.set_ylim(y_range)
ax1.set_title(r'$\phi$ (Roll)')

fig2 = plt.figure(2)
ax2 = fig2.add_subplot(1, 1, 1)
xs = list(range(0, 200))
ys2 = [0] * x_len
ax2.set_ylim(y_range)
ax2.set_title(r'$\theta$ (Pitch)')

fig3 = plt.figure(3)
ax3 = fig3.add_subplot(1, 1, 1)
xs = list(range(0, 200))
ys3 = [0] * x_len
ax3.set_ylim(y_range)
ax3.set_title(r'$\Psi$ (Yaw)')

fig4 = plt.figure(4)
ax4 = fig4.add_subplot(1, 1, 1)
xs = list(range(0, 200))
ys4 = [0] * x_len
ax4.set_ylim(y_range)
# Initialize communication with TMP102


# Create a blank line. We will update the line in animate
line1, = ax1.plot(xs, ys1)
line2, = ax2.plot(xs, ys2)
line3, = ax3.plot(xs, ys3)
line4, = ax4.plot(xs, ys4)
# Add labels

i = 0
prevtime = 0
dotR = 0
wmesp = 0
starttime =time.time()
def calculateAng():
        global i, Rcap, prevtime, dotR, wmesp
        ValidData = 0     #to keep track that the reading taken from sensor is valid (can set any other value than 10)
        while ValidData!=10:
            ser = serial.Serial('/dev/ttyACM0',115200)
            s = ser.readline()
            s = str(s)
            s = s.split(',')
            ValidData = len(s)
        
        
        #print('i : ',i)
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
        reading = np.array([t, ax, ay, az, gx, gy, gz, mx, my, mz])
        #print('currtime',t)
        with open('/home/dell/Project/csv/dataLive.csv','a') as myfile:
            writer=csv.writer(myfile)
            writer.writerow(reading)
        
        
        if i == 0:
            nowtime = reading[0]
            
            #print(reading[4:7])
            Omegam = reading[4:7]*pi/180
            
            vgr = reading[1:4]
            vg = vgr/(np.linalg.norm(vgr))

            #vg = np.transpose(vg)
            vmr = np.array([reading[8], reading[7], -reading[9]])  #This is done as the reference frame of accelerometer and magnetometer are not same
            
            vm = vmr/(np.linalg.norm(vmr))
            #vm = vmr                             #for i = 0 low pass filter reading will be the actual reading
            vgcap = np.transpose(Rcap)*vg0
            vgcap = np.array([vgcap[0,0],vgcap[1,0],vgcap[2,0]])

            #Euler angles 
            eulang = rotationMatrixToEulerAngles(Rcap)
            
            
            
            vmcap = np.transpose(Rcap)*vm0
            vmcap = np.array([vmcap[0,0],vmcap[1,0],vmcap[2,0]])
            #print(vmcap)
            wmes = correction(vg, vgcap, vm, vmcap)
            wrow =np.array([t, wmes[0], wmes[1], wmes[2]])
            with open('/home/dell/Project/csv/Wmes.csv','a') as myfile:
                writer=csv.writer(myfile)
                writer.writerow(wrow)
            
            
            #Wmes[i,:] = wmes                      #storing the correction values
            dotR = Rcapdot(Rcap, Omegam, bcap, wmes)
            
            prevtime = nowtime
            
           
            
            
            
        if i != 0:   
            nowtime = reading[0]
            delt = nowtime - prevtime
            #print('delt : ', delt)
            Rcap = Rcap*sc.expm(dotR*delt)
            with open('/home/dell/Project/csv/Rcap1.csv','a') as myfile:
                writer=csv.writer(myfile)
                writer.writerow([0,0,0])
                writer.writerow([Rcap[0,0],Rcap[0,1],Rcap[0,2]])
                writer.writerow([Rcap[1,0],Rcap[1,1],Rcap[1,2]])
                writer.writerow([Rcap[2,0],Rcap[2,1],Rcap[2,2]])
            #print('Rcap : ', Rcap)
#            if nowtime > 10 and nowtime < 20:
#                print(Rcap)
                
            Omegam = reading[4:7]*pi/180  
            
            vgr = reading[1:4]
            
           
            vg = vgr/(np.linalg.norm(vgr))
            
            vmr = np.array([reading[8], reading[7], -reading[9]])  #This is done as the reference frame of accelerometer and magnetometer are not same
            
            
            vm = vmr/(np.linalg.norm(vmr))
            
            vgcap = np.transpose(Rcap)*vg0
            vgcap = np.array([vgcap[0,0],vgcap[1,0],vgcap[2,0]])
            
        
            #Euler angles of transpse of matrix
            eulang = rotationMatrixToEulerAngles(Rcap)
            
    
            vmcap = np.transpose(Rcap)*vm0
            vmcap = np.array([vmcap[0,0],vmcap[1,0],vmcap[2,0]])
        #print(vmcap)
            wmes = correction(vg, vgcap, vm, vmcap)
            #Printing vm, vmcap, vg, vgcap as yaw angle is not correct as given by filter
            if i%50 == 0:
                print('vg : ', vg)
                print('vgcap : ', vgcap)
                print('vm : ', vm)
                print('vmcap : ', vmcap)
                print('wmes : ', wmes)
            wrow =np.array([t, wmes[0], wmes[1], wmes[2]])
            with open('/home/dell/Project/csv/Wmes.csv','a') as myfile:
                writer=csv.writer(myfile)
                writer.writerow(wrow)
            #Wmes[i,:] = wmes 
            dotR = Rcapdot(Rcap, Omegam, bcap, wmes)
            
            prevtime = nowtime
            #print(Thetat)
           
            
        i = i + 1
        return np.array([eulang[0], eulang[1], eulang[2]])*180/pi
        

# This function is called periodically from FuncAnimation
def animate1(j, ys1):

    # Read temperature (Celsius) from TMP102
    
    angle = calculateAng() 
    #temp_c = np.random.random()

    # Add y to list
    ys1.append(angle[0])

    # Limit y list to set number of items
    ys1 = ys1[-x_len:]

    # Update line with new Y values
    line1.set_ydata(ys1)

    #print((time.time() - t1))
    time.sleep(0.001)
    return line1,
    
def animate2(j, ys2):

    # Read temperature (Celsius) from TMP102
    
    angle = calculateAng() 
    #temp_c = np.random.random()

    # Add y to list
    
    ys2.append(angle[1])
    
    # Limit y list to set number of items
    ys2 = ys2[-x_len:]

    # Update line with new Y values
    
    line2.set_ydata(ys2)
    time.sleep(0.001)
    #print((time.time() - t1))
    return line2,
    
def animate3(j, ys3):

    # Read temperature (Celsius) from TMP102
    
    angle = calculateAng() 
    #temp_c = np.random.random()

    # Add y to list

    ys3.append(angle[2])
    # Limit y list to set number of items
    ys3 = ys3[-x_len:]

    # Update line with new Y values
    
    line3.set_ydata(ys3)
    #print((time.time() - t1))
    time.sleep(0.001)
    return line3,

def animate4(j, ys4):

    # Read temperature (Celsius) from TMP102
    
    
    #temp_c = np.random.random()

    # Add y to list
    #global wmesp
    ys4.append(wmesp(1))
    # Limit y list to set number of items
    ys4 = ys4[-x_len:]

    # Update line with new Y values
    
    line4.set_ydata(ys4)
    #print((time.time() - t1))
    time.sleep(0.001)
    return line4,
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
    
ani4 = FuncAnimation(fig4,
    animate4,
    fargs=(ys4,),
    interval=50,
    blit=True)    
plt.show()