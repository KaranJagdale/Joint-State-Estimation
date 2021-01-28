import numpy as np
import matplotlib.pyplot as plt
import scipy.linalg as sc
import math
import time
import serial
from matplotlib.animation import FuncAnimation

Kp = 15  #correction term coefficient 
Ki = 8  #Bias update coefficient
Ka = 0.7  #accelerometer term coefficient in correction
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

Rcap = np.matrix([[1, 0, 0],[0, 1, 0],[0, 0, 1]])
bcap = np.array([0.1, 0.1, 0.1])
vlinear = np.array([0, 0, 0])    #calculate linear acceleration if required
vg0 = np.array([[0], [0], [1]])
vm0 = np.array([[0], [1], [0]])
vm0 = np.array([[0], [0.7071], [0.7071]])

# Parameters
x_len = 200         # Number of points to display
y_range = [-180, 180]  # Range of possible Y values to display

# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
xs = list(range(0, 200))
ys = [0] * x_len
ax.set_ylim(y_range)

# Initialize communication with TMP102


# Create a blank line. We will update the line in animate
line, = ax.plot(xs, ys)

# Add labels
plt.title('TMP102 Temperature over Time')
plt.xlabel('Samples')
plt.ylabel('Temperature (deg C)')
starttime = time.time()
i = 0
prevtime = 0
dotR = 0
def calculateAng():
        global i, Rcap, prevtime, dotR
        ser = serial.Serial('COM3',115200)
        s = ser.readline()
        s = str(s)
        s = s.split(',')
            
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
        
        if i == 0:
            nowtime = reading[0]
            
            #print(reading[4:7])
            Omegam = reading[4:7]*pi/180
            
            vgr = reading[1:4]
            vg = vgr/(np.linalg.norm(vgr))

            #vg = np.transpose(vg)
            vmr = reading[7:]
            vm = vmr/(np.linalg.norm(vmr))
            #vm = vmr                             #for i = 0 low pass filter reading will be the actual reading
            vgcap = np.transpose(Rcap)*vg0
            vgcap = np.array([vgcap[0,0],vgcap[1,0],vgcap[2,0]])
            
            
            
            
            #Euler angles of transpse of matrix
            eulang = rotationMatrixToEulerAngles(Rcap)
            
            
            #print(i)
            #vgcap = np.transpose(vgcap)
            #print(vg)
            #print('Karan')
            #print(vgcap,'vgcap')
                #print(vm)
        #temp = np.cross(vg,vgcap)
                #print(temp)
        #print(Rcap)
            #print(bcap)
            vmcap = np.transpose(Rcap)*vm0
            vmcap = np.array([vmcap[0,0],vmcap[1,0],vmcap[2,0]])
            #print(vmcap)
            wmes = correction(vg, vgcap, vm, vmcap)
            #Wmes[i,:] = wmes                      #storing the correction values
            dotR = Rcapdot(Rcap, Omegam, bcap, wmes)
            
            prevtime = nowtime
            
           
            
            
            
        if i != 0:   
            nowtime = reading[0]
            delt = nowtime - prevtime
            Rcap = Rcap*sc.expm(dotR*delt)
#            if nowtime > 10 and nowtime < 20:
#                print(Rcap)
                
            Omegam = reading[4:7]*pi/180  
            
            vgr = reading[1:4]
            
           
            vg = vgr/(np.linalg.norm(vgr))
            
            vmr = reading[7:]
            
            
            vm = vmr/(np.linalg.norm(vmr))
            
            vgcap = np.transpose(Rcap)*vg0
            vgcap = np.array([vgcap[0,0],vgcap[1,0],vgcap[2,0]])
            
        
            #Euler angles of transpse of matrix
            eulang = rotationMatrixToEulerAngles(Rcap)
            
    
            vmcap = np.transpose(Rcap)*vm0
            vmcap = np.array([vmcap[0,0],vmcap[1,0],vmcap[2,0]])
        #print(vmcap)
            wmes = correction(vg, vgcap, vm, vmcap)
            #Wmes[i,:] = wmes 
            dotR = Rcapdot(Rcap, Omegam, bcap, wmes)
            
            #Rcap = Rcap*sc.expm(np.transpose(Rcap)*dotR*(data[i+1,0] - data[i,0]))
            
            
        #bcap = bcap + dotbcap*(data[i+1,0] - data[i,0])
            
            prevtime = nowtime
            #print(Thetat)
           
            
        i = i + 1
        return eulang[1]
        

# This function is called periodically from FuncAnimation
def animate(i, ys):

    # Read temperature (Celsius) from TMP102
    t1 = time.time()
    temp_c = calculateAng() 
    #temp_c = np.random.random()

    # Add y to list
    ys.append(temp_c)

    # Limit y list to set number of items
    ys = ys[-x_len:]

    # Update line with new Y values
    line.set_ydata(ys)
    print((time.time() - t1))
    return line,

# Set up plot to call animate() function periodically
ani = FuncAnimation(fig,
    animate,
    fargs=(ys,),
    interval=50,
    blit=True)
plt.show()
