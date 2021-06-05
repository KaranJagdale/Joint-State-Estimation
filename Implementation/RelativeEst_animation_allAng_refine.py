import numpy as np
import matplotlib.pyplot as plt
import scipy.linalg as sc
import math
import time
import serial, csv
from matplotlib.animation import FuncAnimation

Kp = 15  #correction term coefficient 
Ki = 8  #Bias update coefficient
Ka = 0.6  #accelerometer term coefficient in correction
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
with open('/home/dell/Project/csv/dataLive1.csv','w') as myfile:
    writer=csv.writer(myfile)
    writer.writerow(head)

head=['time','ax','ay','az','gx','gy','gz','mx','my','mz']
with open('/home/dell/Project/csv/dataLive2.csv','w') as myfile:
    writer=csv.writer(myfile)
    writer.writerow(head)
    
Rcap1 = np.matrix([[1, 0, 0],[0, 1, 0],[0, 0, 1]])
Rcap2 = np.matrix([[1, 0, 0],[0, 1, 0],[0, 0, 1]])
Rrel = np.transpose(Rcap1)*Rcap2
bcap1 = np.array([0.1, 0.1, 0.1])
bcap2 = np.array([0.1, 0.1, 0.1])
vlinear = np.array([0, 0, 0])    #calculate linear acceleration if required
vg0 = np.array([[0], [0], [1]])
vm0 = np.array([[0], [1], [0]])
vm0 = np.array([[0], [0.7071], [0.7071]])

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

#fig4 = plt.figure(4)
#ax4 = fig4.add_subplot(1, 1, 1)
#xs = list(range(0, 200))
#ys4 = [0] * x_len
#ax4.set_ylim(y_range)
# Initialize communication with TMP102


# Create a blank line. We will update the line in animate
line1, = ax1.plot(xs, ys1)
line2, = ax2.plot(xs, ys2)
line3, = ax3.plot(xs, ys3)
#line4, = ax4.plot(xs, ys4)
# Add labels

i = 0
prevtime1 = 0
prevtime2 = 0
dotR = 0
wmesp = 0
starttime =time.time()
def calculateAng():
        global i, Rcap1, Rcap2, prevtime1, prevtime2, dotR1, wmesp, dotR2, Rrel
        ValidData = 0     #to keep track that the reading taken from sensor is valid (can set any other value than 10)
        
        #reading IMU1 Data
        while ValidData!=10:
            ser1 = serial.Serial('/dev/ttyACM0',115200)
            s1 = ser1.readline()
            s1 = str(s1)
            s1 = s1.split(',')
            ValidData = len(s1)
        t1 = time.time() - starttime         
        #reading IMU2 Data  
        ValidData = 0   #Necesarry if not done next while loop will be skipped 
        while ValidData!=10:
            ser2 = serial.Serial('/dev/ttyACM1',115200)
            s2 = ser2.readline()
            s2 = str(s2)
            s2 = s2.split(',')
            ValidData = len(s2)
        t2 = time.time() - starttime 
        
        #print(s2)
        
        ax = float(s1[1])
        ay = float(s1[2])
        az = float(s1[3])
        gx = float(s1[4])
        gy = float(s1[5])
        gz = float(s1[6])
        mx = float(s1[7])
        my = float(s1[8])
        mz = float(s1[9][:(len(s1[9])-5)])  #to remove trailing elements
        reading1 = np.array([t1, ax, ay, az, gx, gy, gz, mx, my, mz])
        
            
        ax = float(s2[1])
        ay = float(s2[2])
        az = float(s2[3])
        gx = float(s2[4])
        gy = float(s2[5])
        gz = float(s2[6])
        mx = float(s2[7])
        my = float(s2[8])
        mz = float(s2[9][:(len(s2[9])-5)])  #to remove trailing elements
        reading2 = np.array([t2, ax, ay, az, gx, gy, gz, mx, my, mz])
        #print('currtime',t)
        with open('/home/dell/Project/csv/dataLive1.csv','a') as myfile:
            writer=csv.writer(myfile)
            writer.writerow(reading1)
        
        with open('/home/dell/Project/csv/dataLive2.csv','a') as myfile:
            writer=csv.writer(myfile)
            writer.writerow(reading2)
        
        if i == 0:
            nowtime1 = reading1[0]
            nowtime2 = reading2[0]
            #IMU1 calculations
            Omegam1 = reading1[4:7]*pi/180
            
            vgr1 = reading1[1:4]

            vg1 = vgr1/(np.linalg.norm(vgr1))

            #vg = np.transpose(vg)
            vmr1 = reading1[7:]
            vm1 = vmr1/(np.linalg.norm(vmr1))
            #vm = vmr                             #for i = 0 low pass filter reading will be the actual reading
            vgcap1 = np.transpose(Rcap1)*vg0
            vgcap1 = np.array([vgcap1[0,0],vgcap1[1,0],vgcap1[2,0]])

            #Euler angles 
            eulang1 = rotationMatrixToEulerAngles(Rcap1)
            eulang2 = rotationMatrixToEulerAngles(Rcap2)
            eulang = rotationMatrixToEulerAngles(Rrel)
            
            
            vmcap1 = np.transpose(Rcap1)*vm0
            vmcap1 = np.array([vmcap1[0,0],vmcap1[1,0],vmcap1[2,0]])
            #print(vmcap)
            wmes1 = correction(vg1, vgcap1, vm1, vmcap1)
            wmesp = wmes1
            #Wmes[i,:] = wmes                      #storing the correction values
            dotR1 = Rcapdot(Rcap1, Omegam1, bcap1, wmes1)
            
            
            #IMU2 calculations
            Omegam2 = reading2[4:7]*pi/180
            
            vgr2 = reading2[1:4]
            vg2 = vgr2/(np.linalg.norm(vgr2))

            #vg = np.transpose(vg)
            vmr2 = reading2[7:]
            vm2 = vmr2/(np.linalg.norm(vmr2))
            #vm = vmr                             #for i = 0 low pass filter reading will be the actual reading
            vgcap2 = np.transpose(Rcap2)*vg0
            vgcap2 = np.array([vgcap2[0,0],vgcap2[1,0],vgcap2[2,0]])

            #Euler angles 
            eulang2 = rotationMatrixToEulerAngles(Rcap2)
            
            
            
            vmcap2 = np.transpose(Rcap2)*vm0
            vmcap2 = np.array([vmcap2[0,0],vmcap2[1,0],vmcap2[2,0]])
            #print(vmcap)
            wmes2 = correction(vg2, vgcap2, vm2, vmcap2)
            wmesp = wmes2
            #Wmes[i,:] = wmes                      #storing the correction values
            dotR2 = Rcapdot(Rcap2, Omegam2, bcap2, wmes2)
            
            prevtime1 = nowtime1
            prevtime2 = nowtime2
           
            
            
            
        if i != 0:   
            nowtime1 = reading1[0]
            nowtime2 = reading2[0]
            delt1 = nowtime1 - prevtime1
            delt2 = nowtime2 - prevtime2
            print('delt1 : ', delt1)
            Rcap1 = Rcap1*sc.expm(dotR1*delt1)
            Rcap2 = Rcap2*sc.expm(dotR2*delt2)
            Rrel = np.transpose(Rcap1)*Rcap2
#            if nowtime > 10 and nowtime < 20:
#                print(Rcap)
                
            Omegam1 = reading1[4:7]*pi/180
            
            vgr1 = reading1[1:4]
            vg1 = vgr1/(np.linalg.norm(vgr1))

            #vg = np.transpose(vg)
            vmr1 = reading1[7:]
            vm1 = vmr1/(np.linalg.norm(vmr1))
            #vm = vmr                             #for i = 0 low pass filter reading will be the actual reading
            vgcap1 = np.transpose(Rcap1)*vg0
            vgcap1 = np.array([vgcap1[0,0],vgcap1[1,0],vgcap1[2,0]])

            #Euler angles 
            eulang1 = rotationMatrixToEulerAngles(Rcap1)
            eulang2 = rotationMatrixToEulerAngles(Rcap2)
            eulang = rotationMatrixToEulerAngles(Rrel)
            
            
            vmcap1 = np.transpose(Rcap1)*vm0
            vmcap1 = np.array([vmcap1[0,0],vmcap1[1,0],vmcap1[2,0]])
            #print(vmcap)
            wmes1 = correction(vg1, vgcap1, vm1, vmcap1)
            wmesp = wmes1
            #Wmes[i,:] = wmes                      #storing the correction values
            dotR1 = Rcapdot(Rcap1, Omegam1, bcap1, wmes1)
            
            Omegam2 = reading2[4:7]*pi/180
            
            vgr2 = reading2[1:4]
            vg2 = vgr2/(np.linalg.norm(vgr2))

            #vg = np.transpose(vg)
            vmr2 = reading2[7:]
            vm2 = vmr2/(np.linalg.norm(vmr2))
            #vm = vmr                             #for i = 0 low pass filter reading will be the actual reading
            vgcap2 = np.transpose(Rcap2)*vg0
            vgcap2 = np.array([vgcap2[0,0],vgcap2[1,0],vgcap2[2,0]])

            #Euler angles 
            
            
            
            
            vmcap2 = np.transpose(Rcap2)*vm0
            vmcap2 = np.array([vmcap2[0,0],vmcap2[1,0],vmcap2[2,0]])
            #print(vmcap)
            wmes2 = correction(vg2, vgcap2, vm2, vmcap2)
            wmesp = wmes2
            #Wmes[i,:] = wmes                      #storing the correction values
            dotR2 = Rcapdot(Rcap2, Omegam2, bcap2, wmes2)
            
            prevtime1 = nowtime1
            prevtime2 = nowtime2
            #print(Thetat)
           
            
        i = i + 1
        #Currently all angles (absolute and relative) are present in the return value of the function
        return np.array([eulang[0], eulang[1], eulang[2], eulang1[0], eulang1[1], eulang1[2], eulang2[0], eulang2[1], eulang2[2]])*180/pi
        

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
    time.sleep(0.003)
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
    time.sleep(0.003)
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
    time.sleep(0.003)
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
    time.sleep(0.003)
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
    
#ani4 = FuncAnimation(fig4,
#    animate4,
#    fargs=(ys4,),
#    interval=50,
#    blit=True)    
plt.show()