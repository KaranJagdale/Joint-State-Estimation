import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import scipy.linalg as sc
import math

#Kp = 5, Ki =2, ka =0.8 km = 0.2 works good 
#Best case till now Kp = 15, Ki =2, ka =0.8 km = 0.2
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
    
def main():
    df = pd.read_csv('/home/dell/Project/csv/datachair.csv') 
    data = np.array(df)
    Rcap = np.matrix([[1, 0, 0],[0, 1, 0],[0, 0, 1]])
    bcap = np.array([0.1, 0.1, 0.1])
    vlinear = np.array([0, 0, 0])    #calculate linear acceleration if required
    vg0 = np.array([[0], [0], [1]])
    vm0 = np.array([[0], [1], [0]])
    vm0 = np.array([[0], [0.7071], [0.7071]])
    Theta = []
    Instants = []
    B1cap = []
    B2cap = []
    B3cap = []
    Psi = []
    Phi= []
    #Euler angle of transpose
    Thetat = []
    Psit = []
    Phit = []
    Vm_lp = []           #to store the low pass filtered values of magnetic field
    vm_lp = np.array([0,0,0])          #for initial value of low pass filter  (there is reason why this is set to zero)
    vg_lp = np.array([0,0,0])    
    Vg_lp = []
    T = []                #used to store the actual time instants
    Wmes = np.zeros((data.shape[0]-1,ndim))             #used to store the correction term
    #print('a')
    print(type(data[0,:]))
    print(data.shape[0])
    for i in range(data.shape[0]-1):
        if i == 0:
            reading = data[i,:]            #here data is arranged as: timestamp, ax, ay, az, gx, gy, gz, mx, my, mz
            nowtime = reading[0]
            T.append(nowtime)
            Omegam = reading[4:7]*pi/180       
            vgr = reading[1:4]
            vgr = vgr/(np.linalg.norm(vgr))
            vg_lp = vg_lp*alfag + vgr*(1-alfag) 
            vg = vg_lp/(np.linalg.norm(vg_lp))
            #vg = np.transpose(vg)
            vmr = reading[7:]
            vmr = vmr/(np.linalg.norm(vmr))
            vm_lp = vm_lp*alfam + vmr*(1-alfam)        
            vm = vm_lp/(np.linalg.norm(vm_lp))
            #vm = vmr                             #for i = 0 low pass filter reading will be the actual reading
            vgcap = np.transpose(Rcap)*vg0
            vgcap = np.array([vgcap[0,0],vgcap[1,0],vgcap[2,0]])
            theta = np.arcsin(-Rcap[0,2])
            Theta.append(theta)
            psi = np.arctan2(Rcap[0,1],Rcap[0,0])
            Psi.append(psi)
            phi = np.arctan2(Rcap[1,2],Rcap[2,2])
            Phi.append(phi)
            Instants.append(i)
            
            #Euler angles of transpse of matrix
            eulang = rotationMatrixToEulerAngles(Rcap)
            Thetat.append(eulang[1])
            Phit.append(eulang[0])
            Psit.append(eulang[2])
            B1cap.append(bcap[0])
            B2cap.append(bcap[1])
            B3cap.append(bcap[2])
            
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
            Wmes[i,:] = wmes                      #storing the correction values
            dotR = Rcapdot(Rcap, Omegam, bcap, wmes)
            dotbcap = bcapdot(wmes)    
            prevtime = nowtime
            
            
        if i != 0:   
            reading = data[i,:]            #here data is arranged as: timestamp, ax, ay, az, gx, gy, gz, mx, my, mz
            nowtime = reading[0]
            T.append(nowtime)
            delt = nowtime - prevtime
            Rcap = Rcap*sc.expm(dotR*delt)
#            if nowtime > 10 and nowtime < 20:
#                print(Rcap)
                
            Omegam = reading[4:7]*pi/180  
            
            vgr = reading[1:4]
            
            vg_lp = vg_lp*alfag + vgr*(1-alfag) 
            vg = vg_lp/(np.linalg.norm(vg_lp))
            vg = vgr/(np.linalg.norm(vgr))
            
            vmr = reading[7:]
            
            vm_lp = vm_lp*alfam + vmr*(1-alfam)        
            vm = vm_lp/(np.linalg.norm(vm_lp))
            vm = vmr/(np.linalg.norm(vmr))
            
            vgcap = np.transpose(Rcap)*vg0
            vgcap = np.array([vgcap[0,0],vgcap[1,0],vgcap[2,0]])
            theta = np.arcsin(-Rcap[0,2])
            Theta.append(theta)
            psi = np.arctan2(Rcap[0,1],Rcap[0,0])
            Psi.append(psi)
            phi = np.arctan2(Rcap[1,2],Rcap[2,2])
            Phi.append(phi)
            Instants.append(i)
        
            #Euler angles of transpse of matrix
            eulang = rotationMatrixToEulerAngles(Rcap)
            Thetat.append(eulang[1])
            Phit.append(eulang[0])
            Psit.append(eulang[2])
        
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
            Wmes[i,:] = wmes 
            dotR = Rcapdot(Rcap, Omegam, bcap, wmes)
            dotbcap = bcapdot(wmes)
            #Rcap = Rcap*sc.expm(np.transpose(Rcap)*dotR*(data[i+1,0] - data[i,0]))
            
            bcap = bcap + dotbcap*delt
            #print(sc.det(Rcap),'det')
            B1cap.append(bcap[0])
            B2cap.append(bcap[1])
            B3cap.append(bcap[2])
        #bcap = bcap + dotbcap*(data[i+1,0] - data[i,0])
            
            prevtime = nowtime
        
    
    Instants = np.array(Instants)/datarate   #to convert instants to time instants
    T = np.array(T)
    B1cap = np.array(B1cap)*180/pi
    B2cap = np.array(B2cap)*180/pi
    B3cap = np.array(B3cap)*180/pi
    
    Theta = np.array(Theta)*180/pi
    Psi = np.array(Psi)*180/pi
    Phi = np.array(Phi)*180/pi
    
    Thetat = np.array(Thetat)*180/pi
    Psit = np.array(Psit)*180/pi
    Phit = np.array(Phit)*180/pi
    Wmes = np.transpose(Wmes)
    print(Wmes[1,:].shape)  
    print(T.shape)
    
    print('alpha')
    plt.figure(1)
    plt.plot(T,Thetat)
    fig1 = plt.figure(1)
    fig1.suptitle(r'$\theta$ vs time')
    plt.ylabel(r'$\theta (^{o})$')
    plt.xlabel('time (s)')
    #plt.savefig('theta.png')
    
    print('psi')
    plt.figure(2)
    plt.plot(T,Psit)
    fig2 = plt.figure(2)
    fig2.suptitle(r'$\psi$ vs time')
    plt.ylabel('Theta(Degrees)')
    plt.xlabel('time (s)')
    #plt.savefig('psi.png')
    
    
    print('phi')
    plt.figure(3)
    plt.plot(T,Phit)
    fig3 = plt.figure(3)
    fig3.suptitle(r'$\phi$ vs time')
    plt.ylabel('Theta(Degrees)')
    plt.xlabel('time (s)')
    #plt.savefig('phi.png')
    
    print('b1cap')
    plt.figure(4)
    plt.plot(T,B1cap)
    fig4 = plt.figure(4)
    fig4.suptitle('$\hat{b}_1$ vs time')
    plt.xlabel('time (s)')
    plt.ylabel(r'$\hat{b}_1({^o/s})$')
    #plt.savefig('b1.png')
    
    print('b2cap')
    plt.figure(5)
    plt.plot(T,B2cap)
    fig5 = plt.figure(5)
    fig5.suptitle('$\hat{b}_2$ vs time')
    plt.xlabel('time (s)')
    plt.ylabel(r'$\hat{b}_2({^o/s})$')
    #plt.savefig('b2.png')
    
    
    print('b3cap')
    plt.figure(6)
    plt.plot(T,B3cap)
    fig6 = plt.figure(6)
    fig6.suptitle('$\hat{b}_3$ vs time')
    plt.xlabel('time (s)')
    plt.ylabel(r'$\hat{b}_3({^o/s})$')
    
    plt.figure(7)
    plt.plot(T,Wmes[0,:])
    fig7 = plt.figure(7)
    fig7.suptitle('$\omega_{mes}1$ vs time')
    plt.xlabel('time (s)')
    
    plt.figure(8)
    plt.plot(T,Wmes[1,:])
    fig8 = plt.figure(8)
    fig8.suptitle('$\omega_{mes}2$ vs time')
    plt.xlabel('time (s)')

    plt.figure(9)
    plt.plot(T,Wmes[2,:])
    fig9 = plt.figure(9)
    fig9.suptitle('$\omega_{mes}3$ vs time')
    plt.xlabel('time (s)')
    plt.show()
    
if __name__ == '__main__':
    main()
