import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import scipy.linalg as sc

#Kp = 5, Ki =2, ka =0.8 km = 0.2 works good 
#Best case till now Kp = 15, Ki =2, ka =0.8 km = 0.2
Kp = 15  #correction term coefficient 
Ki = 5    #Bias update coefficient
Ka = 0.8      #accelerometer term coefficient in correction
Km = 0.2      #magnetometer term coefficient in correction
pi = 3.14159265359
datarate = 20 #rate of data collection in Hz
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
    
def main():
    df = pd.read_csv('/home/dell/Project/Joint-State-Estimation-main/Data221020_1504.csv') 
    data = np.array(df)
    Rcap = np.matrix([[1, 0, 0],[0, 1, 0],[0, 0, 1]])
    bcap = np.array([0.1, 0.1, 0.1])
    vlinear = np.array([0, 0, 0])    #calculate linear acceleration if required
    vg0 = np.array([[0], [0], [1]])
    vm0 = np.array([[0], [1], [0]])
    Theta = []
    Instants = []
    B1cap = []
    B2cap = []
    B3cap = []
    Psi = []
    Phi= []
    #print('a')
    print(type(data[0,:]))
    print(data.shape[0])
    for i in range(data.shape[0]-1):
        reading = data[i,:]            #here data is arranged as: timestamp, ax, ay, az, gx, gy, gz, mx, my, mz
        Omegam = reading[4:7]*pi/180       
        vg = reading[1:4]
        vg = vg/(np.linalg.norm(vg))
        #vg = np.transpose(vg)
        vm = reading[7:]
        vm = vm/(np.linalg.norm(vm))
        vgcap = Rcap*vg0
        vgcap = np.array([vgcap[0,0],vgcap[1,0],vgcap[2,0]])
        theta = np.arcsin(-Rcap[0,2])
        Theta.append(theta)
        psi = np.arctan2(Rcap[0,1],Rcap[0,0])
        Psi.append(psi)
        phi = np.arctan2(Rcap[1,2],Rcap[2,2])
        Phi.append(phi)
        Instants.append(i)
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
        vmcap = Rcap*vm0
        vmcap = np.array([vmcap[0,0],vmcap[1,0],vmcap[2,0]])
        #print(vmcap)
        wmes = correction(vg, vgcap, vm, vmcap)
        dotR = Rcapdot(Rcap, Omegam, bcap, wmes)
        dotbcap = bcapdot(wmes)
        #Rcap = Rcap*sc.expm(np.transpose(Rcap)*dotR*(data[i+1,0] - data[i,0]))
        Rcap = Rcap*sc.expm(dotR*0.05)
        
        #print(sc.det(Rcap),'det')
        B1cap.append(bcap[0])
        B2cap.append(bcap[1])
        B3cap.append(bcap[2])
        #bcap = bcap + dotbcap*(data[i+1,0] - data[i,0])
        bcap = bcap + dotbcap*0.05
        
    
    Instants = np.array(Instants)/datarate
    B1cap = np.array(B1cap)*180/pi
    B2cap = np.array(B2cap)*180/pi
    B3cap = np.array(B3cap)*180/pi
    
    Theta = np.array(Theta)*180/pi
    Psi = np.array(Psi)*180/pi
    Phi = np.array(Phi)*180/pi
    
    print('alpha')
    plt.figure(1)
    plt.plot(Instants,Theta)
    fig1 = plt.figure(1)
    fig1.suptitle(r'$\theta$ vs time')
    plt.ylabel(r'$\theta (^{o})$')
    plt.xlabel('time (s)')
    plt.savefig('theta.png')
    
    print('psi')
    plt.figure(2)
    plt.plot(Instants,Psi)
    fig2 = plt.figure(2)
    fig2.suptitle(r'$\psi$ vs time')
    plt.ylabel(r'$\psi (^{o})$')
    plt.xlabel('time (s)')
    plt.savefig('psi.png')
    
    
    print('phi')
    plt.figure(3)
    plt.plot(Instants,Phi)
    fig3 = plt.figure(3)
    fig3.suptitle(r'$\phi$ vs time')
    plt.ylabel(r'$\phi (^{o})$')
    plt.xlabel('time (s)')
    plt.savefig('phi.png')
    
    print('b1cap')
    plt.figure(4)
    plt.plot(Instants,B1cap)
    fig4 = plt.figure(4)
    fig4.suptitle('$\hat{b}_1$ vs time')
    plt.xlabel('time (s)')
    plt.ylabel(r'$\hat{b}_1({^o/s})$')
    plt.savefig('b1.png')
    
    print('b2cap')
    plt.figure(5)
    plt.plot(Instants,B2cap)
    fig5 = plt.figure(5)
    fig5.suptitle('$\hat{b}_2$ vs time')
    plt.xlabel('time (s)')
    plt.ylabel(r'$\hat{b}_2({^o/s})$')
    plt.savefig('b2.png')
    
    
    print('b3cap')
    plt.figure(6)
    plt.plot(Instants,B3cap)
    fig6 = plt.figure(6)
    fig6.suptitle('$\hat{b}_3$ vs time')
    plt.xlabel('time (s)')
    plt.ylabel(r'$\hat{b}_3({^o/s})$')
    plt.savefig('b3.png')
    
    #plt.plot(Instants,Bcap)
    
if __name__ == '__main__':
    main()
