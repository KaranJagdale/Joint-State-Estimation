import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

g0 = 9.81
def main():
    df = pd.read_csv('/home/dell/Project/csv/datatemp.csv') 
    datapt = len(df['time'])
    print(datapt)
    #pltrng = 
    
    #print(a)
    Ax = df['ax']
    Ax = np.array(Ax)  
    #print(len(Ax))
    Axm = Ax[0::5]*g0
    
    Ay = df['ay']
    Ay = np.array(Ay)  
    #print(len(Ax))
    Aym = Ay[0::5]*g0
    
    Az = df['az']
    Az = np.array(Az)  
    #print(len(Ax))
    Azm = Az[0::5]*g0
    
    
    
    Mx = df['mx']
    #print(len(Mx))
    Mx = np.array(Mx)  
    #print(len(Ax))
    Mxm = Mx[0::5]
    
    My = df['my']
    #print(len(My))
    My = np.array(My)  
    #print(len(Ax))
    Mym = My[0::5]
    
    Mz = df['mz']
    #print(len(Mx))
    Mz = np.array(Mz)  
    #print(len(Ax))
    Mzm = Mz[0::5]
    

    Gx = df['gx']
    #print(len(Mx))
    Gx = np.array(Gx)  
    #print(len(Ax))
    Gxm = Gx[0::5]    
    
    Gy = df['gy']
    #print(len(Mx))
    Gy = np.array(Gy)  
    #print(len(Ax))
    Gym = Gy[0::5]   
    
    Gz = df['gz']
    #print(len(Mx))
    Gz = np.array(Gz)  
    #print(len(Ax))
    Gzm = Gz[0::5]   
    
    T = df['time']
    T = np.array(T)
    Tm = T[0::5]

    plt.figure(1)
    plt.plot(T,Ax)
    fig1 = plt.figure(1)
    fig1.suptitle('ax')
    plt.savefig('ax.png')
    plt.figure(2)
    plt.plot(T,Ay)
    fig2 = plt.figure(2)
    fig2.suptitle('ay')
    plt.savefig('ay.png')
    plt.figure(3)
    plt.plot(T,Az)
    fig3 = plt.figure(3)
    fig3.suptitle('az')
    plt.savefig('az.png')
    plt.figure(4)
    plt.plot(T,Gx) 
    fig4 = plt.figure(4)
    fig4.suptitle('gx')
    plt.savefig('gx.png')
    plt.figure(5)
    plt.plot(T,Gy)    
    fig5 = plt.figure(5)
    fig5.suptitle('gy')
    plt.savefig('gy.png')
    plt.figure(6)
    plt.plot(T,Gz)  
    fig6 = plt.figure(6)
    fig6.suptitle('gz')
    plt.savefig('gz.png')
    plt.figure(7)
    plt.plot(T,Mx)  
    fig7 = plt.figure(7)
    fig7.suptitle('mx')
    plt.savefig('mx.png')
    plt.figure(8)
    plt.plot(T,My) 
    fig8 = plt.figure(8)
    fig8.suptitle('my')
    plt.savefig('my.png')
    
    plt.figure(9)
    plt.plot(T,Mz)    
    fig9 = plt.figure(9)
    fig9.suptitle('mz')
    plt.savefig('mz.png')
    #ap[lying low pass filter to Mx
  
    
    
    
    
if __name__ == '__main__':
    main()
