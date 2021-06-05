import serial
import time

tT1 = 0
tT2 = 0
for i in range(100):
	t1 = time.time()
	
	ser1=serial.Serial('/dev/ttyACM0',115200)
	t2 = time.time()
	tT1 = tT1 + (t2 - t1)
	ser2=serial.Serial('/dev/ttyACM1',115200)
	tT2 = tT2 + (time.time()- t2)
	s1 = ser1.readline()
	print(s1)
	

	#print(time.time() - t1)
	s2 = ser2.readline()
	print(s2)
	time.sleep(0.2)
	
	ser1.close()
	ser2.close()
print(tT1/100)
print(tT2/100)
print((tT1+tT2)/100)
