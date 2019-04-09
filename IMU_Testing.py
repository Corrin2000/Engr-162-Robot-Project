from MPU9250 import MPU9250
import numpy as np
import sys
import smbus
import time

from IMUFilters import AvgCali
from IMUFilters import genWindow
from IMUFilters import WindowFilterDyn
from IMUFilters import KalmanFilter
from IMUFilters import FindSTD
from IMUFilters import InvGaussFilter

import grovepi

bus = smbus.SMBus(1)

mpu9250 = MPU9250()

x = 1

width=10
depth=10
dly=0.01
adv = True

biases=AvgCali(mpu9250,depth,dly)
state=[[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[0,0,0,0,0,0,0,0,0]]#Estimated error (p) and measurement state (x) 
out=[0,0,0,0,0,0,0,0,0]
std=FindSTD(biases,mpu9250,dly)
pick = 1 #1 uses window filter, anything else uses Kalman
count = 3 #Number of standard deviations used for filtering

t0=time.time()

accelx=genWindow(width,0)#Can play with width to adjust system
accely=genWindow(width,0)
accelz=genWindow(width,0)

baseAccelData = mpu9250.readAccel()
baseAccelDataX = WindowFilterDyn(accelx,dly,InvGaussFilter(adv,baseAccelData['x'], biases[0],std[0],count))

#test case for right now
def distanceUsingAccelX(accelX, dT):
    accelX = abs(accelX)
    baseVel = accelX * dT
    baseDist = baseVel *dT * dT
    print(accelX)
    print(baseDist)

    
#distanceUsingAccelX(baseAccelData['x'])

try:
    while x != 0:
        gyro_data = mpu9250.readGyro()
        accel_data = mpu9250.readAccel()
        magnet_data = mpu9250.readMagnet()

        accelx=WindowFilterDyn(accelx,dly,InvGaussFilter(adv,accel_data['x'], biases[0],std[0],count))
        accely=WindowFilterDyn(accely,dly,InvGaussFilter(adv,accel_data['y'], biases[1],std[1],count))
        accelz=WindowFilterDyn(accelz,dly,InvGaussFilter(adv,accel_data['z'], biases[2],std[2],count))

        magX=WindowFilterDyn(accelx,dly,InvGaussFilter(adv,accel_data['x'], biases[0],std[0],count))
        magY=WindowFilterDyn(accely,dly,InvGaussFilter(adv,accel_data['y'], biases[1],std[1],count))
        magZ=WindowFilterDyn(accelz,dly,InvGaussFilter(adv,accel_data['z'], biases[2],std[2],count))
        
        #print(accel_data)

        #distanceUsingAccelX(accel_data[0])
        
        #print('in loop accel: ')
        #print(accel_data['x'])

        print(magnet_data)
        
        time.sleep(0.05)
            

        
        
except KeyboardInterrupt:
    print("you pressed control c")


