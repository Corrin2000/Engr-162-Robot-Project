from MPU9250 import MPU9250
import numpy as np
import sys
import smbus
import time
import grovepi

from IMUFilters import AvgCali
from IMUFilters import genWindow
from IMUFilters import WindowFilterDyn
from IMUFilters import KalmanFilter
from IMUFilters import FindSTD
from IMUFilters import InvGaussFilter

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