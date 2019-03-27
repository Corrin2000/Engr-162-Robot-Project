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

rotation = 0
dT = 0.1

try:
    while True:
        gyro_data = mpu9250.readGyro()
        accel_data = mpu9250.readAccel()
        print(gyro_data)
        rotation += gyro_data[0] * dT
        print(rotation)
        time.sleep(dT)
except KeyboardInterrupt:
    print("you pressed control c")
