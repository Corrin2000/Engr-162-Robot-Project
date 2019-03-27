import brickpi3
import grovepi
import time
from math import pi
from module import *

#x,y,angle
curr = [0,0]
target = [[1,1],[1,0],[2,1],[0,2]]

def moveGridI(dir,curr,target,i): #for dir, 0 is x, 1 is y. for bin, 0 is no sensor, 1 is sensor
    I = 0
    zeroEncoder()
    target[i][dir] *= conversion
    dist_total = target[i][dir] - curr[dir]
    dist_travelled = 0
    e = dist_total - dist_travelled
    while(abs(dist_travelled) < abs(dist_total)):
        e = dist_total - dist_travelled
        P = front_kP * e
        I += front_kI * e * dT/2
        spd = P + I

        gyro = readGyro
        angleCorrection = angleCorrect(gyro[0])
        BP.set_motor_power(BP.PORT_C, spd - angleCorrection)
        BP.set_motor_power(BP.PORT_B, spd + angleCorrection)
        dist_travelled = BP.get_motor_encoder(BP.PORT_B) * pow(diam*2.54/2,2)*pi / 360
        print('Distance Remaining: %.2f' % e)
        time.sleep(dT)
    curr[dir] = target[i][dir]

try:
    for i in range(0,4):
        print('Current = [%d,%d]' % (curr[0],curr[1]))
        print('Destination = [%d,%d]' % (target[i][0],target[i][1]))
        moveGridI(0,curr,target,i)
        rotate('l')
        BP.set_motor_power(BP.PORT_B+BP.PORT_C,0)
        print('Current = [%d,%d]' % (curr[0],curr[1]))
        print('Destination = [%d,%d]' % (target[i][0],target[i][1]))
        moveGridI(1,curr,target,i)
        rotate('r')
        BP.set_motor_power(BP.PORT_B+BP.PORT_C,0)
        print('zzzzzzzzzzzzz....')
        time.sleep(3)
    BP.reset_all()
except KeyboardInterrupt:
    print('You pressed ctrl+c..')
    BP.reset_all()

