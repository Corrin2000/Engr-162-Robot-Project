import brickpi3
import grovepi
import time
import numpy as np
import IMU_Setup as mag
from math import pi, sqrt, atan2, sin, cos
from IR_Functions import IR_Read
from IMUFilters import genWindow, WindowFilterDyn, InvGaussFilter

'''
Notes:
Do everything in the order right then left. Keep it standardized.
Port C is right, port B is left.
Assumes the robot starts at (0,0) pointing in the +x direction
Distances are in cm unless otherwise stated

Starting instructions: Start the ramp at fully upright.
'''

BP = brickpi3.BrickPi3()
BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))
BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C))
BP.reset_all()

#constant initialization
dT = 0.05
diam = 2.5 #of wheel, in inches
spd_front = 30
rotTotal = 0
conversion = 40 #40 cm per grid square

#Borders and cutoffs
front_border = 15
side_border = 10
turn_alert = 25
turn_front_offset = 8
IR_cutoff = 120
move_b4_turn = 12
'''
Magnet is suposed to sense at 7 in away. Then should pull a uturn
'''

#PI(d) K Values
rotate_kP = 0.3
rotate_kI = 1.0
front_kP = 0.5
front_kI = 1.0
side_kP = 3.0
side_kI = 3.0
angle_kP = 1.0
angle_kI = 1.5
kRot = 4.8 #paper: 4.8   URSC table = 4.8

#Mapping Init
notes = 'This is a map of the maze.'
origin = [0,0] #stored in grid points. [x,y]
map_size = 8 #length of each side on the map
prev_encoder = 0

#initialize the map and the origin
map = np.zeros((map_size,map_size),np.int8)
currLoc = [[origin[0],origin[1]],[origin[0]*conversion,origin[1]*conversion]] #stored in [grid points, cm]

#IMU Setup
accelx=genWindow(mag.width,0)
accely=genWindow(mag.width,0)
accelz=genWindow(mag.width,0)

#sensor setup
gyro_port = BP.PORT_4
BP.set_sensor_type(gyro_port, BP.SENSOR_TYPE.EV3_GYRO_ABS_DPS)
us_front = BP.PORT_3
BP.set_sensor_type(us_front, BP.SENSOR_TYPE.EV3_ULTRASONIC_CM)
us_right = 6
us_left = 5

#sensor warmup
flag = 1
while flag:
    try:
        dist_front = BP.get_sensor(us_front)
        gyro = BP.get_sensor(gyro_port)
        flag = 0
    except brickpi3.SensorError as error:
        print('Warming up...')
        flag = 1
    time.sleep(dT)

import sensorClass
sensors = sensorClass.sensorClass()

def rotateStatic(dir, angle=90):
    # r is right, l is left
    I = 0
    gyro = sensors.dataGyro
    global rotTotal
    try:
        if(dir == 'r'):
            target = rotTotal + angle
            while gyro[0] < target:
                gyro = sensors.dataGyro
                I = rotateStaticSub(gyro, target, I)
        elif(dir=='l'):
            target = rotTotal - angle
            while gyro[0] > target:
                gyro = sensors.dataGyro
                I = rotateStaticSub(gyro, target, I)
        BP.set_motor_power(BP.PORT_B+BP.PORT_C,0)
        rotTotal = target
        gyro = sensors.dataGyro
        print('gyro abs: %3d' % gyro[0])
        zeroEncoder()
    except KeyboardInterrupt:
        print('You pressed ctrl+c..')
        BP.reset_all()

def rotateStaticSub(gyro, target, I):
    e = target - gyro[0]
    P =  rotate_kP * e
    I += rotate_kI * e * dT/2
    turnSpd = P + I
    print('gyro abs: %3d target: %3d e: %3d' % (gyro[0], target, e))
    BP.set_motor_power(BP.PORT_B, turnSpd)
    BP.set_motor_power(BP.PORT_C, -turnSpd)
    time.sleep(dT)
    return I

def navMaze():
    global prev_encoder
    time.sleep(2*dT)
    try:
        while True:
            flag = 1
            dist_right = sensors.dataUltra[0]
            dist_left = sensors.dataUltra[1]
            dist_front = sensors.dataUltra[2]
            IR_val = sensors.dataIR
            magnet_data = readMagnet()
            BP.set_motor_position(BP.PORT_A, 0)
            
            if(dist_left > turn_alert and dist_front > turn_alert and dist_right > turn_alert):
                BP.set_motor_power(BP.PORT_C+BP.PORT_B,0)
                map[map_size - 1 - origin[1]][origin[0]] = 5
                map[map_size - 1 - currLoc[0][1]][currLoc[0][0]] = 4
                printMap()
                while(BP.get_motor_encoder(BP.PORT_A) < 90):
                    BP.set_motor_power(BP.PORT_A,20)
                BP.set_motor_power(BP.PORT_A,0)
                break
            elif(dist_left > turn_alert):
                moveDist(move_b4_turn)
                rotateStatic('l')
                dist_left = sensors.dataUltra[1]
                zeroEncoder()
                while dist_left > turn_alert:
                    dist_left = sensors.dataUltra[1]
                    BP.set_motor_power(BP.PORT_C+BP.PORT_B,spd_front)
                    print('moving until sees lefthand wall')
                    printMap()
                    prev_encoder = mapUpdate(prev_encoder)
                BP.set_motor_power(BP.PORT_C+BP.PORT_B,0)
                flag = 0
            #elif(dist_front > turn_alert-turn_front_offset):
            elif(dist_front > turn_alert-turn_front_offset and IR_val < IR_cutoff and abs(magnet_data['z'] + 50) < 70):
                flag = 0
                pass
            elif(dist_right > turn_alert):
                moveDist(move_b4_turn)
                rotateStatic('r')
                dist_right = sensors.dataUltra[0]
                zeroEncoder()
                while dist_right > turn_alert:
                    dist_right = sensors.dataUltra[0]
                    BP.set_motor_power(BP.PORT_C+BP.PORT_B,spd_front)
                    print('moving until sees righthand wall')
                    printMap()
                    prev_encoder = mapUpdate(prev_encoder)
                BP.set_motor_power(BP.PORT_C+BP.PORT_B,0)
                flag = 0
            elif(flag):
                if(IR_val < IR_cutoff):
                    angleInRad = (rotTotal % 360)*pi/180
                    map[map_size - 1 - currLoc[0][1] - int(sin(angleInRad))][currLoc[0][0] + int(cos(angleInRad))] = 2
                elif(abs(magnet_data['z'] + 50) < 70):
                    angleInRad = (rotTotal % 360)*pi/180
                    map[map_size - 1 - currLoc[0][1] - int(sin(angleInRad))][currLoc[0][0] + int(cos(angleInRad))] = 3
                rotateStatic('l')
                rotateStatic('l')
            
            printMap()
            prev_encoder = mapUpdate(prev_encoder)
            gyro = sensors.dataGyro
            rightSideCorrection, leftSideCorrection = sideCorrect(dist_right, dist_left)
            angleCorrection = angleCorrect(gyro[0])
            print('speed: %d R_correct: %d L_correct: %d' % (spd_front, rightSideCorrection - angleCorrection, leftSideCorrection + angleCorrection))
            BP.set_motor_power(BP.PORT_C, spd_front + rightSideCorrection - angleCorrection)
            BP.set_motor_power(BP.PORT_B, spd_front + leftSideCorrection + angleCorrection)

            time.sleep(dT)
    except KeyboardInterrupt:
        print('You pressed ctrl+c..')
        printMap()
        BP.reset_all()
    finally:
        sensors.stop()

def sideCorrect(dist_right, dist_left):
    I_right = 0
    I_left = 0
    try:
        if(dist_right < side_border):
            P_right = side_kP * (side_border-dist_right)
            I_right += side_kI * (side_border-dist_right) * dT/2
            rightCorrection = P_right + I_right
        else:
            rightCorrection = 0
        if(dist_left < side_border):
            I_left += side_kI * (side_border-dist_left) * dT/2
            P_left = side_kP * (side_border-dist_left)
            leftCorrection = P_left + I_left
        else:
            leftCorrection = 0

        return rightCorrection, leftCorrection
    except KeyboardInterrupt:
        print('You pressed ctrl+c..')
        BP.reset_all()

def angleCorrect(gyroAbs):
    I = 0
    try:
        e = rotTotal-gyroAbs
        P = angle_kP * e
        I += angle_kI * e * dT/2
        angleCorrection = P + I
        return angleCorrection
    except KeyboardInterrupt:
        print('You pressed ctrl+c..')
        BP.reset_all()

def readMagnet(fake = 0):
    if(not fake):
        accel_data = mag.mpu9250.readAccel()
        magnet_data = mag.mpu9250.readMagnet()
        magX=WindowFilterDyn(accelx,mag.dly,InvGaussFilter(mag.adv,accel_data['x'], mag.biases[0],mag.std[0],mag.count))
        magY=WindowFilterDyn(accely,mag.dly,InvGaussFilter(mag.adv,accel_data['y'], mag.biases[1],mag.std[1],mag.count))
        magZ=WindowFilterDyn(accelz,mag.dly,InvGaussFilter(mag.adv,accel_data['z'], mag.biases[2],mag.std[2],mag.count))
        #print(magnet_data)
    return magnet_data

def moveDist(target_dist):
    zeroEncoder()
    global prev_encoder
    curr_dist = 0
    try:
        while curr_dist < target_dist:
            e_front = target_dist - curr_dist
            gyro = sensors.dataGyro
            angleCorrection = angleCorrect(gyro[0])
            BP.set_motor_power(BP.PORT_C, spd_front - angleCorrection)
            BP.set_motor_power(BP.PORT_B, spd_front + angleCorrection)
            motor_encoder = BP.get_motor_encoder(BP.PORT_B)
            curr_dist = motor_encoder * diam * 2.54 * pi / 360
            print('distance remaining: %d' % e_front)
            printMap()
            prev_encoder = mapUpdate(prev_encoder)
            time.sleep(dT)
    except KeyboardInterrupt:
        print('You pressed ctrl+c..')
        BP.reset_all()
    BP.set_motor_power(BP.PORT_B+BP.PORT_C, 0)

def zeroEncoder():
    global prev_encoder
    BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))
    BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C))
    prev_encoder = 0

def mapUpdate(prev_encoder):
    angleInRad = (rotTotal % 360)*pi/180
    direction = sin(angleInRad)
    encoder = BP.get_motor_encoder(BP.PORT_B)

    if(direction != 0):
        flipDir = -direction
    else:
        flipDir = cos(angleInRad)

    currLoc[1][int(abs(direction))] += flipDir * (encoder - prev_encoder) * diam * 2.54 * pi / 360
    currLoc[0] = [int((currLoc[1][0] + 20) / conversion), int((currLoc[1][1] + 20) / conversion)]
    map[map_size - 1 - currLoc[0][1]][currLoc[0][0]] = 1
    #print(currLoc)
    return encoder

def printMap():
    f1 = open('team33_map.csv', 'w+')
    print('Team: 33\nMap: 0\nUnit Length: 40\nUnit: cm\nOrigin: (%d,%d)\nNotes: %s' %(origin[0],origin[1],notes),file=f1)
    print('\n'.join([','.join(['{:}'.format(item) for item in row]) for row in map]),file=f1)
    f1.close()
