import brickpi3
import grovepi
import time
import numpy as np
from math import pi,sqrt,atan2,sin,cos
from IR_Functions import *

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
IR_cutoff = 15
move_b4_turn = 15
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
notes = 'This is a map of the maze'
origin = [0,2] #stored in grid points. [x,y]
n=8 #length of each side on the map
prev_encoder = 0

#initialize the map and the origin
map = np.zeros((n,n),np.int8)
map[n-1-origin[1]][origin[0]] = 5
currLoc = [[origin[0],origin[1]],[origin[0]*conversion,origin[1]*conversion]] #stored in [grid points, cm]

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

def rotateStatic(dir, angle=90):
    # r is right, l is left
    I = 0
    gyro = readGyro()
    gyro[0] = gyro[0]
    global rotTotal
    try:
        if(dir == 'r'):
            target = rotTotal + angle
            while gyro[0] < target:
                gyro = readGyro()
                I = rotateStaticSub(gyro, target, I)
        elif(dir=='l'):
            target = rotTotal - angle
            while gyro[0] > target:
                gyro = readGyro()
                I = rotateStaticSub(gyro, target, I)
        BP.set_motor_power(BP.PORT_B+BP.PORT_C,0)
        rotTotal = target
        gyro = readGyro()
        print('gyro abs: %3d' % gyro[0])
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
    try:
        while True:
            flag = 1 #for turning order. Fix.
            dist_right, dist_left, dist_front = readUltra()
            IR_sqrt = readIR()
            BP.set_motor_position(BP.PORT_A, 64)
            
            if(dist_left > turn_alert and dist_front > turn_alert - 10 and dist_right > turn_alert):
                BP.set_motor_power(BP.PORT_C+BP.PORT_B,0)
                map[currLoc[0][0]][currLoc[0][1]] = 4
                printMap()
                while(BP.get_motor_encoder(BP.PORT_A) < 99):
                    BP.set_motor_position(BP.PORT_A, 100)
                break
            elif(dist_left > turn_alert):
                moveDist(move_b4_turn)
                rotateStatic('l')
                dist_left = grovepi.ultrasonicRead(us_left)
                while dist_left > turn_alert:
                    dist_left = grovepi.ultrasonicRead(us_left)
                    BP.set_motor_power(BP.PORT_C+BP.PORT_B,spd_front)
                    print('moving until sees lefthand wall')
                BP.set_motor_power(BP.PORT_C+BP.PORT_B,0)
                flag = 0
            elif(!(dist_front < turn_alert-5) and !(IR_sqrt > IR_cutoff) and !((magnet_data['x'] > 87 and magnet_data['z'] > 0) or (magnet_data['x'] > 51 and magnet_data['z'] < 0))): #and if front is not blocked due to IR or magnet sensors
                flag = 0
                pass
            elif(dist_right > turn_alert):
                moveDist(move_b4_turn)
                rotateStatic('r')
                dist_right = grovepi.ultrasonicRead(us_right)
                while dist_right > turn_alert:
                    dist_right = grovepi.ultrasonicRead(us_right)
                    BP.set_motor_power(BP.PORT_C+BP.PORT_B,spd_front)
                    print('moving until sees righthand wall')
                BP.set_motor_power(BP.PORT_C+BP.PORT_B,0)
                flag = 0
            elif(flag): #Or if front is blocked due to IR or magnet sensors
                rotateStatic('l')
                rotateStatic('l')
                #if front is blocked due to sources, update the map
                #else just turn around
            
            prev_encoder = mapUpdate(prev_encoder)
            gyro = readGyro()
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

def moveDist(target_dist):
    zeroEncoder()
    curr_dist = 0
    try:
        while curr_dist < target_dist:
            e_front = target_dist - curr_dist
            gyro = readGyro()
            angleCorrection = angleCorrect(gyro[0])
            BP.set_motor_power(BP.PORT_C, spd_front - angleCorrection)
            BP.set_motor_power(BP.PORT_B, spd_front + angleCorrection)
            motor_encoder = BP.get_motor_encoder(BP.PORT_B)
            curr_dist = motor_encoder * pow(diam*2.54/2,2)*pi / 360
            print('distance remaining: %d' % e_front)
            time.sleep(dT)
    except KeyboardInterrupt:
        print('You pressed ctrl+c..')
        BP.reset_all()
    BP.set_motor_power(BP.PORT_B+BP.PORT_C, 0)

def turnToPt(currentX, currentY, targetX, targetY, currentAngle):
    angle = 180/pi*atan2((targetY - currentY), (targetX - currentX + 0.000001))
    if angle > currentAngle:
        rotateStatic('l', angle - currentAngle)
    else:
        rotateStatic('r', currentAngle - angle)
    return angle

def readUltra(fake=0):
    if(not fake):
        dist_right = grovepi.ultrasonicRead(us_right)
        if(dist_right > 50):
            dist_right = 50
        dist_left = grovepi.ultrasonicRead(us_left)
        if(dist_left > 50):
            dist_left = 50
        dist_front = BP.get_sensor(us_front)
        if(dist_front > 50):
            dist_front = 50
        print("front: %2d right: %2d left: %2d" % (dist_front, dist_right, dist_left))
    return(dist_right, dist_left, dist_front)

def readGyro(fake=0):
    if(not fake):
        gyro = BP.get_sensor(gyro_port)
    return gyro

def readIR(fake = 0):
    if(not fake):
        IR = IR_Read()
        IR_sqrt = IR[0]**0.5 + IR[1]**0.5
    return IR_sqrt

def moveGrid(dir,bin,curr,target): #for dir, 0 is x, 1 is y. for bin, 0 is no sensor, 1 is sensor
    I = 0
    zeroEncoder()
    while curr[dir] < target[dir]:
        if(bin):
            IR_data = IR_Read()
            AvgIR = (IR_data[0]+IR_data[1])/2
            print('One: %d Two: %d Avg: %d' % (IR_data[0], IR_data[1], AvgIR))

        e = target[dir] - curr[dir]
        P = front_kP * e
        I += front_kI * e * dT/2
        spd = P + I

        if(bin and (AvgIR > IR_cutoff)): #check magnet stuff #Check current direction?
            BP.set_motor_power(BP.PORT_B+BP.PORT_C, 0)
            gridAvoid(curr,e)
        else:
            gyro = readGyro()
            angleCorrection = angleCorrect(gyro[0])
            BP.set_motor_power(BP.PORT_C, spd - angleCorrection)
            BP.set_motor_power(BP.PORT_B, spd + angleCorrection)

        motor_encoder = BP.get_motor_encoder(BP.PORT_B)
        curr[dir] = motor_encoder * pow(diam*2.54/2,2)*pi / 360
        print('Distance Remaining: %.2f' % e)

        time.sleep(dT)

def nav2ptAvoidance(curr,target):
    try:
        moveGrid(0,1,curr,target)
        rotateStatic('l')
        moveGrid(1,1,curr,target)
        BP.reset_all()
    except KeyboardInterrupt:
        print('You pressed ctrl+c..')
        BP.reset_all()

def gridAvoid(curr,distRemain):
    rotateStatic('l')
    moveGrid(0,1,curr,[curr[0]-conversion/2,curr[1]])
    rotateStatic('r')
    moveGrid(0,1,curr,[curr[0],curr[1]+1+distRemain])

def zeroEncoder():
    global prev_encoder
    BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))
    prev_encoder = 0

def mapUpdate(prev_encoder):
    angleInRad = (rotTotal % 360)*pi/180
    direction = sin(angleInRad)
    encoder = BP.get_motor_encoder(BP.PORT_B)

    if(direction != 0):
        flipDir = direction
    else:
        flipDir = cos(angleInRad)

    currLoc[1][int(abs(direction))] += flipDir * (encoder - prev_encoder) * pow(diam*2.54/2,2)*pi / 360 / dT
    currLoc[0] = [int(currLoc[1][1] / conversion), n-1-int(currLoc[1][0] / conversion)]
    map[currLoc[0][0]][currLoc[0][1]] = 1
    return encoder

def printMap():
    f1 = open('team33_map.csv', 'w+')
    print('Team: 33\nMap: 0\nUnit Length: 40\nUnit: cm\nOrigin: (%d,%d)\nNotes: %s' %(origin[0],origin[1],notes),file=f1)
    print('\n'.join([','.join(['{:}'.format(item) for item in row]) for row in map]),file=f1)
    f1.close()
