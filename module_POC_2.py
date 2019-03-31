import brickpi3
import grovepi
import time
from math import pi,sqrt,atan2
from IR_Functions import *

'''
Notes:
Do everything in the order right then left. Keep it standardized.
Port C is right, port B is left.
Assumes the robot starts at (0,0) pointing in the +x direction
Distances are in cm unless otherwise stated
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
IR_cutoff = 75
move_b4_turn = 20 #24

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
    gyro = BP.get_sensor(gyro_port)
    gyro[0] = gyro[0]
    global rotTotal
    try:
        if(dir == 'r'):
            target = rotTotal + angle
            while gyro[0] < target:
                gyro = BP.get_sensor(gyro_port)
                I = rotateStaticSub(gyro, target, I)
        elif(dir=='l'):
            target = rotTotal - angle
            while gyro[0] > target:
                gyro = BP.get_sensor(gyro_port)
                I = rotateStaticSub(gyro, target, I)
        BP.set_motor_power(BP.PORT_B+BP.PORT_C,0)
        rotTotal = target
        gyro = BP.get_sensor(gyro_port)
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
    try:
        while True:
            flag = 1 #for turning order. Fix.
            dist_right, dist_left, dist_front = readUltra()

            if(dist_left > turn_alert):
                moveDist(move_b4_turn)
                rotateStatic('l')
                dist_left = grovepi.ultrasonicRead(us_left)
                while dist_left > turn_alert:
                    dist_left = grovepi.ultrasonicRead(us_left)
                    BP.set_motor_power(BP.PORT_C+BP.PORT_B,spd_front)
                    print('moving until sees lefthand wall')
                BP.set_motor_power(BP.PORT_C+BP.PORT_B,0)
                flag = 0
            elif(dist_front > turn_alert-5): #and if front is not blocked due to IR or magnet sensors
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
            
            gyro = BP.get_sensor(gyro_port)
            rightSideCorrection, leftSideCorrection = sideCorrect(dist_right, dist_left)
            angleCorrection = angleCorrect(gyro[0])
            print('speed: %d R_correct: %d L_correct: %d' % (spd_front, rightSideCorrection - angleCorrection, leftSideCorrection + angleCorrection))
            BP.set_motor_power(BP.PORT_C, spd_front + rightSideCorrection - angleCorrection)
            BP.set_motor_power(BP.PORT_B, spd_front + leftSideCorrection + angleCorrection)

            time.sleep(dT)
    except KeyboardInterrupt:
        print('You pressed ctrl+c..')
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
    BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))
    curr_dist = 0
    try:
        while curr_dist < target_dist:
            e_front = target_dist - curr_dist
            gyro = BP.get_sensor(gyro_port)
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

def readUltra():
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

def moveGrid(dir,bin,curr,target): #for dir, 0 is x, 1 is y. for bin, 0 is no sensor, 1 is sensor
    I = 0
    BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))
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
            gyro = BP.get_sensor(gyro_port)
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

def dropCargo():
    try:
        while True:
            BP.set_motor_position(BP.PORT_A,-73)
    except KeyboardInterrupt:
        print('You pressed ctrl+c..')
        while(BP.get_motor_encoder(BP.PORT_A) != -4):
            BP.set_motor_position(BP.PORT_A,-4)
        BP.reset_all()
