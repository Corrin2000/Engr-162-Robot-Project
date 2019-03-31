import brickpi3
import grovepi
import time
from IR_Functions import IR_Read

BP = brickpi3.BrickPi3()
BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))
BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C))
BP.reset_all()

'''
try:
    while True:
        gyro = BP.get_sensor(gyro_port)
        angleCorrection = angleCorrect(gyro[0])
        print(angleCorrection)
        BP.set_motor_power(BP.PORT_C, spd_front - angleCorrection)
        BP.set_motor_power(BP.PORT_B, spd_front + angleCorrection)
        time.sleep(dT)
except KeyboardInterrupt:
    print('You pressed ctrl+c..')
    BP.reset_all()
'''

#0 at fully upright
#100 at down
#64 at holding
'''
try:
    while True:
        BP.set_motor_position(BP.PORT_A, 64)
        print(BP.get_motor_encoder(BP.PORT_A))
except KeyboardInterrupt:
    print('You pressed ctrl+c..')
    while(BP.get_motor_encoder(BP.PORT_A) < 99):
        BP.set_motor_position(BP.PORT_A, 100)
    BP.reset_all()
'''

try:
    while True:
        IR = IR_Read()
        IR_avg = (IR[0]**0.5 + IR[1]**0.5)
        IR_max = max(IR)
        print('1: %d 2: %d avg: %d' % (IR[0],IR[1],IR_avg))
        if((IR_avg > 70) or (IR_max > 100)):
            print('too close')
        else:
            print('good range')
        time.sleep(0.05)
except KeyboardInterrupt:
    print('You pressed ctrl+c..')
    BP.reset_all()
