import brickpi3
import grovepi
import time
from module import *

try:
    while True:
        gyro = BP.get_sensor(gyro_port)
        angleCorrection = angleCorrect(gyro[0])
        print(angleCorrection)
        BP.set_motor_power(BP.PORT_C, spd_front + angleCorrection)
        BP.set_motor_power(BP.PORT_B, spd_front - angleCorrection)
        time.sleep(dT)
except KeyboardInterrupt:
    print('You pressed ctrl+c..')
    BP.reset_all()
