import time
from module import *

time.sleep(2*dT)
try:
    while True:
        magnet_data = readMagnet()
        IR_val = sensors.dataIR
        flag = 1

        #dist_front > turn_alert - turn_front_offset
        if(IR_val < IR_cutoff):
        #if(abs(magnet_data['z'] + 50) < 70):
            flag = 0
            pass
        elif(flag): #Or if front is blocked due to IR or magnet sensors
            rotateStatic('l')
            rotateStatic('l')
        gyro = sensors.dataGyro
        angleCorrection = angleCorrect(gyro[0])
        BP.set_motor_power(BP.PORT_C, spd_front - angleCorrection)
        BP.set_motor_power(BP.PORT_B, spd_front + angleCorrection)
        
except KeyboardInterrupt:
    BP.set_motor_power(BP.PORT_B+BP.PORT_C,0)
    print('You pressed ctrl+c..')
    BP.reset_all()
finally:
    sensors.stop()
