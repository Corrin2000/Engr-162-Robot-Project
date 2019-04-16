import time
from module import BP, dT, sensors

try:
    while True:
        [dist_right, dist_left, dist_front] = sensors.dataUltra
        IR_sqrt = sensors.dataIR
        magnet_data = sensors.dataMag
        gyro = sensors.dataGyro

        print("front: %2d right: %2d left: %2d" % (dist_front, dist_right, dist_left))
        print('gyro abs: %d' % (gyro[0]))
        print('IR Sqrt: %d' % IR_sqrt)
        print(magnet_data + '\n')
        time.sleep(dT)
except KeyboardInterrupt:
    print('You pressed ctrl+c..')
    BP.reset_all()