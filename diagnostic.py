import time
from module import readUltra, readGyro, readIR, readMagnet, BP, dT

try:
    while True:
        dist_right, dist_left, dist_front = readUltra()
        IR_sqrt = readIR()
        magnet_data = readMagnet()
        gyro = readGyro()

        print("front: %2d right: %2d left: %2d" % (dist_front, dist_right, dist_left))
        print('gyro abs: %d' % (gyro[0]))
        print('IR Sqrt: %d' % IR_sqrt)
        print(magnet_data + '\n')
        time.sleep(dT)
except KeyboardInterrupt:
    print('You pressed ctrl+c..')
    BP.reset_all()