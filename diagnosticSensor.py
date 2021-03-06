import time
from module import BP, dT, sensors, readMagnet

try:
    time.sleep(2*dT)
    while True:
        dist_right = sensors.dataUltra[0]
        dist_left = sensors.dataUltra[1]
        dist_front = sensors.dataUltra[2]
        IR_sqrt = sensors.dataIR
        magnet_data = readMagnet()
        gyro = sensors.dataGyro

        print("front: %2d right: %2d left: %2d" % (dist_front, dist_right, dist_left))
        print('gyro abs: %d' % (gyro[0]))
        print('IR Sqrt: %d' % IR_sqrt)
        print(magnet_data)
        time.sleep(dT)
except KeyboardInterrupt:
    print('You pressed ctrl+c..')
    BP.reset_all()
finally:
    sensors.stop()
