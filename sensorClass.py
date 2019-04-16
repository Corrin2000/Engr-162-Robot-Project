import brickpi3
import grovepi
import IMU_Setup as mag
from threading import Timer
from module import BP, dT, accelx, accely, accelz, us_front, us_left, us_right, gyro_port
from IR_Functions import IR_Read
from IMUFilters import genWindow, WindowFilterDyn, InvGaussFilter

class sensors(object):
    def __init__(self, fake):
        self._timer     = None
        self.is_running = False
        self.dataUltra  = 0
        self.dataGyro   = 0
        self.dataIR     = 0
        self.dataMag    = 0
        self.fake       = fake
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.readGyro(self.fake)
        self.readUltra(self.fake)
        self.readIR(self.fake)
        self.readMagnet(self.fake)

    def start(self):
        if not self.is_running:
            self._timer = Timer(dT, self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        self._timer.cancel()
        self.is_running = False
    
    def readUltra(self, fake = 0):
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
        self.dataUltra = [dist_right, dist_left, dist_front]
    
    def readGyro(self, fake = 0):
        if(not fake):
            gyro = BP.get_sensor(gyro_port)
            print('gyro abs: %d' % (gyro[0]))
        self.dataGyro = gyro

    def readIR(self, fake = 0):
        if(not fake):
            IR = IR_Read()
            IR_sqrt = IR[0]**0.5 + IR[1]**0.5
            print('IR Sqrt: %d' % IR_sqrt)
        self.dataIR = IR_sqrt

    def readMagnet(self, fake = 0):
        if(not fake):
            accel_data = mag.mpu9250.readAccel()
            magnet_data = mag.mpu9250.readMagnet()
            magX=WindowFilterDyn(accelx,mag.dly,InvGaussFilter(mag.adv,accel_data['x'], mag.biases[0],mag.std[0],mag.count))
            magY=WindowFilterDyn(accely,mag.dly,InvGaussFilter(mag.adv,accel_data['y'], mag.biases[1],mag.std[1],mag.count))
            magZ=WindowFilterDyn(accelz,mag.dly,InvGaussFilter(mag.adv,accel_data['z'], mag.biases[2],mag.std[2],mag.count))
            print(magnet_data)
        self.dataMag = magnet_data