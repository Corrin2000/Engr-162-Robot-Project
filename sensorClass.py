import brickpi3
import grovepi
import IMU_Setup as mag
from threading import Timer
from module import BP, dT, accelx, accely, accelz, us_front, us_left, us_right, gyro_port
from IR_Functions import IR_Read
from IMUFilters import genWindow, WindowFilterDyn, InvGaussFilter
from numpy import zeros

class sensorClass(object):
    def __init__(self, fake=0):
        self._timer     = None
        self.is_running = False

        #data storage
        self.dataUltra  = [0,0,0]
        self.dataGyro   = 0
        self.dataIR     = 0
        self.listIR     = zeros(8)
        self.dataMag    = 0
        self.index      = 0
        
        self.fake       = fake
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.readGyro(self.fake)
        self.readUltra(self.fake)
        self.readIR(self.fake)

    def start(self):
        if not self.is_running:
            self._timer = Timer(2*dT, self._run)
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
        self.dataUltra = [dist_right, dist_left, dist_front]
    
    def readGyro(self, fake = 0):
        if(not fake):
            gyro = BP.get_sensor(gyro_port)
        self.dataGyro = gyro

    def readIR(self, fake = 0):
        if(not fake):
            IR = IR_Read()
            if(IR[0] > 500):
                IR[0] = 0
            if(IR[1] > 500):
                IR[1] = 0
            IR_val = IR[0] + IR[1]
            print('one: %3d two: %d'%(IR[0],IR[1]))
        self.listIR[self.index] = IR_val
        print('IR val: %3d IR runAvg: %d' % (IR_val, self.dataIR))
        self.index += 1
        self.index = self.index % len(self.listIR)
        self.dataIR = sum(self.listIR)/len(self.listIR)