import time
import brickpi3
import grovepi

BP = brickpi3.BrickPi3()
BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))
BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C))

#sensor setup
us_front = BP.PORT_3
BP.set_sensor_type(us_front, BP.SENSOR_TYPE.EV3_ULTRASONIC_CM)
us_left = 5
us_right = 6

# initialization
KP = 1.0 # proportional control gain
KI = 2.0 # integral control gain
KD = 0.0 # derivative control gain
dT = 0.02 # time step

target = 20

P = 0
I = 0
D = 0
e_prev = 0
e = 1

#control loop
try:
    while True:
        #PID control
        dist_front = BP.get_sensor(us_front)
        e = target - dist_front
        print("%d cm from the wall " % dist_front)

        P = KP * e
        I += KI * e * dT/2
        D = KD * (e - e_prev)/ dT

        power_in = P + I + D
        e_prev = e
        BP.set_motor_power(BP.PORT_B+BP.PORT_C, -power_in)
        
        #left/right sensors
        dist_left = grovepi.ultrasonicRead(us_left)
        if(dist_left > 50):
            dist_left = 50
        dist_right = grovepi.ultrasonicRead(us_right)
        if(dist_right > 50):
            dist_right = 50
        print("left: %d right: %d" % (dist_left, dist_right))

        time.sleep(dT)

except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    print('You pressed ctrl+c..')
    BP.set_motor_power(BP.PORT_B+BP.PORT_C, 0)
    BP.reset_all()  
