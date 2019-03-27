# Program provides feedback control based on motor encoder readings
# Developed for in class activity
# ENGR 162, Spring 2018

import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers
import grovepi

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
us_port = 2

# initialization
# Tuning parameters
KP = 1.0 # proportional control gain
KI = 2.0 # integral control gain
KD = 0.0 # derivative control gain

dT = 0.02 # time step

target_dist = 20

current_dist = 0

P = 0
I = 0
D = 0
e_prev = 0
e = 1

# --------------------------------
# Hardware initialization
# --------------------------------
BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))
BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C))

# ---------------------------------------------------------
# Control loop -- run infinitely until a keyboard interrupt
# ---------------------------------------------------------
try:
    while True:
        current_dist = grovepi.ultrasonicRead(us_port)
        print('Ultrasonic is %d' % current_dist)

        e = target_dist - current_dist
        print("error is " + str(e))

        # set up P,I,D, terms for control inputs
        P = KP * e
        I += KI * e * dT/2
        D = KD * (e - e_prev)/ dT

        # control input for motor
        power_in = P + I + D
        BP.set_motor_power(BP.PORT_B+BP.PORT_C, -power_in)
        
        # save error for this step; needed for D
        e_prev = e

        time.sleep(dT)

# ---------------------------------------------------------------------
# If a problem occurse with the while or an interrupt from the keyboard
# ---------------------------------------------------------------------
except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    print('You pressed ctrl+c..')
    BP.set_motor_power(BP.PORT_B+BP.PORT_C, 0)
    BP.reset_all()  
