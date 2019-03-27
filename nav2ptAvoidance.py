import brickpi3
import grovepi
import time
from module import *
from IR_Functions import *

BP = brickpi3.BrickPi3()
curr = [0,0] #[x,y]
IR_data = [0,0]

#UPDATE FOR EACH RUN
target = [1,1] #[x,y]
target[0] *= conversion
target[1] *= conversion

nav2ptAvoidance(curr,target)