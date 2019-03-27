import brickpi3
import grovepi
import time
import math
import module as m

#x,y,angle
curr = [0,0,0]
target = [[3,1],[4,3],[0,2],[1,4]]
time.sleep(2)
conversion = 40 #cm per grid square
for i in range(0,4):
    targetX = target[i][0] * conversion
    targetY = target[i][1] * conversion
    curr[2] = m.turnToPt(curr[0],curr[1],targetX,targetY,curr[2])
    dist = math.sqrt(pow(targetX - curr[0], 2) + pow(targetY - curr[1], 2))
    m.moveDist(dist)
    curr[0] = targetX
    curr[1] = targetY
    time.sleep(2)
m.BP.reset_all()
