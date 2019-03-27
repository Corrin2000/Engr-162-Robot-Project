import numpy as np
from math import sin,pi,cos
from module import * #uncomment this later for the actual robot

'''
1 = path gears took
0 = not part of path
5 = origin (starting point of GEARS)
2 = heat source
3 = magnetic source
4 = exit point

(0,0) is lower left corner
'''

notes = 'This is a map of the maze'
origin = [0,0] #stored in grid points
n=8 #length of each side on the map
prev_encoder = 0

#initialize the map and the origin
map = np.zeros((n,n),np.int8)
map[n-1-origin[1]][origin[0]] = 5
currLoc = [[origin[0],origin[1]],[origin[0]*conversion,origin[1]*conversion]] #stored in [grid points, cm]

def mapUpdate(prev_encoder):
    angleInRad = (rotTotal % 360)*pi/180
    direction = sin(angleInRad)
    encoder = BP.get_motor_encoder(BP.PORT_B)

    if(direction != 0):
        flipDir = direction
    else:
        flipDir = cos(angleInRad)

    currLoc[1][abs(direction)] += flipDir * (encoder - prev_encoder) * pow(diam*2.54/2,2)*pi / 360 / dT
    currLoc[0] = [n-1-int(currLoc[1][0] % conversion), int(currLoc[1][1] % conversion)]
    map[currLoc[0][0]][currLoc[0][1]] = 1
    return encoder

def printMap():
    f1 = open('team33_map.csv', 'w+')
    print('Team: 33\nMap: 0\nUnit Length: 40\nUnit: cm\nOrigin: (%d,%d)\nNotes: %s' %(origin[0],origin[1],notes),file=f1)
    print('\n'.join([','.join(['{:}'.format(item) for item in row]) for row in map]),file=f1)
    f1.close()