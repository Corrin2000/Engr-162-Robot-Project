'''
1 = path gears took
0 = not part of path
5 = origin (starting point of GEARS)
2 = heat source
3 = magnetic source
4 = exit point

(0,0) is lower left corner
'''

notes = 'This maps hazards'
f1 = open('team33_hazards.csv', 'w+')
print('Team: 33\nMap: 0\nNotes: %s\n' %(notes),file=f1)
print('Resource Type, Parameter of Interest, Parameter Value, \
Resource X Coordinate, Resource Y Coordinaten\n\
      ', file=f1)

f1.close()