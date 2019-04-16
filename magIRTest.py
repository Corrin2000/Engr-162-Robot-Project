from module import *

try:
    while True:
        magnet_data = sensors.dataMag
        IR_sqrt = sensors.dataIR
        flag = 1

        #dist_front > turn_alert - turn_front_offset
        #if(IR_sqrt < IR_cutoff):
        if(abs(magnet_data['z'] + 50) < 70):
            flag = 0
            pass
        elif(flag): #Or if front is blocked due to IR or magnet sensors
            rotateStatic('l')
            rotateStatic('l')
        BP.set_motor_power(BP.PORT_B+BP.PORT_C,20)
        
except KeyboardInterrupt:
    BP.set_motor_power(BP.PORT_B+BP.PORT_C,0)
    print('You pressed ctrl+c..')
    BP.reset_all()