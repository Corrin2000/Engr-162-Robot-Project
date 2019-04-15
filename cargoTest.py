from module import BP

try:
    while True:
        print(BP.get_motor_encoder(BP.PORT_A))
except KeyboardInterrupt:
    print('You pressed ctrl+c..')
    BP.reset_all()