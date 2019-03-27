import grovepi

# Set up function
def IR_setup():
        sensor1= 14		# Pin 14 is A0 Port.
        sensor2 = 15		# Pin 15 is A0 Port.
        grovepi.pinMode(sensor1,"INPUT")
        grovepi.pinMode(sensor2,"INPUT")

# Output function
def IR_PrintValues():
        try:
                sensor1= 14		# Pin 14 is A0 Port.
                sensor2 = 15		# Pin 15 is A0 Port.               
                sensor1_value = grovepi.analogRead(sensor1)
                sensor2_value = grovepi.analogRead(sensor2)
                
                print ("One = " + str(sensor1_value) + "\tTwo = " + str(sensor2_value))
                #time.sleep(.1) # Commenting out for now

        except IOError:
                print ("Error")

#Read Function		
def IR_Read():
        try:
                sensor1= 14		# Pin 14 is A0 Port.
                sensor2 = 15		# Pin 15 is A0 Port.                
                sensor1_value = grovepi.analogRead(sensor1)
                sensor2_value = grovepi.analogRead(sensor2)
                
                return [sensor1_value, sensor2_value]

        except IOError:
                print ("Error")
