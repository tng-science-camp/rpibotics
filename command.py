#!/usr/bin/env python3
########## DO NOT CHANGE ##########
import rover
import time
  
#create instance
myRover=rover.MARSROVER() 

###################################

#Enter your code below
#Use "#" to comment out the code

#Use "#" to comment out the code
#go_forward(units, check_ir) check_ir default is 1, 0 to disable
#myRover.turn_right(6)
#myRover.go_backward(7)
myRover.arm_lance()
myRover.go_forward(7,0)
#myRover.turn_left(10)
#myRover.turn_right(10)
#myRover.capture_image()
#myRover.measure_humidity() 
#myRover.measure_temperature() 

#myRover.turn_left(5)
#myRover.turn_right(5)
#myRover.measure_temperature() 
#myRover.go_backward(2)

#take photo
#myRover.capture_image()

#turn using encoder
#myRover.turn_left_using_speed_sensor(20)
#myRover.turn_right_using_speed_sensor(20)

#Use this function to control iso and shutter speed (in microsecond, max 6s)
#myRover.capture_manual_image(100,1000000)

########## DO NOT CHANGE BELOW ##########
time.sleep(0.5)
#always disarm lance at the end of the program
myRover.disarm_lance()
#clear gpio ports
myRover.clear()



