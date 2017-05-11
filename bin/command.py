#!/usr/bin/env python3
from rover import rover_sj17
import time

myRover = rover_sj17.RoverSJ17()

myRover.motor1.turn_clockwise()
myRover.motor2.turn_counter_clockwise()
time.sleep(3)
myRover.motor1.stop()
myRover.motor2.stop()

myRover.motor1.turn_counter_clockwise()
time.sleep(2)
myRover.motor1.stop()

# Enter your code below
# Use "#" to comment out the code


