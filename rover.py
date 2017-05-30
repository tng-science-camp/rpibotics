#!/usr/bin/env python3
# botdemyrover library
# functions to control L293 motor control and on-board camera
# www.botdemy.com

# Import
import Adafruit_DHT

import smbus, time, math, sys, string
import picamera
import RPi.GPIO as gpio

#### Constants ####

# Print actions without actually performing them
TEST_MODE=False

#PWM/duty cycle for each motor
PWM_FREQ = 24
LMOTORSPEED=80
RMOTORSPEED=78

#lance servo duty cycle determines position 2.1, 10.5
LANCE_OPEN =  2.1
#LANCE_OPEN =  2.1
LANCE_CLOSE = 10.5

# Two motors with two directions each = four GPIO pins
# kc: pwm gpio 12:13

GPIO_PIN_RIGHT_MOTOR_FORWARD=26
GPIO_PIN_RIGHT_MOTOR_BACKWARD=19
GPIO_PIN_RIGHT_PWM=13

GPIO_PIN_LEFT_MOTOR_FORWARD=20
GPIO_PIN_LEFT_MOTOR_BACKWARD=16
GPIO_PIN_LEFT_PWM=12

# Step duration in sec
STEP_DURATION=0.1

# paulse
PAUSE=0.5

#camera image location
IMG_LOC="/var/www/html/rover_img/"

#### One-time setup ####

#i2c address of hmc5883
rev = gpio.RPI_REVISION
if rev == 2 or rev == 3:
    bus = smbus.SMBus(1)
else:
    bus = smbus.SMBus(0)

#hmc5889 new?
HMC5883_ADDRESS = 0x1e
#HMC5883_ADDRESS = 0x0d
AM2302 = Adafruit_DHT.AM2302
GPIO_PIN_AM2302 = 24	   #gpio pin #24

#IR Obstacle Sensors
GPIO_PIN_IR_SENSOR_LEFT = 17  #gpio pin #17
GPIO_PIN_IR_SENSOR_RIGHT = 27  #gpio pin #27

#Optocoupler Speed Encoders
GPIO_PIN_SPEED_SENSOR_LEFT = 8  #gpio pin #8
GPIO_PIN_SPEED_SENSOR_RIGHT = 7 #gpio pin #7

#LANCE SERVO
#TODO: change pin
GPIO_PIN_LANCE_SERVO = 21 #gpio pin #21

# Set up the GPIO pins, referring to the constants
gpio.setwarnings(False)
gpio.setmode(gpio.BCM)

gpio.setup(GPIO_PIN_RIGHT_MOTOR_FORWARD, gpio.OUT)
gpio.setup(GPIO_PIN_RIGHT_MOTOR_BACKWARD, gpio.OUT)
gpio.setup(GPIO_PIN_RIGHT_PWM, gpio.OUT)
Rpwm = gpio.PWM(GPIO_PIN_RIGHT_PWM, PWM_FREQ)  # Initialize PWM on pwmPin 20Hz frequency

gpio.setup(GPIO_PIN_LEFT_MOTOR_FORWARD, gpio.OUT)
gpio.setup(GPIO_PIN_LEFT_MOTOR_BACKWARD, gpio.OUT)
gpio.setup(GPIO_PIN_LEFT_PWM, gpio.OUT)
Lpwm = gpio.PWM(GPIO_PIN_LEFT_PWM, PWM_FREQ)  # Initialize PWM on pwmPin 20Hz frequency

gpio.setup(GPIO_PIN_IR_SENSOR_LEFT, gpio.IN)
gpio.setup(GPIO_PIN_IR_SENSOR_RIGHT, gpio.IN)

gpio.setup(GPIO_PIN_SPEED_SENSOR_LEFT, gpio.IN)
gpio.setup(GPIO_PIN_SPEED_SENSOR_RIGHT, gpio.IN)

#gpio.setup(GPIO_PIN_LED, gpio.OUT)

gpio.setup(GPIO_PIN_LANCE_SERVO, gpio.OUT)
lance_servo_pwm = gpio.PWM(GPIO_PIN_LANCE_SERVO, 50)  # Initialize Servo PWM  50


#### Objects ####

#class BotdemyRover(object):
class MARSROVER(object):
    # There should only be one instance
    _instances=[]

    # Initialise the object
    def __init__(self,testMode=TEST_MODE):
        if ( len(self._instances)>1 ):
            print ("ERROR: You can't have more than one Botdemy Rover instance.")
            exit(1)
        self._instances.append(self)
        self.testMode=testMode
        if ( self.testMode ):
            print ("+Running in Test Mode")

    def read_word_2c(self,adr):

        high = bus.read_byte_data(HMC5883_ADDRESS, adr)
        low = bus.read_byte_data(HMC5883_ADDRESS, adr+1)
        val = (high << 8) + low

        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val


    # Get z-axle magnetic field
    def read_magnet_detector (self):
        if ( self.testMode ):
            print ("@read_magnet_detector: Read magnetometer's z axis")
        else:
            bus.write_byte_data(HMC5883_ADDRESS, 0, 0b01110000)
            bus.write_byte_data(HMC5883_ADDRESS, 1, 0b00100000)
            bus.write_byte_data(HMC5883_ADDRESS, 2, 0b00000000)
 
            scale = 0.92

            #y is new z
            z_out = self.read_word_2c(7) * scale
            print("@read_magnet_detector: ", z_out)

    # Get Bearing from the magnetometer
    def get_heading(self):
        if ( self.testMode ):
            print ("@get_heading: Read magnetometer")
        else:

          bus.write_byte_data(HMC5883_ADDRESS, 0, 0b01110000)
          bus.write_byte_data(HMC5883_ADDRESS, 1, 0b00100000)
          bus.write_byte_data(HMC5883_ADDRESS, 2, 0b00000000)
 
          scale = 0.92
        
          #1st 
          x_out = self.read_word_2c(3) * scale

          #orig
          #y_out = self.read_word_2c(7) * scale

          #y is new z
          z_out = self.read_word_2c(7) * scale

          #orig
          #z_out = self.read_word_2c(5) * scale
          #z is new y
          y_out = self.read_word_2c(5) * scale
 
          #flip x and y xis value due to the sensor mounting orientation
          x_out = x_out*-1
          y_out = y_out*-1

          heading = math.atan2(y_out, x_out)

          if (heading < 0):
            heading += 2 * math.pi

          print("@get_heading: ", math.degrees(heading))
#          print("Heading: ", heading)
          return heading

    # Measure humidity
    def measure_humidity(self):
        if ( self.testMode ):
            print ("@measure_humidity")
        else:
            humidity, temperature = Adafruit_DHT.read_retry(AM2302, GPIO_PIN_AM2302)

            # Un-comment the line below to convert the temperature to Fahrenheit.
            #temperature = temperature * 9/5.0 + 32

            if humidity is not None:
                print('@measure_humidity: Humidity={0:0.1f}%'.format(humidity))
                #print('Temp={0:0.1f}*F  Humidity={1:0.1f}%'.format(temperature, humidity))
            else:
                print('@measure_humidity: Failed to get measurement. Try again!')

    # Measure temperature
    def measure_temperature(self):
        if ( self.testMode ):
            print ("@measure_temperature")
        else:
            humidity, temperature = Adafruit_DHT.read_retry(AM2302, GPIO_PIN_AM2302)

            # Un-comment the line below to convert the temperature to Fahrenheit.
            temperatureF = temperature * 9/5.0 + 32

            if temperature is not None:
                print('@measure_temperature: Temp={0:0.1f}*C  {1:0.1f}*F'.format(temperature, temperatureF))
                #print('Temp={0:0.1f}*F  Humidity={1:0.1f}%'.format(temperature, humidity))
            else:
                print('@measure_temperature: Failed to get measurement. Try again!')

    # turn using speed sensor
    def turn_right_using_speed_sensor(self,slots):
        if ( self.testMode ):
            print ("@turn_right_using_speed_sensor")
        else:
            print ("@turn_right_using_speed_sensor")
            count = 0
            ticks = slots * 2 #two state changes per slot: up/down and down/up"
            last_read = 0
            cur_read = 0
            while(count<ticks):
                #if state changes from high to low or low to high
                cur_read = gpio.input(GPIO_PIN_SPEED_SENSOR_LEFT)
                if last_read != cur_read:
                    count += 1
                    last_read = cur_read
                
                #debug
                print(count)

                gpio.output(GPIO_PIN_LEFT_MOTOR_FORWARD, True)
                gpio.output(GPIO_PIN_LEFT_MOTOR_BACKWARD, False)
                Lpwm.start(60)


            #shutdown motor
            gpio.output(GPIO_PIN_LEFT_MOTOR_FORWARD, False)
            gpio.output(GPIO_PIN_LEFT_MOTOR_BACKWARD, False)
            gpio.output(GPIO_PIN_RIGHT_MOTOR_FORWARD, False)
            gpio.output(GPIO_PIN_RIGHT_MOTOR_BACKWARD, False)
            Lpwm.stop()
            Rpwm.stop()

            #pause before next step
            time.sleep(PAUSE)


    # turn using speed sensor
    def turn_left_using_speed_sensor(self,slots):
        if ( self.testMode ):
            print ("@turn_left_using_speed_sensor")
        else:
            print ("@turn_left_using_speed_sensor")
            count = 0
            ticks = slots * 2 #two state changes per slot: up/down and down/up"
            last_read = 0
            cur_read = 0
            while(count<ticks):
                cur_read = gpio.input(GPIO_PIN_SPEED_SENSOR_RIGHT)
                #if state changes from high to low or low to high
                if last_read != cur_read:
                    count += 1
                    last_read = cur_read
                
                #debug
                print(count)

                gpio.output(GPIO_PIN_RIGHT_MOTOR_FORWARD, True)
                gpio.output(GPIO_PIN_RIGHT_MOTOR_BACKWARD, False)
                Rpwm.start(RMOTORSPEED)


            #shutdown motor
            gpio.output(GPIO_PIN_LEFT_MOTOR_FORWARD, False)
            gpio.output(GPIO_PIN_LEFT_MOTOR_BACKWARD, False)
            gpio.output(GPIO_PIN_RIGHT_MOTOR_FORWARD, False)
            gpio.output(GPIO_PIN_RIGHT_MOTOR_BACKWARD, False)
            Lpwm.stop()
            Rpwm.stop()

            #pause before next step
            time.sleep(PAUSE)




    # Move forward so many units
    def go_forward(self,units,check_ir=0):
        

        if ( self.testMode ):
            for step in range(0,units):
                print ("+Go forward")
                time.sleep(STEP_DURATION)
        else:
            # Turn on the enable pin
            #gpio.output(GPIO_PIN_ENABLE, True)
            for step in range(0,units):
                print ("+Go forward")

                #check for obstacle when check_ir is on(1)
                if check_ir == 1:
                    if (not gpio.input(GPIO_PIN_IR_SENSOR_LEFT)):
                        print("+Abort: Obstacle detected on the left IR sensor")
                        break
                    if (not gpio.input(GPIO_PIN_IR_SENSOR_RIGHT)):
                        print("+Abort: Obstacle detected on the right IR sensor")
                        break

                gpio.output(GPIO_PIN_LEFT_MOTOR_FORWARD, True)
                gpio.output(GPIO_PIN_LEFT_MOTOR_BACKWARD, False)
                Lpwm.start(LMOTORSPEED)

                gpio.output(GPIO_PIN_RIGHT_MOTOR_FORWARD, True)
                gpio.output(GPIO_PIN_RIGHT_MOTOR_BACKWARD, False)
                Rpwm.start(RMOTORSPEED)

                time.sleep(STEP_DURATION)

            #shutdown motor
            gpio.output(GPIO_PIN_LEFT_MOTOR_FORWARD, False)
            gpio.output(GPIO_PIN_LEFT_MOTOR_BACKWARD, False)
            gpio.output(GPIO_PIN_RIGHT_MOTOR_FORWARD, False)
            gpio.output(GPIO_PIN_RIGHT_MOTOR_BACKWARD, False)
            Lpwm.stop()
            Rpwm.stop()

            #pause before next step
            time.sleep(PAUSE)


    def clear(self):
       gpio.cleanup()


    # Move backward so many units
    def go_backward(self,units):
        if ( self.testMode ):
            for step in range(0,units):
                print ("+Go backward")
                time.sleep(STEP_DURATION)
        else:
            # Turn on the enable pin
            #gpio.output(GPIO_PIN_ENABLE, True)
            for step in range(0,units):
                print ("+Go backward")
                gpio.output(GPIO_PIN_LEFT_MOTOR_FORWARD, False)
                gpio.output(GPIO_PIN_LEFT_MOTOR_BACKWARD, True)
                Lpwm.start(LMOTORSPEED)

                gpio.output(GPIO_PIN_RIGHT_MOTOR_FORWARD, False)
                gpio.output(GPIO_PIN_RIGHT_MOTOR_BACKWARD, True)
                Rpwm.start(RMOTORSPEED)

                time.sleep(STEP_DURATION)

            #shutdown motor
            gpio.output(GPIO_PIN_LEFT_MOTOR_FORWARD, False)
            gpio.output(GPIO_PIN_LEFT_MOTOR_BACKWARD, False)
            gpio.output(GPIO_PIN_RIGHT_MOTOR_FORWARD, False)
            gpio.output(GPIO_PIN_RIGHT_MOTOR_BACKWARD, False)
            Lpwm.stop()
            Rpwm.stop()

            #pause before next step
            time.sleep(PAUSE)

    # turn left in sec
    def turn_left(self,units):
        if ( self.testMode ):
            for step in range(0,units):
                print ("+Turn left")
                time.sleep(STEP_DURATION)
        else:
                #turn motors
            for step in range(0,units):
                print ("+Turn left")
                gpio.output(GPIO_PIN_LEFT_MOTOR_FORWARD, False)
                gpio.output(GPIO_PIN_LEFT_MOTOR_BACKWARD, True)
                Lpwm.start(LMOTORSPEED)

                gpio.output(GPIO_PIN_RIGHT_MOTOR_FORWARD, True)
                gpio.output(GPIO_PIN_RIGHT_MOTOR_BACKWARD, False)
                Rpwm.start(RMOTORSPEED)

                time.sleep(STEP_DURATION)

            #shutdown motor
            gpio.output(GPIO_PIN_LEFT_MOTOR_FORWARD, False)
            gpio.output(GPIO_PIN_LEFT_MOTOR_BACKWARD, False)
            gpio.output(GPIO_PIN_RIGHT_MOTOR_FORWARD, False)
            gpio.output(GPIO_PIN_RIGHT_MOTOR_BACKWARD, False)
            Lpwm.stop()
            Rpwm.stop()

            #pause before next step
            time.sleep(PAUSE)

    # turn right in sec
    def turn_right(self,units):
        if ( self.testMode ):
            for step in range(0,units):
                print ("+Turn right")
                time.sleep(STEP_DURATION)
        else:
                #turn motors
            for step in range(0,units):
                print ("+Turn right")
                gpio.output(GPIO_PIN_LEFT_MOTOR_FORWARD, True)
                gpio.output(GPIO_PIN_LEFT_MOTOR_BACKWARD, False)
                Lpwm.start(LMOTORSPEED)

                gpio.output(GPIO_PIN_RIGHT_MOTOR_FORWARD, False)
                gpio.output(GPIO_PIN_RIGHT_MOTOR_BACKWARD, True)
                Rpwm.start(RMOTORSPEED)

                time.sleep(STEP_DURATION)

            #shutdown motor
            gpio.output(GPIO_PIN_LEFT_MOTOR_FORWARD, False)
            gpio.output(GPIO_PIN_LEFT_MOTOR_BACKWARD, False)
            gpio.output(GPIO_PIN_RIGHT_MOTOR_FORWARD, False)
            gpio.output(GPIO_PIN_RIGHT_MOTOR_BACKWARD, False)
            Lpwm.stop()
            Rpwm.stop()

            #pause before next step
            time.sleep(PAUSE)


    def heading_error(self,target_heading):

        #get current heading
        current_heading = self.get_heading()

        PID_error = target_heading - current_heading

        if PID_error > math.pi:
          PID_error -= 2 * math.pi

        if PID_error < -math.pi:
          PID_error += 2 * math.pi

        #print("target: ", math.degrees(target_heading), "/" , target_heading, "current: ", math.degrees(current_heading), "/", current_heading, "error:", PID_error)
        print("target: ", int(math.degrees(target_heading)), "/" "current: ", int(math.degrees(current_heading)), "/", "error:", int(math.degrees(PID_error)))

        return PID_error
   

    def go_straight(self,target_heading,units):

        #convert to radians
        target_heading = math.radians(target_heading)



        for step in range(0,units*10):
          print ("====== Loop " , step)
          #initial pwm
          Lpwm = 100
          Rpwm = 100

          PID_error = self.heading_error(target_heading)        
          K =  10
        

          # steer to right
          if PID_error > 0:
            #reduce right motor PWM
            Rpwm -= int(K*PID_error)
            print("Steer to right by ", math.degrees(PID_error), " Set Right pwm to ", Rpwm)

          # steer to left
          if PID_error < 0:
            #reduce left motor PWM
            Lpwm += int(K*PID_error)
            print("Steer to left by ", math.degrees(PID_error), " Set Left pwm to ", Lpwm)
       
          time.sleep(0.1)

        


    def turn_right_by_degree(self,degrees):

        #TODO: check degrees to be between 0 and 180
        if ( self.testMode ):
            for step in range(0,units):
                print ("+Turn right")
        else:

            #store initial_heading needed to calculate turning radian
            init_heading = self.get_heading()

            #how much to turn (in radians)
            turning_radian = math.radians(degrees)

            #print("turning radian ", turning_radian)
            print("----------------------------------------------------------")

            #current_radian starting at 0 and keeps track of turning
            current_radian = 0

            #keep turning until turning_radian is reached
            #note: why >= =0.1? It may register negative radian in the beginning 
            while ((current_radian < turning_radian) and (current_radian >= -0.1)): 
                
                #turn motors
                gpio.output(GPIO_PIN_LEFT_MOTOR_FORWARD, True)
                gpio.output(GPIO_PIN_LEFT_MOTOR_BACKWARD, False)
                Lpwm.start(100)

                gpio.output(GPIO_PIN_RIGHT_MOTOR_FORWARD, False)
                gpio.output(GPIO_PIN_RIGHT_MOTOR_BACKWARD, True)
                Rpwm.start(100)
#
                time.sleep(STEP_DURATION)

                current_heading = self.get_heading()

                #if crossed compass wrapping
                if current_heading < init_heading - math.pi:
                    print("+++++ crossing north @", current_heading)
                    current_heading += 2 * math.pi

                #calculate how much it turned so far
                current_radian = current_heading - init_heading

                #current_radian = init_heading - abs(current_heading)
                print ("+Turn right init heading: ", init_heading, "current heading: ", current_heading ,"current radian: " , current_radian  , "    turning radian: " , turning_radian )



            #print("+Turn Completed")
            print("Turn: ", degrees, " Start: ", math.degrees(init_heading),  " End: ", math.degrees(current_heading))
            #shutdown motor
            #self.stop()


    def turn_left_by_degree(self,degrees):

        #TODO: check degrees to be between 0 and 180
        if ( self.testMode ):
            for step in range(0,units):
                print ("+Turn left")
                #self._waitTurnBias()
        else:

            #store initial_heading needed to calculate turning radian
            init_heading = self.get_heading()

            #how much to turn (in radians)
            turning_radian = math.radians(degrees)

            print("----------------------------------------------------------")
            #print("turning radian ", turning_radian)

            #current_radian starting at 0 and keeps track of turning
            current_radian = 0

            #keep turning until turning_radian is reached
            #note: why >= =0.1? It may register negative radian in the beginning 
            while ((current_radian < turning_radian) and (current_radian >= -0.1)): 
                
                #turn motors
                gpio.output(GPIO_PIN_LEFT_MOTOR_FORWARD, False)
                gpio.output(GPIO_PIN_LEFT_MOTOR_BACKWARD, True)
                Lpwm.start(100)

                gpio.output(GPIO_PIN_RIGHT_MOTOR_FORWARD, True)
                gpio.output(GPIO_PIN_RIGHT_MOTOR_BACKWARD, False)
                Rpwm.start(100)

                time.sleep(STEP_DURATION)

                current_heading = self.get_heading()

                #if crossed compass wrapping
                if current_heading > init_heading +  math.pi:
                    print("+++++ crossing north @", current_heading)
                    current_heading -= 2 * math.pi

                #calculate how much it turned so far
                current_radian = init_heading - current_heading

                print ("+Turn left init heading: ", init_heading, "current heading: ", current_heading ,"current radian: " , current_radian  , "    turning radian: " , turning_radian )

            #print("+Turn Completed")
            print("Turn: ", degrees, " Start: ", math.degrees(init_heading),  " End: ", math.degrees(current_heading))
            #shutdown motor
            #self.stop()


    # Stop all the motors 
    def stop(self):
            if ( self.testMode ):
                print ("+All Stop")
            else:
                #gpio.output(GPIO_PIN_ENABLE, False)
                gpio.output(GPIO_PIN_LEFT_MOTOR_FORWARD, False)
                gpio.output(GPIO_PIN_LEFT_MOTOR_BACKWARD, False)
                gpio.output(GPIO_PIN_RIGHT_MOTOR_FORWARD, False)
                gpio.output(GPIO_PIN_RIGHT_MOTOR_BACKWARD, False)
                Lpwm.stop()
                Rpwm.stop()
    

    # turn on LED
    def led_on(self):
            if ( self.testMode ):
                print ("+LED on")
            else:
                print ("+LED on")
                gpio.output(GPIO_PIN_LED, True)

    # turn off LED
    def led_off(self):
            if ( self.testMode ):
                print ("+LED off")
            else:
                print ("+LED off")
                gpio.output(GPIO_PIN_LED, False)


    def capture_image(self):
        if ( self.testMode ):
            print ("@capture_image: Take photo and save it to " + IMG_LOC)
        else:

            #create instance
            camera = picamera.PiCamera()
            #camera.resolution = (1640, 922)
            camera.resolution = (1280, 720)

            #awb mode
            #see http://picamera.readthedocs.io/en/release-1.10/api_camera.html
            camera.awb_mode = 'auto'

            #flip horizontally & vertically
            camera.vflip = True
            camera.hflip = True

            #photopath=tempfile.gettempdir() + '/' + fileprefix + '-' + time.strftime("%Y%m%d-%a-%H%M-%S") + '.jpg'
            #photopath='/var/www/pydev/' + time.strftime("%Y%m%d-%a-%H%M-%S") + '.jpg'
            photopath=IMG_LOC + time.strftime("%Y%m%d-%H%M-%S") + '.jpg'
            print ("@capture_image: Take photo and save it as " + photopath)
            camera.capture(photopath)
            camera.close()

    def capture_manual_image(self,iso,shutter_speed):
        if ( self.testMode ):
            print ("@capture_manual_image: Take photo and save it to " + IMG_LOC)
        else:

            #create instance
            camera = picamera.PiCamera()
            camera.resolution = (1640, 922)
            
            #flip horizontally & vertically
            camera.vflip = True
            camera.hflip = True

            #set framerate to 1/6
            #TODO: should this be a parameter?
            #camera.framerate =  Fraction(1,6)

            #set capture ISO (100 - 800)
            camera.iso = iso

            # Wait for the automatic gain control to settle
            time.sleep(2)

            #set awb
            #camera.awb_mode = 'off'

            #set shutter speed in microsecon. max value is 6000000us (6000ms or 6s)
            camera.shutter_speed = shutter_speed

            #photopath='/var/www/pydev/' + time.strftime("%Y%m%d-%a-%H%M-%S") + '.jpg'
            photopath=IMG_LOC + time.strftime("%Y%m%d-%H%M-%S") + '_iso-' + str(iso) + '_ss-' + str(shutter_speed) +'.jpg'
            print ("@capture_manual_image: Take photo with ISO="+str(iso)+" and Shutter Speed="+str(shutter_speed)+ ". Save it as " + photopath)
            camera.capture(photopath)
            camera.close()

    def arm_lance (self):
        if ( self.testMode ):
            print ("@arm_lance")
        else:
            print ("@arm_lance")
            lance_servo_pwm.start(LANCE_OPEN)
            time.sleep(2)

    def disarm_lance (self):
        if ( self.testMode ):
            print ("@disarm_lance")
        else:
            print ("@disarm_lance")
            lance_servo_pwm.start(LANCE_CLOSE)
            time.sleep(2)






