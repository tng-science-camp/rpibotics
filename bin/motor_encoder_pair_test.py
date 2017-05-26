#!/usr/bin/env python3
import argparse
import curses
import time
from rover.dc_motor import DCMotor
from rover.optocoupler_encoder import OptocouplerEncoder

parser = argparse.ArgumentParser(
    description="Runs DC motor with the option to change the PWM parameters.")
parser.add_argument("-f", dest="frequency", default=20, type=int,
                    help="Set the frequency of the PWM in Hz")
parser.add_argument("-d", dest="duty_cycle", default=70, type=int,
                    help="Set the duty cycle of the PWM in %%")
parser.add_argument("-e", dest="encoder_pin", default=7, type=int,
                    help="GPIO pin of the optocoupler")
args = vars(parser.parse_args())


def main(stdscr,
         frequency=args['frequency'],
         duty_cycle=args['duty_cycle'],
         encoder_pin=args['encoder_pin']):
    quit = False
    motor = DCMotor(13, 26, 19, frequency)
    encoder = OptocouplerEncoder(encoder_pin, s=20)
    encoder.run()
    turn_clockwise = True
    stop = True
    stdscr.nodelay(True)
    stdscr.clear()
    stdscr.addstr("q       exit program\n")
    stdscr.addstr("s       toggle motor start or stop\n")
    stdscr.addstr("c       toggle the motor direction\n")
    stdscr.addstr("r       reset the encoder\n")
    stdscr.addstr("RIGHT   increase PWM frequency\n")
    stdscr.addstr("LEFT    decrease PWM frequency\n")
    stdscr.addstr("UP      increase PWM duty cycle\n")
    stdscr.addstr("DOWN    decrease PWM duty cycle\n")
    stdscr.addstr("===============================\n")
    stdscr.addstr("Status:        ")
    if stop:
        stdscr.addstr("Stopped\n")
    else:
        stdscr.addstr("Running\n")
    stdscr.addstr("Direction:     ")
    if turn_clockwise:
        stdscr.addstr("Clockwise\n")
    else:
        stdscr.addstr("Counter-Clockwise\n")
    stdscr.addstr("Frequency:     " + str(frequency) + "\n")
    stdscr.addstr("Duty Cycle:    " + str(duty_cycle) + "\n")
    stdscr.addstr("Rotations:     " + str(encoder.get_rotations()) + "\n")
    stdscr.addstr("Rotation Rate: " + str(encoder.get_rotation_rate()) + "\n")
    while not quit:
        try:
            code = stdscr.getch()
            stdscr.clear()
            if code == ord('q'):
                motor.stop()
                stdscr.addstr("Exiting in 3 sec: ")
                for i in range(3, 0, -1):
                    stdscr.addstr(".")
                    stdscr.refresh()
                    time.sleep(1)
                quit = True
            else:
                if code == ord('s'):
                    stop = not stop
                    if stop:
                        motor.stop()
                    elif turn_clockwise:
                        motor.turn_clockwise(duty_cycle)
                    else:
                        motor.turn_counter_clockwise(duty_cycle)
                elif code == ord('c'):
                    turn_clockwise = not turn_clockwise
                    if turn_clockwise:
                        motor.turn_clockwise(duty_cycle)
                    else:
                        motor.turn_counter_clockwise(duty_cycle)
                elif code == ord('r'):
                    encoder.reset()
                elif code == curses.KEY_LEFT:
                    frequency -= 10
                    motor.change_pwm_frequency(frequency)
                elif code == curses.KEY_RIGHT:
                    frequency += 10
                    motor.change_pwm_frequency(frequency)
                elif code == curses.KEY_UP:
                    duty_cycle +=2
                    motor.change_pwm_frequency(duty_cycle)
                    if turn_clockwise:
                        motor.turn_clockwise(duty_cycle)
                    else:
                        motor.turn_counter_clockwise(duty_cycle)
                elif code == curses.KEY_DOWN:
                    duty_cycle -= 2
                    if turn_clockwise:
                        motor.turn_clockwise(duty_cycle)
                    else:
                        motor.turn_counter_clockwise(duty_cycle)

                stdscr.addstr("q       exit program\n")
                stdscr.addstr("s       toggle motor start or stop\n")
                stdscr.addstr("c       toggle the motor direction\n")
                stdscr.addstr("r       reset the encoder\n")
                stdscr.addstr("RIGHT   increase PWM frequency\n")
                stdscr.addstr("LEFT    decrease PWM frequency\n")
                stdscr.addstr("UP      increase PWM duty cycle\n")
                stdscr.addstr("DOWN    decrease PWM duty cycle\n")
                stdscr.addstr("===============================\n")
                stdscr.addstr("Status:        ")
                if stop:
                    stdscr.addstr("Stopped\n")
                else:
                    stdscr.addstr("Running\n")
                stdscr.addstr("Direction:     ")
                if turn_clockwise:
                    stdscr.addstr("Clockwise\n")
                else:
                    stdscr.addstr("Counter-Clockwise\n")
                stdscr.addstr("Frequency:     " + str(frequency) + "\n")
                stdscr.addstr("Duty Cycle:    " + str(duty_cycle) + "\n")
                stdscr.addstr("Rotations:     " +
                              str(encoder.get_rotations()) + "\n")
                stdscr.addstr("Rotation Rate: " + str(
                    encoder.get_rotation_rate()) + "\n")
                stdscr.refresh()
        except Exception as e:
            # No input
            pass

    curses.endwin()


curses.wrapper(main)
