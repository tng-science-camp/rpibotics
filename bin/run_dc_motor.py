#!/usr/bin/env python3
import argparse
import curses
import time
from rpibotics.dc_motor import DCMotor

parser = argparse.ArgumentParser(
    description="Runs DC motor with the option to change the PWM parameters.")
parser.add_argument("-f", dest="frequency", default=20, type=int,
                    help="Set the frequency of the PWM in Hz")
parser.add_argument("-d", dest="duty_cycle", default=70, type=int,
                    help="Set the duty cycle of the PWM in %%")
args = vars(parser.parse_args())


def main(stdscr, frequency=args['frequency'], duty_cycle=args['duty_cycle']):
    quit = False
    motor = DCMotor(13, 26, 19, frequency)
    turn_clockwise = True
    stop = True
    stdscr.nodelay(True)
    stdscr.clear()
    stdscr.addstr("q       exit program\n")
    stdscr.addstr("s       toggle motor start or stop\n")
    stdscr.addstr("c       toggle the motor direction\n")
    stdscr.addstr("RIGHT   increase PWM frequency\n")
    stdscr.addstr("LEFT    decrease PWM frequency\n")
    stdscr.addstr("UP      increase PWM duty cycle\n")
    stdscr.addstr("DOWN    decrease PWM duty cycle\n")
    stdscr.addstr("===============================\n")
    stdscr.addstr("Status:     ")
    if stop:
        stdscr.addstr("Stopped\n")
    else:
        stdscr.addstr("Running\n")
    stdscr.addstr("Direction:  ")
    if turn_clockwise:
        stdscr.addstr("Clockwise\n")
    else:
        stdscr.addstr("Counter-Clockwise\n")
    stdscr.addstr("Frequency:  " + str(frequency) + "\n")
    stdscr.addstr("Duty Cycle: " + str(duty_cycle))
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
                stdscr.addstr("q       exit program\n")
                stdscr.addstr("s       toggle motor start or stop\n")
                stdscr.addstr("c       toggle the motor direction\n")
                stdscr.addstr("RIGHT   increase PWM frequency\n")
                stdscr.addstr("LEFT    decrease PWM frequency\n")
                stdscr.addstr("UP      increase PWM duty cycle\n")
                stdscr.addstr("DOWN    decrease PWM duty cycle\n")
                stdscr.addstr("===============================\n")
                stdscr.addstr("Status:     ")
                if stop:
                    stdscr.addstr("Stopped\n")
                else:
                    stdscr.addstr("Running\n")
                stdscr.addstr("Direction:  ")
                if turn_clockwise:
                    stdscr.addstr("Clockwise\n")
                else:
                    stdscr.addstr("Counter-Clockwise\n")
                stdscr.addstr("Frequency:  " + str(frequency) + "\n")
                stdscr.addstr("Duty Cycle: " + str(duty_cycle))
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
                #else:
                #    stdscr.addstr("None")
                stdscr.refresh()
        except Exception as e:
            # No input
            pass

    curses.endwin()


curses.wrapper(main)
