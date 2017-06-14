#!/usr/bin/env python3
import argparse
import curses
from rpibotics.optocoupler_encoder import OptocouplerEncoder

parser = argparse.ArgumentParser(
    description="Runs the optocoupler_encoder to determine the number of "
                "rotations and the rotation rate.")
parser.add_argument("-p", dest="pin", default=7, type=int,
                    help="GPIO pin of the optocoupler")
parser.add_argument("-s", dest="slit_count", default=20, type=int,
                    help="Number of slits in the encoder disk")
args = vars(parser.parse_args())


def main(stdscr, pin=args['pin'], slit_count=args['slit_count']):
    encoder = OptocouplerEncoder(pin, slit_count=slit_count)
    quit = False
    stdscr.nodelay(True)
    stdscr.clear()
    stdscr.addstr("q   exit program\n")
    stdscr.addstr("r   reset the encoder\n")
    stdscr.addstr("=====================\n")
    stdscr.addstr("Rotations:     " + str(encoder.get_rotations()) + "\n")
    stdscr.addstr("Rotation Rage: " + str(encoder.get_rotation_rate()) + "\n")
    while not quit:
        try:
            code = stdscr.getch()
            if code == ord('q'):
                quit = True
            elif code == ord('r'):
                encoder.reset()
            stdscr.clear()
            stdscr.addstr("q   exit program\n")
            stdscr.addstr("r   reset the encoder\n")
            stdscr.addstr("=====================\n")
            stdscr.addstr(
                "Rotations:     " + str(encoder.get_rotations()) + "\n")
            stdscr.addstr(
                "Rotation Rage: " + str(encoder.get_rotation_rate()) +
                "\n")
            stdscr.refresh()
        except Exception as e:
            # No input
            pass

    curses.endwin()


curses.wrapper(main)
