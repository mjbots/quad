#!/usr/bin/env python

import os
import re
import subprocess
import sys
import time

SCRIPT_PATH = sys.path[0]
HTOOL = os.path.join(SCRIPT_PATH, '..', 'mech', 'build', 'herkulex_tool')
HTOOL_ARGS = ' --servo.stream.type serial --servo.stream.serial.serial_port /dev/ttyACM99 --address 98 --servo.stream.stdio_debug=0 '
DELAY=0.25


def set_drive(value):
    subprocess.check_call(HTOOL + HTOOL_ARGS + ' --value_write gimbal_yaw:%d000' % value, shell=True)


def get_position():
    output = subprocess.check_output(
        HTOOL + HTOOL_ARGS + ' --ram_read gimbal_encoder_yaw', shell=True)
    return int(re.search('\((.*)\)', output).group(1))


def wait_for_stable():
    old = None
    for i in range(100):
        current = get_position()
        if old:
            delta = abs(current - old)
            if delta < 4:
                return current
        time.sleep(0.01)
        old = current

    print 'no stable!'
    return current


def do_loop(start, end, step):
    value = start
    while (end - value) * step >= 0:
        set_drive(value)
        time.sleep(DELAY)
        set_drive(value + step * 2)
        time.sleep(DELAY)
        set_drive(value)
        time.sleep(DELAY)
        
        result = wait_for_stable()
        print value, result
        sys.stdout.flush()

        value += step


def main():
    do_loop(0, 2048, 4)
    do_loop(2048, 0, -4)


if __name__ == '__main__':
    main()
