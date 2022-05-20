#!/usr/bin/env python3

# Copyright 2020-2022 Josh Pieper, jjp@pobox.com.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

'''Set servo-level configuration for a quad A1 robot.'''


import argparse
import asyncio
import moteus
import moteus_pi3hat
import os
import subprocess
import sys
import tempfile


SCRIPT_PATH = os.path.dirname(__file__)
MOTEUS_TOOL = ['moteus_tool',
               '--pi3hat-cfg',
               '1=1,2,3;2=4,5,6;3=7,8,9;4=10,11,12',
               ]


CONFIG_A1 = {
    'servopos.position_min' : [
        ('1,2,4,5,7,8,10,11', '-.70'),
        ('3,6,9,12', '-.25'),
    ],
    'servopos.position_max' : [
        ('1,2,4,5,7,8,10,11', '.70'),
        ('3,6,9,12', '.25'),
    ],
    'servo.pwm_min' : '0.006',
    'servo.flux_brake_min_voltage' : '20.5',
    'servo.flux_brake_resistance_ohm' : '0.05',
    'servo.pid_position.kp' : [
        ('1,2,4,5,7,8,10,11', '50'),
        ('3,6,9,12', '200'),
    ],
    'servo.feedforward_scale' : '0.0',
    'servo.pid_position.kd' : '6',
    'servo.pid_dq.ki' : '150.0',
    'motor.unwrapped_position_scale' : [
        ('1,3,4,6,7,9,10,12', '0.16666667'),
        ('2,5,8,11', '0.093750'),
    ],
}

CONFIG_A2 = {
    'servopos.position_min' : [
        ('1,2,4,5,7,8,10,11', '-.70'),
        ('3,6,9,12', '-.25'),
    ],
    'servopos.position_max' : [
        ('1,2,4,5,7,8,10,11', '.70'),
        ('3,6,9,12', '.25'),
    ],
    'servo.flux_brake_min_voltage' : '20.5',
    'servo.flux_brake_resistance_ohm' : '0.05',
    'servo.pid_position.kp' : [
        ('1,2,4,5,7,8,10,11', '100'),
        ('3,6,9,12', '300'),
    ],
    'servo.velocity_threshold' : '0',
    'servo.pid_position.kd' : '9',
    'servo.pid_dq.kp' : '0.026',
    'servo.pid_dq.ki' : '56',
    'servo.encoder_filter.kp' : '2513',
    'servo.encoder_filter.ki' : '1579136',
    'motor.unwrapped_position_scale' : [
        ('1,3,4,6,7,9,10,12', '0.16666667'),
        ('2,5,8,11', '0.1071428'),
    ],
}


async def config_servo(args, transport, servo_id):
    if args.verbose:
        print(f"*** SERVO {servo_id} ***")

    c = moteus.Controller(id=servo_id, transport=transport)
    s = moteus.Stream(c, verbose=args.verbose)

    await s.flush_read()

    CONFIG = CONFIG_A1 if args.a1 else CONFIG_A2

    for key, data_or_value in CONFIG.items():
        if type(data_or_value) == str:
            value = data_or_value
            await s.command(
                "conf set {} {}".format(key, value).encode('utf8'))
        else:
            for servo_selector, value in data_or_value:
                ids = set([int(x) for x in servo_selector.split(',')])
                if not servo_id in ids:
                    continue
                await s.command(
                    "conf set {} {}".format(key, value).encode('utf8'))

    await s.command(b'conf write')


async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-v', '--verbose', action='store_true')
    parser.add_argument('--a1', action='store_true')

    args = parser.parse_args()

    if os.geteuid() != 0:
        raise RuntimeError('This must be run as root')

    transport = moteus_pi3hat.Pi3HatRouter(
        servo_bus_map = {
            1: [1, 2, 3],
            2: [4, 5, 6],
            3: [7, 8, 9],
            4: [10, 11, 12],
        },
    )

    for i in range(1, 13):
        await config_servo(args, transport, i)

if __name__ == '__main__':
    asyncio.run(main())
