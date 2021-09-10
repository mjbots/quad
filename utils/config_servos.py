#!/usr/bin/env python3

# Copyright 2020 Josh Pieper, jjp@pobox.com.
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


import os
import subprocess
import sys
import tempfile


SCRIPT_PATH = os.path.dirname(__file__)
MOTEUS_TOOL = ['moteus_tool',
               '--pi3hat-cfg',
               '1=1,2,3;2=4,5,6;3=7,8,9;4=10,11,12',
               ]


CONFIG = {
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


def run(*args, **kwargs):
    print('RUN: ', *args, **kwargs)
    subprocess.check_call(*args, **kwargs)


def main():
    if os.geteuid() != 0:
        raise RuntimeError('This must be run as root')

    for key, data_or_value in CONFIG.items():
        if type(data_or_value) == str:
            with tempfile.NamedTemporaryFile(delete=False) as config:
                value = data_or_value

                config.write(
                    'conf set {} {}\n'.format(key, value).encode('utf8'))
                config.flush()

                run(MOTEUS_TOOL + ['-t1-12', '--write-config', config.name])
        else:
            data = data_or_value
            for servo_selector, value in data:
                with tempfile.NamedTemporaryFile(delete=False) as config:
                    config.write(
                        'conf set {} {}\n'.format(key, value).encode('utf8'))
                    config.flush()
                    run(MOTEUS_TOOL + ['-t{}'.format(servo_selector),
                         '--write-config', config.name])

    # Now store them all persistently.
    with tempfile.NamedTemporaryFile(delete=False) as config:
        config.write(b'conf write\n')
        config.flush()
        run(MOTEUS_TOOL + ['-t1-12', '--write-config', config.name])


if __name__ == '__main__':
    main()
