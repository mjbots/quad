#!/usr/bin/env python

# Copyright 2015 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

import optparse
import serial
import sys


def main():
    parser = optparse.OptionParser()
    parser.add_option('-i', '--input', help='blg parameters')
    parser.add_option('-s', '--serial', help='serial port', 
                      default='/dev/ttyACM99')
    
    options, args = parser.parse_args()
    assert len(args) == 0

    port = serial.Serial(options.serial, baudrate=115200, timeout=0.1)

    data = [x.strip() for x in open(options.input).readlines()]
    for parameter in data:
        port.write(parameter + '\r\n')
        # Now query the parameter back.
        query = ' '.join(parameter.split(' ', 2)[0:2])
        port.write(query + '\r\n')
        result = port.read(1000).strip()
        print result

if __name__ == '__main__':
    main()


