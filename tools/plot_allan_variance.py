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
import pylab

def main():
    parser = optparse.OptionParser()

    options, args = parser.parse_args()

    lines = open(args[0]).readlines()
    header, lines = lines[0], lines[1:]
    columns = [x.strip() for x in header.split(',')]

    data = dict([(x, []) for x in columns])

    for line in lines:
        fields = [float(x.strip()) for x in line.split(',')]
        for i, value in enumerate(fields):
            data[columns[i]].append(value)

    time_field = columns[0]
    time_data = data[time_field]

    for key, value in data.iteritems():
        if key == time_field:
            continue
        if 'time' in key:
            continue
        pylab.plot(time_data, value, label=key)

    pylab.loglog()
    pylab.grid()
    pylab.legend()
    pylab.show()


if __name__ == '__main__':
    main()
