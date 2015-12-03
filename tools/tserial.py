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

'''%prog [options]

Interact with a serial device capable of reporting tlog style
telemetry.  Optionally log the data to disk in a tlog file.
'''

import optparse
import serial
import struct

class Serial(object):
    def __init__(self, options):
        self.options = options
        self.port = serial.Serial(port=options.serial,
                                  baudrate=options.baudrate)

        # Try to stop anything that might be spewing.
        self.stop()

        # Try to dump anything that is still in the receive queue.
        self.port.setTimeout(0.1)
        result = self.port.read(8192)
        print 'ignored %d bytes on start' % len(result)

    def stop(self):
        self.port.write('\ntel stop\n')

    def readline(self):
        while True:
            line = self.port.readline()
            if line.startswith('unknown'):
                continue
            if line.strip() == '':
                continue
            return line

    def list(self):
        result = []
        self.port.write('\ntel list\n')
        while True:
            line = self.readline()
            if line.startswith("OK"):
                break
            result.append(line.strip())

        return result

    def schema(self, name):
        self.port.write('\ntel schema %s\n' % name)
        line = self.readline()
        assert line.startswith('schema ' + name), 'got unexpected schema response: ' + line
        size_str = self.port.read(4)
        assert len(size_str) == 4
        size = struct.unpack('<I', size_str)[0]

        data = self.port.read(size)
        return data

    def rate(self, name, rate):
        self.port.write('\ntel rate %s %d\n' % (name, rate))

    def read_next_data(self):
        # Read until we get an "emit" line.
        self.port.setTimeout(None)
        line = ''
        while True:
            line = self.readline()
            if line.startswith('emit '):
                break

        name = line.split(' ')[1].strip()
        size_str = self.port.read(4)
        assert len(size_str) == 4
        size = struct.unpack('<I', size_str)[0]

        data = self.port.read(size)
        return name, data


class LogWriter(object):
    def __init__(self, name):
        self.fd = open(name, 'wb')
        self.fd.write('TLOG0002')

        self.fd.flush()

        self.next_identifier = 1
        self.names = {}

    BLOCK_SCHEMA = 1
    BLOCK_DATA = 2

    def make_pstring(self, data):
        return struct.pack('<I', len(data)) + data

    def _make_schema_block(self, identifier, name, schema):
        result = ''
        result += struct.pack('<II', identifier, 0)
        result += self.make_pstring(name)
        result += schema
        return result

    def _make_data_block(self, identifier, data):
        result = struct.pack('<IH', identifier, 0) + data
        return result

    def write_schema(self, name, schema):
        identifier = self.next_identifier
        self.next_identifier += 1

        self.names[name] = identifier

        self.write_block(self.BLOCK_SCHEMA,
                         self._make_schema_block(identifier, name, schema))

    def write_data(self, name, data):
        identifier = self.names[name]
        self.write_block(self.BLOCK_DATA,
                         self._make_data_block(identifier, data))

    def write_block(self, block_id, data):
        self.fd.write(struct.pack('<HI', block_id, len(data)) + data)
        self.fd.flush()


def main():
    usage, description = __doc__.split('\n\n', 1)
    parser = optparse.OptionParser(usage=usage, description=description)

    parser.add_option('--serial', '-s', default='/dev/ttyACM0')
    parser.add_option('--baudrate', '-b', type='int', default=115200)
    parser.add_option('--list', '-l', action='store_true')
    parser.add_option('--name', '-n', action='append', default=[])
    parser.add_option('--rate', '-r', type='int', default=1,
                      help='1 is every update, otherwise ms')
    parser.add_option('--output', '-o', help='output tlog file')

    options, args = parser.parse_args()

    ser = Serial(options)
    if options.list:
        print '\n'.join(ser.list())
        return

    if len(options.name) == 0:
        # If no names are specified, get everything.
        print 'getting names'
        options.name = ser.list()

    output = None
    if options.output:
        output = LogWriter(options.output)

    print 'getting schemas'
    # Get the schema for all the requested things.
    for name in options.name:
        schema = ser.schema(name)
        if output:
            output.write_schema(name, schema)
        print 'got schema for %s len %d' % (name, len(schema))

    print 'setting rates'
    # Now start everything being sent out.
    for name in options.name:
        ser.rate(name, options.rate)

    print 'starting to record'

    # Now, we just continue reading, looking for more data to come
    # out.
    try:
        records = 0
        while True:
            name, data = ser.read_next_data()
            if output:
                output.write_data(name, data)
            records += 1
            print 'count: %d\r' % records,
    finally:
        pass
        #ser.stop()


if __name__ == '__main__':
    main()
