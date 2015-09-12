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

import blessings
import json
import optparse
import socket
import threading
import time


class Viewer(object):
    def __init__(self, options, args):
        self.options = options
        self.args = args

        if ':' not in options.target:
            self.target = (options.target, 13380)
        else:
            host, port = options.target.split(':', 1)
            self.target = (host, int(port))

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def poll_task(self):
        name_str = '[' + ','.join(['"%s"' % x for x in self.args]) + ']'
        while True:
            self.sock.sendto(
                '{"command":"get","names":%s}' % name_str, self.target)
            time.sleep(self.options.interval)

    def read_task(self):
        while True:
            data = self.sock.recv(15000)

            j = json.loads(data)
            if not 'type' in j:
                print 'unexpected message:', data
                continue

            if j['type'] == 'reply':
                self.handle_reply(j, data)

    def handle_reply(self, msg, data):
        if self.options.raw:
            print data
        else:
            print msg['reply']


def main():
    parser = optparse.OptionParser()
    parser.add_option('--target', '-t', default='localhost')
    parser.add_option('--interval', '-i', type='float', default=0.2)
    parser.add_option('--raw', '-r', action='store_true')

    options, args = parser.parse_args()

    viewer = Viewer(options, args)

    pt = threading.Thread(target=viewer.poll_task)
    pt.daemon = True
    pt.start()
    rt = threading.Thread(target=viewer.read_task)
    rt.daemon = True
    rt.start()
    while True:
        time.sleep(1)

if __name__ == '__main__':
    main()
