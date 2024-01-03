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

'''Zero a leg on the quad A1'''


import argparse
import asyncio
import moteus
import moteus_pi3hat
import sys


class Servo:
    def __init__(self, transport, id):
        self.id = id
        self.transport = transport
        self.controller = moteus.Controller(id=id, transport=transport)
        self.stream = moteus.Stream(self.controller)

    async def read_position(self):
        servo_stats = await self.stream.read_data("servo_stats")
        return servo_stats.position

    async def zero_offset(self):
        await self.stream.command("d cfg-set-output 0".encode('latin1'))
        await self.stream.command("conf write".encode('latin1'))
        await self.stream.command("d rezero".encode('latin1'))

    async def flush_read(self):
        await self.stream.flush_read()


async def async_readline(fd):
    result = b''
    while True:
        this_char = await fd.read(1)
        if this_char == b'\n' or this_char == b'\r':
            return result
        result += this_char


async def main():
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument(
        '-l', '--leg', type=int, help='leg (1-4) to zero')

    args = parser.parse_args()
    servo_ids = {
        1: [1, 2, 3],
        2: [4, 5, 6],
        3: [7, 8, 9],
        4: [10, 11, 12],
    }[args.leg]

    transport = moteus_pi3hat.Pi3HatRouter(
        servo_bus_map = {
            1: [1, 2, 3],
            2: [4, 5, 6],
            3: [7, 8, 9],
            4: [10, 11, 12],
        },
    )
    servos = [Servo(transport, x) for x in servo_ids]
    [await servo.flush_read() for servo in servos]

    aio_stdin = moteus.aiostream.AioStream(sys.stdin.buffer.raw)
    read_future = asyncio.create_task(async_readline(aio_stdin))

    while True:
        print(', '.join([f"{servo.id: 2d}: {await servo.read_position():7.3f}"
                       for servo in servos]) + '    ',
              end='\r', flush=True)
        if read_future.done():
            break

    print()
    print()
    print("Zeroing servos")

    [await servo.zero_offset() for servo in servos]

    print("DONE")


if __name__ == '__main__':
    asyncio.run(main())
