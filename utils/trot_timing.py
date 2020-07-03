#!/usr/bin/python3 -B

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

############
# Brainstorming
#
# Quantities I care about:
#
#  speed => overall speed over ground
#  cycle_time => time from one foot touch down to the next touch down
#  swing_time => time spent between lift off and touch down
#  oneleg_time => time when a single virtual leg is on the ground
#  oneleg_dist => distance that a single leg travels when in contact with the ground
#  twoleg_time => time when two virtual legs are on the ground
#  twoleg_dist => distance that two legs travel when in contact with the ground
#  flight_time => time when no virtual legs are on the ground
#  flight_dist => distance traveled with no legs on ground
#  travel_dist => total available distance for the leg to travel on ground
#
#
# Relations that must be maintained:
#
#  0.5 * travel_dist >= 0.5 * oneleg_dist + twoleg_dist
#  oneleg_dist = oneleg_time * speed
#  twoleg_dist = twoleg_time * speed
#  cycle_time = oneleg_time + max(flight_time, twoleg_time)
#  min_swing_time <= swing_time <= max_swing_time
#  twoleg_time <= max_twoleg_time
#
#
# The phases of a single cycle look like:
#
#  1. Touchdown: The touchdown leg lands at 0.5 * oneleg_dist + twoleg_dist
#  2. Two leg stance: Both legs continue moving
#  3. Liftoff: When the touchdown leg reaches 0.5 * oneleg_dist, the rear leg lifts off
#  4. Swing: During the swing phase, the in-contact leg moves to -0.5 oneleg_dist
#
# If twoleg_dist == 0.0 && flight_time >= 0.0, then the cycle looks like:

#  1. Touchdown: The touchdown leg lands at 0.5 * oneleg_dist, the
#     other leg is still in flight.
#  2. Liftoff: The down leg travels to -0.5 * oneleg_dist and lifts off.
#  3. Flight: The machine continues flying at speed with no legs in
#     contact, after flight_time, the furthest forward leg touches down.

# cycle_time = oneleg_time + max(flight_time, twoleg_time)
# cycle_dist = oneleg_dist + max(flight_dist, twoleg_dist)
# speed = cycle_dist / cycle_time

# The gait space is broken up into:
#
#  A (slowest): Limited by max_twoleg_time
#  B (medium): 0 < twoleg_time < max_twoleg_time
#  C (fast): 0 < flight_time < max_flight_time
#  D (fastest): min_swing_time <= swing_time < max_swing_time,
#           flight_time == max_flight_time
#  E (max speed): swing_time == min_swing_time, flight_time == max_flight_time

# For phase A:
#
#  oneleg_time = max_swing_time
#  twoleg_time = max_twoleg_time

#  0.5 * travel_dist >= 0.5 * oneleg_dist + twoleg_dist
#  twoleg_dist <= 0.5 * travel_dist - 0.5 * oneleg_dist
#  max_twoleg_time * speed <= 0.5 * travel_dist - 0.5 * max_swing_time * speed
#  max_twoleg_time * speed + 0.5 * max_swing_time * speed <= 0.5 * travel_dist
#  speed * (max_twoleg_time + 0.5 * max_swing_time) <= 0.5 * travel_dist
#  speed < (0.5 * travel_dist) / (max_twoleg_time + 0.5 * max_swing_time)


# Phase B ends when twoleg_time == 0.0
#
# speed <= travel_dist / max_swing_time
#
# 0.5 * travel_dist = 0.5 * oneleg_dist + twoleg_dist
# twoleg_time * speed = 0.5 * travel_dist - 0.5 * max_swing_time * speed
# twoleg_time = 0.5 * travel_dist / speed - 0.5 * max_swing_time

# Phase C ends when flight_time == max_flight_time

#
# swing_time = max_swing_time
# twoleg_time = 0
# oneleg_time = swing_time - 2 * flight_time
# speed = travel_dist / oneleg_time = travel_dist / (swing_time - 2 * flight_time)
#
# speed * (swing_time - 2 * flight_time) = travel_dist
# 2 * speed * flight_time = speed * swing_time - travel_dist
# flight_time = swing_time / 2 - travel_dist / 2 * speed
#
# speed = travel_dist / (max_swing_time - 2 * max_flight_time)
#

# Phase D ends at the max speed

# flight_time = max_flight_time
# twoleg_time = 0
# speed = travel_dist / (swing_time - 2 * flight_time)
# speed * swing_time - 2 * speed * flight_time = travel_dist
# swing_time = travel_dist / speed + 2 * flight_time


# Phase E is at maximum speed:
#
#  swing_time = min_swing_time
#  flight_time = max_flight_time
#  twoleg_time = 0
#
#  swing_time = 2 * flight_time + oneleg_time
#  oneleg_time = swing_time - 2 * flight_time
#

############
# When to lift
#
#  If flight_time <= 0.0 we lift when the opposing leg is at 0.5 * swing_time

#  If flight_time >= 0.0 we lift when the current leg is at -0.5 * oneleg_time
#    * Weight this based on how far the next leg is in its swing
#      phase.  Split the difference between getting the correct flight
#      time and getting the correct lift point.


###########
# Where to place
#
#  If flight_time <= 0.0, we place at 0.5 * swing_time + twoleg_time
#  If flight_time > 0.0, we place at 0.5 * oneleg_time

import matplotlib
import matplotlib.pyplot as plt
import numpy as np


MAX_TRAVEL_DIST = 0.25
MAX_SWING_TIME = 0.20
MAX_TWOLEG_TIME = 0.35
MAX_FLIGHT_TIME = 0.02
MIN_SWING_TIME = 0.17
MAX_SPEED = MAX_TRAVEL_DIST / (MIN_SWING_TIME - 2 * MAX_FLIGHT_TIME)



class Params:
    swing_time = 0.0
    oneleg_time = 0.0
    twoleg_time = 0.0
    flight_time = 0.0
    speed = 0.0



def calc_params(speed):
    r = Params()

    if speed < ((0.5 * MAX_TRAVEL_DIST) / (MAX_TWOLEG_TIME + 0.5 * MAX_SWING_TIME)):
        # Phase A:
        r.swing_time = r.oneleg_time = MAX_SWING_TIME
        r.twoleg_time = MAX_TWOLEG_TIME
        r.speed = speed
        return r

    if speed < (MAX_TRAVEL_DIST / MAX_SWING_TIME):
        # Phase B
        r.swing_time = r.oneleg_time = MAX_SWING_TIME
        r.twoleg_time = 0.5 * MAX_TRAVEL_DIST / speed - 0.5 * MAX_SWING_TIME
        r.speed = speed

        return r

    if speed < (MAX_TRAVEL_DIST / (MAX_SWING_TIME - 2 * MAX_FLIGHT_TIME)):
        # Phase C
        r.swing_time = MAX_SWING_TIME
        r.twoleg_time = 0
        r.flight_time = 0.5 * r.swing_time - MAX_TRAVEL_DIST / (2 * speed)
        r.oneleg_time = r.swing_time - 2 * r.flight_time
        r.speed = speed
        return r

    if speed < MAX_SPEED:
        # Phase D
        r.flight_time = MAX_FLIGHT_TIME
        r.twoleg_time = 0
        r.swing_time = MAX_TRAVEL_DIST / speed + 2 * r.flight_time
        r.oneleg_time = r.swing_time - 2 * r.flight_time
        r.speed = speed
        return r

    r.swing_time = MIN_SWING_TIME
    r.flight_time = MAX_FLIGHT_TIME
    r.twoleg_time = 0
    r.oneleg_time = r.swing_time - 2 * r.flight_time
    r.speed = MAX_SPEED

    return r


def extra(p):
    p.frequency = 0.5 / (p.oneleg_time + max(p.flight_time, p.twoleg_time))

    if p.flight_time <= 0.0:
        p.lift_pos = (-0.5 * p.swing_time) * p.speed
        p.place_pos = (0.5 * p.swing_time + p.twoleg_time) * p.speed
    else:
        p.lift_pos = (-0.5 * p.oneleg_time) * p.speed
        p.place_pos = (0.5 * p.oneleg_time) * p.speed

    return p


def main():
    speeds = np.arange(0, MAX_SPEED + 0.2, 0.01)
    p = [extra(calc_params(x)) for x in speeds]

    fig, axs = plt.subplots(4)
    axs[0].plot(speeds, [x.speed for x in p], label='speed')
    axs[0].set(ylabel='mps')
    axs[1].plot(speeds, [x.swing_time for x in p], label='swing')
    axs[1].plot(speeds, [x.oneleg_time for x in p], label='oneleg')
    axs[1].plot(speeds, [x.twoleg_time for x in p], label='twoleg')
    axs[1].plot(speeds, [x.flight_time for x in p], label='flight')
    axs[1].set(ylabel='s')
    axs[2].plot(speeds, [x.lift_pos for x in p], label='liftpos')
    axs[2].plot(speeds, [x.place_pos for x in p], label='placepos')
    axs[2].set(ylabel='m')
    axs[3].plot(speeds, [x.frequency for x in p], label='frequency')
    axs[3].set(xlabel='mps')
    axs[3].set(ylabel='Hz')

    for ax in axs:
        ax.legend()

    axs[0].set_title('Trot Gait vs Speed')

    plt.show()


if __name__ == '__main__':
    main()
