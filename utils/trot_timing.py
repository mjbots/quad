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

import matplotlib
import matplotlib.pyplot as plt
import numpy as np


MAX_TRAVEL_DIST = 0.25
MAX_SWING_TIME = 0.20
MAX_STANCE_TIME = 0.35
MAX_FLIGHT_TIME = 0.05
MIN_SWING_TIME = 0.15


class Params:
    swing_time = 0.0
    stance_time = 0.0
    speed = 0.0



def calc_params(speed):
    r = Params()

    if speed == 0.0:
        r.swing_time = MAX_SWING_TIME
        r.stance_time = MAX_STANCE_TIME
        r.speed = speed
        return r

    # swing_time + half stance
    st_hs = MAX_TRAVEL_DIST / speed

    if st_hs > (MAX_SWING_TIME + 0.5 * MAX_STANCE_TIME):
        # We are at the maximal values.
        r.swing_time = MAX_SWING_TIME
        r.stance_time = MAX_STANCE_TIME
        r.speed = speed
        return r

    if st_hs > MAX_SWING_TIME - 0.5 * MAX_FLIGHT_TIME:
        # We don't have to adjust our swing time yet.
        r.swing_time = MAX_SWING_TIME
        r.stance_time = 2 * (st_hs - r.swing_time)
        r.speed = speed
        return r

    if True:
        # We haven't maxed out our speed yet, but are maintaining our
        # ratio of flight time to swing time.
        r.stance_time = -MAX_FLIGHT_TIME
        r.swing_time = st_hs - 0.5 * r.stance_time
        r.speed = speed
        if r.swing_time >= MIN_SWING_TIME:
            return r

    # We have hit our maximum speed.
    r.stance_time = -MAX_FLIGHT_TIME
    r.swing_time = MIN_SWING_TIME
    r.speed = MAX_TRAVEL_DIST / (r.swing_time + 0.5 * r.stance_time)
    return r


def extra(p):
    p.frequency = 0.5 / (p.swing_time + 0.5 * p.stance_time)
    p.dist_per_step = 0.5 * p.speed / p.frequency
    return p


def main():
    speeds = np.arange(0, 2.5, 0.01)
    p = [extra(calc_params(x)) for x in speeds]

    fig, axs = plt.subplots(3)
    axs[0].plot(speeds, [x.speed for x in p], label='speed')
    axs[1].plot(speeds, [x.swing_time for x in p], label='swing')
    axs[1].plot(speeds, [x.stance_time for x in p], label='stance')
    axs[1].plot(speeds, [x.dist_per_step for x in p], label='step dist')
    axs[2].plot(speeds, [x.frequency for x in p], label='frequency')
    axs[2].set(xlabel='mps')

    axs[0].legend()
    axs[1].legend()
    axs[2].legend()

    axs[0].set_title('Trot Gait vs Speed')

    plt.show()


if __name__ == '__main__':
    main()
