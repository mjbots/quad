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

import matplotlib.pyplot as plt
import numpy as np


class Bezier:
    def __init__(self, start, end):
        self._start = start
        self._end = end
        self._delta = end - start

    def position(self, phase):
        bezier = phase ** 3 + 3.0 * ((phase ** 2) * (1.0 - phase))
        return self._start + bezier * self._delta

    def velocity(self, phase):
        bezier = 6 * phase * (1.0 - phase)
        return bezier * self._delta

    def acceleration(self, phase):
        bezier = 6 - 12 * phase
        return bezier * self._delta


def main():
    tvals = np.arange(0, 1, 0.005)

    initial_velocity = np.array([-50, 10])
    initial_pos = np.array([1, 2])
    target_pos = np.array([6, 3])
    target_vel = np.array([-40, 20])

    lift_fraction = 0.10

    phase1 = np.array([x for x in tvals if x < lift_fraction])
    phase1_accel = -initial_velocity * (1. / lift_fraction)
    phase1_position = np.array(
        [initial_pos +
         x * initial_velocity +
         0.5 * phase1_accel * x ** 2 for x in phase1])
    phase1_velocity = np.array(
        [initial_velocity * (1.0 - x / lift_fraction)
         for x in phase1])

    xymove = Bezier(initial_pos + 0.5 * lift_fraction * initial_velocity,
                    target_pos)
    print(xymove._start, xymove._end)
    phase2 = np.array([x for x in tvals if x >= lift_fraction and x < (1.0 - lift_fraction)])
    phase2_position = np.array([xymove.position((x - lift_fraction) / (1 - 2 * lift_fraction)) for x in phase2])
    phase2_velocity = np.array([xymove.velocity((x - lift_fraction) / (1 - 2 * lift_fraction)) for x in phase2])


    phase3 = np.array([x for x in tvals if x >= (1 - lift_fraction)])
    phase3_accel = target_vel * (1.0 / lift_fraction)
    phase3_position = np.array(
        [target_pos + 0.5 * phase3_accel * (x - (1 - lift_fraction)) ** 2 for x in phase3])
    phase3_velocity = np.array([(x - (1 - lift_fraction)) / lift_fraction * target_vel for x in phase3])

    xvals = tvals
    bezier_z = Bezier(0, 1)
    zpos = np.array([bezier_z.position(2 * x)
                     if x < 0.5 else
                     bezier_z.position(2 * (1 - x))
                     for x in xvals])
    zvel = np.array([bezier_z.velocity(2 * x)
                     if x < 0.5 else
                     -bezier_z.velocity(2 * (1 - x))
                     for x in xvals])

    if False:
        plt.plot(xvals, zpos)
        plt.plot(xvals, zvel)
        plt.show()

    if False:
        S = 6
        plt.plot(phase1_position[:,0], phase1_position[:,1], color='blue')
        plt.quiver(phase1_position[:,0][::S], phase1_position[:,1][::S],
                   phase1_velocity[:,0][::S], phase1_velocity[:,1][::S],
                   color='red', label='phase 1')
        plt.plot(phase2_position[:,0], phase2_position[:,1], color='blue')
        plt.quiver(phase2_position[:,0][::S], phase2_position[:,1][::S],
                   phase2_velocity[:,0][::S], phase2_velocity[:,1][::S],
                   color='green', label='phase 2')
        plt.plot(phase3_position[:,0], phase3_position[:,1], color='blue')
        plt.quiver(phase3_position[:,0][::S], phase3_position[:,1][::S],
                   phase3_velocity[:,0][::S], phase3_velocity[:,1][::S],
                   color='brown', label='phase 3')

    xypos = np.concatenate((phase1_position, phase2_position, phase3_position))
    xyvel = np.concatenate((phase1_velocity, phase2_velocity, phase3_velocity))

    if True:
        S = 4
        plt.plot(xypos[:,0], zpos, color='blue')
        def plot(start, end, **kwargs):
            plt.quiver(xypos[:,0][start:end][::S], zpos[start:end][::S],
                       xyvel[:,0][start:end][::S], zvel[start:end][::S],
                       scale=500,
                       **kwargs)
        plot(0, len(phase1_position), color='red', label='phase 1')
        plot(len(phase1_position), len(phase1_position) + len(phase2_position), color='green', label = 'phase 2')
        plot(len(phase1_position) + len(phase2_position), -1, color='orange', label = 'phase 3')

    plt.axes().set_aspect('equal', 'datalim')
    plt.legend()
    plt.show()


if __name__ == '__main__':
    main()
