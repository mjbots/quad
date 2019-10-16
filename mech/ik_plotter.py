#!/usr/bin/python3 -B

# Copyright 2019 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

"""This tool generates plots used to compare two different IK
implementations."""

import matplotlib.pyplot as plt
import numpy as np
import sys

def read_data(filename):
    data_file = open(filename)
    xvals = [float(x) for x in data_file.readline().split(' ')]
    yvals = [float(y) for y in data_file.readline().split(' ')]

    data = []
    for x in range(len(xvals)):
        for y in range(len(yvals)):
            fields = [float(x) for x in data_file.readline().split(' ')]
            while len(data) < 3:
                data.append(np.zeros((len(xvals), len(yvals))))
            for i, val in enumerate(fields[0:3]):
                data[i][x, y] = val

    return data, xvals, yvals


def main():
    old, xvals, yvals = read_data(sys.argv[1])
    new, xvals, yvals = read_data(sys.argv[2])

    # zmin = min([x.min() for x in data]) - 0.01
    # zmax = max([x.max() for x in data]) + 0.01

    zmin = -0.50
    zmax = 0.90

    levels = [(zmin + (zmax - zmin) / 20 * i) for i in range(21)]

    fig = plt.figure()
    fig.subplots_adjust(right=0.85, left=0.1)
    plt.suptitle('Joint torque over end-effector x/y position for 10N down force')

    for i in range(3):
        plt.subplot(2, 3, 1 + i)
        plt.title(["Shoulder", "Femur", "Tibia"][i])

        cs = plt.contourf(xvals, yvals, old[i], levels)

    for i in range(3):
        plt.subplot(2, 3, 1 + i + 3)
        plt.title(["Shoulder", "Femur", "Tibia"][i])

        cs = plt.contourf(xvals, yvals, new[i], levels)

    cbar_ax = fig.add_axes([0.90, 0.15, 0.03, 0.7])
    cb = plt.colorbar(cs, cax=cbar_ax)
    cb.ax.set_ylabel("Nm")

    fig.text(0.02, 0.3, "New\nIK")
    fig.text(0.02, 0.7, "Old\nIK")
    plt.show()


if __name__ == '__main__':
    main()
