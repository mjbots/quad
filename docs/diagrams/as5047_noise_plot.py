#!/usr/bin/env python

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
import sys

data = [[float(y) for y in line.strip().split(',')]
        for line in (open(sys.argv[1]).readlines()[1:])
        if line.strip() != '']

ax = plt.subplot()
ax.plot([1.0 / x[0] for x in data], [x[1]/4. for x in data])
ax.set_xlim(40000, 10)
ax.set_xscale("log")
ax.set_xlabel("Frequency Hz")
ax.set_ylabel("LSB")
ax.set_title("AS5047U Position Noise vs Frequency")

plt.show()
