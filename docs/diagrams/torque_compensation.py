#!/usr/bin/python3

import matplotlib
import matplotlib.pyplot as plt


DATA = [
    (1.012, 1.033, 3.166),
    (2.496, 2.619, 7.831),
    (3.844, 3.698, 12.144),
    (4.924, 4.976, 15.53),
    (5.733, 5.506, 17.985),
    (6.812, 6.452, 20.556),
    (7.757, 7.425, 24.214),
    (8.836, 8.592, 27.998),
    (9.375, 9.125, 30.32),
    (10.455, 9.835, 35.043),
    (11.669, 11.417, 41.542),
    (12.883, 12.956, 48.286),
    (13.693, 13.656, 53.627),
    (15.311, 15.407, 65.86),
    (15.986, 16.071, 71.987),
    (16.93, 16.49, 81.1),
    (17.2, 16.89, 83.73),
]


ax1 = plt.subplot()

ax1.plot([x[0] for x in DATA], [x[1] for x in DATA], color='r',
         label="Measured Torque")
ax1.plot([x[0] for x in DATA], [x[0] for x in DATA], color='b',
         linewidth=1, linestyle='dashed',
         label="Ideal Torque")
ax1.set_ylabel("Nm")
ax1.set_xlabel("Command Torque Nm")
ax1.set_ylim(0, 26)
ax1.grid()
ax1.legend(loc='lower right')

ax2 = ax1.twinx()
ax2.plot([x[0] for x in DATA], [x[2] for x in DATA], color='g',
         label="Phase Current")
ax2.set_ylabel("A")
ax2.legend(loc='upper right')


plt.show()
