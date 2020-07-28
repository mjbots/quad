#!/usr/bin/python3

import matplotlib
import matplotlib.pylab as pylab
import matplotlib.pyplot as plt

data_105kv = [
    # Phase A, Input Power W, Output Torque Nm
    (0., 1.5, 0.),
    (2.29, 2.79, 1.06),
    (4.63, 5.81, 2.13),
    (6.42, 8.59, 2.71),
    (7.19, 11.36, 2.98),
    (8.99, 16.49, 3.7),
    (12.57, 29.02, 4.92),
    (15, 42.8, 5.86),
    (17.4, 57.77, 6.75),
    (20.02, 78.21, 7.65),
    (24.36, 114.26, 8.8),
    (26.58, 144.1, 9.45),
    (29.64, 187.44, 10.2),
    (33.75, 262.16, 11.49),
    (38.65, 383.14, 12.9),
    (47.17, 484.63, 15.2),
    (52.2, None, 15.87),
    (58.8, None, 18.2),
]

data_135kv = [
    (0., 1.5, 0.),
    (3.18, 2.64, 0.95),
    (7.43, 7.32, 2.61),
    (13.3, 21.68, 4.2),
    (21.76, 54.71, 6.92),
    (26.38, 78.85, 7.81),
    (36.17, 154.28, 10.7),
    (44.61, 241.68, 11.77),
    (55.63, 425.5, 14.35),
    (58.98, 420.0, 14.72),
    (61.63, 401.76, 14.99),
    (79.7, None, 16.82),
    (99.89, None, 18.05),
]

def show_plot(index, data, title, xlabel=None):
    ax1 = plt.subplot(index)
    ax1.plot([x[0] for x in data], [x[2] for x in data],
             color='b', label='output torque')
    ax1.set_ylabel('Nm')

    if xlabel:
        ax1.set_xlabel('phase current A')
    ax1.legend()

    ax1.grid(which='major')
    ax1.set_yticks(ticks=[0, 3, 6, 9, 12, 15, 18])
    ax1.set_title(title)


    ax2 = ax1.twinx()
    ax2.plot([x[0] for x in data], [x[1] for x in data],
             color='r', label='input power')
    ax2.set_ylabel('W')
    ax2.set_yscale('log')
    ax2.legend(loc='lower right')


def plot_torques():
    show_plot("212", data_105kv, "105 Kv stator", xlabel=True)
    show_plot("211", data_135kv, "135 Kv stator", xlabel=False)

    plt.show()


def plot_power_torque():
    ax1 = plt.subplot()
    ax1.plot([x[1] for x in data_135kv], [x[2] for x in data_135kv], label='135Kv')
    ax1.plot([x[1] for x in data_105kv], [x[2] for x in data_105kv], label='105Kv')

    ax1.set_xscale("log")
    ax1.set_xlabel("input power W")
    ax1.set_ylabel("output torque Nm")
    ax1.grid()
    ax1.legend()
    ax1.set_title("Output Torque vs Input Power")

    plt.show()


plot_power_torque()
