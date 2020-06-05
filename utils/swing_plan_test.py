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

import enum
import matplotlib
import matplotlib.animation
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import sys


SWING_PERIOD_S = 0.5
STANCE_PERIOD_S = 1.0
MAX_ACCELERATION = 1.0
MIN_STANCE_TIME_S = 0.1


class LiftState(enum.Enum):
    STANCE = 0
    SWING = 1


class Leg:
    state = LiftState.STANCE
    pos = 0

    swing_start = None
    swing_time_elapsed = None
    swing_target = None

    def __init__(self, pos):
        self.pos = pos


    def __repr__(self):
        result = f"{self.state} pos={self.pos:6.3f}"
        if self.swing_start:
            result += f" swing({self.swing_start:6.3f}, {self.swing_target:6.3f})"
        return result


class Robot:
    def __init__(self):
        self.legs = [Leg(-.1), Leg(.1)]
        self.velocity = 0.0
        self.velocity_command = 0.0
        self.next_step_leg_id = 0
        self.start_stance_s = 0.0
        self.last_nonzero_velocity = 0.0

        # Just for logging
        self.time = 0.0
        self.first_half_time = 0.0
        self.second_half_time = 0.0
        self.left_time = 0.0
        self.right_time = 0.0

    def __repr__(self):
        return f"v={self.velocity:.2f} l={self.legs}  h={self.first_half_time:.3f}/{self.second_half_time:.3f}"

    def advance(self, dt_s):
        self.time += dt_s
        any_swing = any([ x.state == LiftState.SWING for x in self.legs])
        all_stance = not any_swing

        if all_stance:
            if not self.start_stance_s:
                self.start_stance_s = self.time
            max_dv = MAX_ACCELERATION * dt_s
            dv = max(-max_dv, min(max_dv, self.velocity_command - self.velocity))
            if self.velocity != 0.0:
                self.last_nonzero_velocity = self.velocity
            self.velocity += dv
            if self.velocity != 0.0:
                change = self.velocity * self.last_nonzero_velocity
                if change < 0.0:
                    self.next_step_leg_id = (self.next_step_leg_id + 1) % 2
        else:
            self.start_stance_s = None
            stance_leg = [x for x in self.legs if x.state == LiftState.STANCE][0]
            if stance_leg.pos > 0.0:
                self.left_time += dt_s
            else:
                self.right_time += dt_s

        # First, move all the legs according to their current state.
        for leg in self.legs:
            if leg.state == LiftState.STANCE:
                leg.pos -= self.velocity * dt_s
            if leg.state == LiftState.SWING:
                leg.swing_time_elapsed += dt_s
                leg.pos = ((leg.swing_target - leg.swing_start) *
                           (leg.swing_time_elapsed / SWING_PERIOD_S) +
                           leg.swing_start)
                if leg.swing_time_elapsed >= SWING_PERIOD_S:
                    leg.state = LiftState.STANCE
                    leg.pos = leg.swing_target
                    leg.swing_start = None
                    leg.swing_time_elapsed = None
                    leg.swing_target = None

        if all_stance:
            # We need to figure out if we are ready to lift.

            next_stance_leg_id = (self.next_step_leg_id + 1) % 2
            next_stance_leg = self.legs[next_stance_leg_id]
            next_step_leg = self.legs[self.next_step_leg_id]

            if self.velocity == 0.0:
                self.first_half_time = 0.0
            else:
                self.first_half_time = ((next_stance_leg.pos - 0.0) /
                                        self.velocity)
            self.second_half_time = (SWING_PERIOD_S - self.first_half_time)

            stance_time = self.time - self.start_stance_s

            if (self.first_half_time < self.second_half_time and
                stance_time > MIN_STANCE_TIME_S):

                self.left_time = 0.0
                self.right_time = 0.0

                # Yes, we are ready to begin a lift.
                next_step_leg.swing_start = next_step_leg.pos
                next_step_leg.swing_time_elapsed = 0

                next_step_leg.swing_target = (
                    # This is the minimum amount of time we need if
                    # there would be zero time with both legs in
                    # stance pose.
                    0.5 * self.velocity * SWING_PERIOD_S +
                    # And this extra amount gives us margin to have
                    # some all stance period.
                    0.5 * self.velocity * STANCE_PERIOD_S)
                next_step_leg.state = LiftState.SWING

                self.next_step_leg_id = next_stance_leg_id


def main():
    robot = Robot()
    robot.velocity_command = 0.5
    robot.velocity = 0.0
    time = 0

    fig, ax = plt.subplots()
    ax.set_ylim(-.5, 2.5)
    ax.set_xlim(-1, 1)
    com, = plt.plot([0],[2.0], 'bo', label="CoM")
    leg1, = plt.plot([], [], 'ro', label="foot1")
    leg2, = plt.plot([], [], 'go', label="foot2")
    text = plt.text(-0.9, 1.0, '')
    plt.legend()

    def update(frame):
        if robot.time > 1.0:
            robot.velocity_command = -1.0
        robot.advance(0.02)

        def height(leg):
            if leg.state == LiftState.STANCE:
                return 0.0
            return 0.5
        leg1.set_data([robot.legs[0].pos], [height(robot.legs[0])])
        leg2.set_data([robot.legs[1].pos], [height(robot.legs[1])])

        text.set_text(
            ("velocity={:.2f}\nswing_time:{:.2f}\n" +
             "left time:{:.2f}\nright time:{:.2f}").format(
                 robot.velocity,
                 SWING_PERIOD_S,
                 robot.left_time,
                 robot.right_time,
                ))

        print(f"{robot.time:6.3f} {robot}")

        return [com, leg1, leg2, text]

    animation = FuncAnimation(fig, update, blit=True, interval=30,
                              frames=np.arange(0, 160))
    if len(sys.argv) > 1:
        animation.save(sys.argv[1],
                       writer=matplotlib.animation.ImageMagickWriter(fps=20))
    else:
        plt.show()


if __name__ == '__main__':
    main()
