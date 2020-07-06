// Copyright 2019-2020 Josh Pieper, jjp@pobox.com.  All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

const CMD_MAX_RATE_X = 0.5;
const CMD_MAX_RATE_Y = 0.2;
const CMD_MAX_RATE_Z = 1.0;

const TRANSLATION_EPSILON = 0.025;
const ROTATION_EPSILON = Math.PI * 7.0 / 180.0;

const getElement = (v) => document.getElementById(v);
const iota = (v) => Array.from((new Array(v)).keys());

class Joystick {
  static BUTTON_START = 9;
  static BUTTON_SELECT = 8;
  static BUTTON_2 = 0;
  static BUTTON_3 = 1;
  static BUTTON_1 = 2;
  static BUTTON_4 = 3;
  static BUTTON_SHOULDER_LT = 4;
  static BUTTON_SHOULDER_RT = 5;
  static BUTTON_SHOULDER_LB = 6;
  static BUTTON_SHOULDER_RB = 7;
  static BUTTON_LEFT_JOY = 10;
  static BUTTON_RIGHT_JOY = 11;
  static BUTTON_HAT_UP = 12;
  static BUTTON_HAT_DOWN = 13;
  static BUTTON_HAT_LEFT = 14;
  static BUTTON_HAT_RIGHT = 15;

  static NUM_BUTTONS = 16;

  static AXES_LEFT_X = 0;
  static AXES_LEFT_Y = 1;
  static AXES_RIGHT_X = 2;
  static AXES_RIGHT_Y = 3;

  constructor() {
    this._zero();
  }

  _zero() {
    this._down = iota(Joystick.NUM_BUTTONS).map(x => false);
    this._pressed = iota(Joystick.NUM_BUTTONS).map(x => false);
    this._axes = iota(4).map(x => 0.0);
  }

  /// Call once per update, this ensures that 'pressed' and 'released'
  /// are correct.
  update() {
    const gp = this._getJoystick();
    if (!gp) {
      // No joystick this time.  Set all of our values to all buttons
      // off.
      this._zero();
      return;
    }

    const old_down = [...this._down];
    this._down = iota(Joystick.NUM_BUTTONS).map(x => gp.buttons[x].pressed);
    this._pressed = iota(Joystick.NUM_BUTTONS).map(
      x => this._down[x] && old_down[x]);

    this._axes = [...gp.axes];
  }

  down(button) {
    return this._down[button];
  }

  pressed(button) {
    return this._pressed[button];
  }

  axis(num) {
    return this._axes[num];
  }

  _getJoystick() {
    const gps = navigator.getGamepads();
    for (const gp of gps) {
      if (gp) { return gp; }
    }
    return null;
  }
};

class Application {
  constructor() {
    this._websocket = null;
    this._mode = "stop";
    this._state = null;
    this._joystick = new Joystick();

    // Fill in our constant values.
    document.getElementById('chart_rot_min').innerHTML =
      (-180.0 * CMD_MAX_RATE_Z / Math.PI).toFixed(0);
    document.getElementById('chart_rot_max').innerHTML =
      (180.0 * CMD_MAX_RATE_Z / Math.PI).toFixed(0);

    getElement('chart_x_min').innerHTML = (-CMD_MAX_RATE_X).toFixed(2);
    getElement('chart_x_max').innerHTML = CMD_MAX_RATE_X.toFixed(2);
    getElement('chart_y_min').innerHTML = (-CMD_MAX_RATE_Y).toFixed(2);
    getElement('chart_y_max').innerHTML = CMD_MAX_RATE_Y.toFixed(2);

    const checkbox = getElement('mode_expander');
    getElement('text').addEventListener('click', () => {
      checkbox.checked = !checkbox.checked;
    });

    for (const mode_label of document.getElementsByClassName("mode_check")) {
      mode_label.addEventListener('input', () => {
        checkbox.checked = false;
        this._updateMode();
      });
    }
  }

  start() {
    setInterval(() => this._handleTimer(), 100);
    this._openWebsocket();
  }

  _updateMode() {
    this._mode = (() => {
      for (const mode_check of document.getElementsByClassName("mode_check")) {
        if (mode_check.checked) { return mode_check.value; }
      }
      throw "Unknown mode";
    })();

    getElement('mode_text').innerHTML = this._mode;
  }

  _openWebsocket() {
    const location = window.location;
    this._websocket = new WebSocket("ws://" + location.host + "/control");
    this._websocket.addEventListener(
      'message', (e) => { this._handleWebsocketMessage(e); });
    this._websocket.addEventListener(
      'close', () => { this._handleWebsocketClose(); });
  }

  _handleWebsocketMessage(e) {
    this._state = JSON.parse(e.data)

    this._updateState();
  }

  _handleWebsocketClose() {
    this._openWebsocket();
  }

  _handleTimer() {
    this._joystick.update();
    this._updateState();
    this._sendCommand();
  }

  _sendCommand() {
    const v_R = [
      -this._joystick.axis(Joystick.AXES_LEFT_Y) * CMD_MAX_RATE_X,
      this._joystick.axis(Joystick.AXES_LEFT_X) * CMD_MAX_RATE_Y,
      0.0
    ];
    const w_R = [
      0.0,
      0.0,
      this._joystick.axis(Joystick.AXES_RIGHT_X) * CMD_MAX_RATE_Z,
    ];

    {
      const desired_rot_cmd = getElement('desired_rot_cmd');
      const scaled_w =
            Math.max(-1.0, Math.min(1.0, w_R[2] / CMD_MAX_RATE_Z));
      desired_rot_cmd.setAttribute('x', `${scaled_w * 38 + 48}%`);

      const desired_trans_cmd = getElement('desired_trans_cmd');
      const scaled_x =
            Math.max(-1.0, Math.min(1.0, v_R[0] / CMD_MAX_RATE_X));
      const scaled_y =
            Math.max(-1.0, Math.min(1.0, v_R[1] / CMD_MAX_RATE_Y));
      desired_trans_cmd.setAttribute('x', `${scaled_y * 38 + 48}%`);
      desired_trans_cmd.setAttribute('y', `${-scaled_x * 38 + 48}%`);
    }

    const v_norm = Math.sqrt(v_R[0] * v_R[0] + v_R[1] * v_R[1]);
    const w_norm = Math.sqrt(w_R[2] * w_R[2]);
    const movement_commanded = (
      v_norm > TRANSLATION_EPSILON ||
        w_norm > ROTATION_EPSILON);
    const force_step = getElement("always_step").checked;

    let command = {
      "command" : {
        "mode" : (() => {
          if (this._mode == "stop") { return "zero_velocity" ;}
          if (this._mode == "idle") { return "rest"; }

          if (!movement_commanded && !force_step &&
              (this._mode == "walk" ||
               this._mode == "pronk")) {
            return "rest";
          }
          if (this._mode == "walk") { return "walk"; }
          if (this._mode == "pronk") { return "jump"; }
          return "zero_velocity";
        })(),
      },
    };

    command["command"]["v_R"] = v_R;
    command["command"]["w_R"] = w_R;

    const command_string = JSON.stringify(command);
    getElement('current_json_command').innerHTML = command_string;

    if (this._websocket.readyState == WebSocket.OPEN) {
      this._websocket.send(command_string)
    }
  }

  _updateState() {
    const connected_container = getElement('connected_state_container');
    const connected_text = getElement('connected_state');

    if (this._websocket.readyState == WebSocket.OPEN) {
      connected_text.innerHTML = 'CONNECTED';
      connected_container.style.backgroundColor = "#00e000";
    } else {
      connected_text.innerHTML = 'NONE';
      connected_container.style.backgroundColor = "#e00000";
    }

    if (this._state === null) {
      return;
    }

    // Primary mode
    {
      const mode_sub = getElement('mode_sub');
      mode_sub.innerHTML = '(' + (() => {
        const cur = this._state.mode;
        if (cur == "zero_velocity") { return "damped"; }
        if (cur == "rest") { return "idle"; }
        return cur;
      })() + ')';
    }

    // Movement command.
    {
      const desired_R = this._state.state.robot.desired_R;
      const desired_rot_act = getElement('desired_rot_act');
      const scaled_w =
            Math.max(-1.0, Math.min(1.0, desired_R.w[2] / CMD_MAX_RATE_Z));
      desired_rot_act.setAttribute('x', `${scaled_w * 38 + 49}%`);

      const desired_trans_act = getElement('desired_trans_act');
      const scaled_x =
            Math.max(-1.0, Math.min(1.0, desired_R.v[0] / CMD_MAX_RATE_X));
      const scaled_y =
            Math.max(-1.0, Math.min(1.0, desired_R.v[1] / CMD_MAX_RATE_Y));
      desired_trans_act.setAttribute('x', `${scaled_y * 38 + 49}%`);
      desired_trans_act.setAttribute('y', `${-scaled_x * 38 + 49}%`);
    }

    // Battery
    {
      const battery_level = getElement('battery_level');
      const battery_text = getElement('battery_text');

      const voltage = this._state.state.robot.voltage;
      battery_text.innerHTML = `${voltage.toFixed(1)}`;
      const percentage =
            100.0 * Math.min(1.0, Math.max(0.0, (voltage - 16.0) / (23.0 - 16.0)));
      battery_level.style.height = `${percentage}%`;
      if (percentage < 30.0) {
        battery_level.style.backgroundColor = "#d00000";
      } else if (percentage < 50.0) {
        battery_level.style.backgroundColor = "#d0d000";
      } else {
        battery_level.style.backgroundColor = "#008000";
      }
    }
  }
};

const app = async () => {
  const app = new Application();
  app.start();
};

document.addEventListener("DOMContentLoaded", app)
