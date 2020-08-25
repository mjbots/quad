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

const CMD_MAX_RATE_X_FORWARD = 1.0;
const CMD_MAX_RATE_X_REVERSE = 0.5;
const CMD_MAX_RATE_Y = 0.2;
const CMD_MAX_RATE_Z = Math.PI * 60 / 180.0;


const CMD_MAX_POSE_YAW = Math.PI * 20 / 180.0;
const CMD_MAX_POSE_PITCH = Math.PI * 11 / 180.0;

const TRANSLATION_EPSILON = 0.025;
const ROTATION_EPSILON = Math.PI * 7.0 / 180.0;

const getElement = (v) => document.getElementById(v);
const iota = (v) => Array.from((new Array(v)).keys());

function _isUndefined(v) {
  return typeof v === "undefined";
}

function normalizeQuaternion(q) {
  const norm = Math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  return [
    q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm
  ];
};

function makeUnitQuaternion() {
  return [1, 0, 0, 0];
};

function quaternionMultiply(lhs, rhs) {
  return normalizeQuaternion([
    lhs[0] * rhs[0] - lhs[1] * rhs[1] - lhs[2] * rhs[2] - lhs[3] * rhs[3],
    lhs[0] * rhs[1] + lhs[1] * rhs[0] + lhs[2] * rhs[3] - lhs[3] * rhs[2],
    lhs[0] * rhs[2] - lhs[1] * rhs[3] + lhs[2] * rhs[0] + lhs[3] * rhs[1],
    lhs[0] * rhs[3] + lhs[1] * rhs[2] - lhs[2] * rhs[1] + lhs[3] * rhs[0]
  ]);
};

function quaternionJs(q) {
  return {
    w: q[0],
    x: q[1],
    y: q[2],
    z: q[3],
  };
}

function makeQuaternionAxisRotate(x, y, z, angle_rad) {
  const factor = Math.sin(angle_rad / 2.0);

  return normalizeQuaternion([
    Math.cos(angle_rad / 2.0), x * factor, y * factor, z * factor])
};

function asymmetricScale(value, minval, maxval) {
  if (value <= 0) { return value * minval; }
  return value * maxval;
}

function asymmetricUnscale(value, minval, maxval) {
  if (value <= 0) { return value / minval; }
  return value / maxval;
}

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
    this._released = iota(Joystick.NUM_BUTTONS).map(x => false);
    this._axes = iota(4).map(x => 0.0);
    this.present = false;
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

    this.present = true;
    const old_down = [...this._down];
    this._down = iota(Joystick.NUM_BUTTONS).map(x => gp.buttons[x].pressed);
    this._pressed = iota(Joystick.NUM_BUTTONS).map(
      x => this._down[x] && !old_down[x]);
    this._released = iota(Joystick.NUM_BUTTONS).map(
      x => !this._down[x] && old_down[x])

    this._axes = [...gp.axes];
  }

  down(button) {
    return this._down[button];
  }

  pressed(button) {
    return this._pressed[button];
  }

  released(button) {
    return this._released[button];
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
    this._mode = "";
    this._state = null;
    this._joystick = new Joystick();

    // Fill in our constant values.
    document.getElementById('chart_rot_min').innerHTML =
      (-180.0 * CMD_MAX_RATE_Z / Math.PI).toFixed(0);
    document.getElementById('chart_rot_max').innerHTML =
      (180.0 * CMD_MAX_RATE_Z / Math.PI).toFixed(0);

    getElement('chart_x_min').innerHTML = (-CMD_MAX_RATE_X_REVERSE).toFixed(2);
    getElement('chart_x_max').innerHTML = CMD_MAX_RATE_X_FORWARD.toFixed(2);
    getElement('chart_y_min').innerHTML = (-CMD_MAX_RATE_Y).toFixed(2);
    getElement('chart_y_max').innerHTML = CMD_MAX_RATE_Y.toFixed(2);

    const checkbox = getElement('mode_expander');
    getElement('text').addEventListener('click', () => {
      checkbox.checked = !checkbox.checked;
    });

    getElement('mstop').addEventListener('click', () => {
      getElement('stop').checked = true;
      this._updateMode();
    });

    for (const mode_label of document.getElementsByClassName("mode_check")) {
      mode_label.addEventListener('input', () => {
        checkbox.checked = false;
        this._updateMode();
      });
    }

    getElement("power_off").addEventListener('click', () => {
      this._powerOff();
    });

    this._keys = {};
    this._kbd_v = [ 0, 0 ];
    this._kbd_w = [ 0 ];

    window.addEventListener(
      'keydown', (e) => { this._keydown(e); });
    window.addEventListener(
      'keyup', (e) => { this._keyup(e); });
  }

  start() {
    setInterval(() => this._handleTimer(), 100);
    this._openWebsocket();
    getElement("command_plot").focus();
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

  _powerOff() {
    for (const mode_check of document.getElementsByClassName("mode_check")) {
      mode_check.checked = false;
    }

    this._mode = 'off';
    getElement('mode_text').innerHTML = this._mode;
  }

  _openWebsocket() {
    // We don't know what mode we are in.
    this._mode = "";
    for (const mode_check of document.getElementsByClassName("mode_check")) {
      mode_check.checked = false;
    }
    this._state = null;

    const location = window.location;
    this._websocket = new WebSocket("ws://" + location.host + "/control");
    this._websocket.addEventListener(
      'message', (e) => { this._handleWebsocketMessage(e); });
    this._websocket.addEventListener(
      'close', () => { this._handleWebsocketClose(); });
  }

  _keydown(e) {
    this._keys[e.key] = true;
  }

  _keyup(e) {
    this._keys[e.key] = false;
  }

  _handleWebsocketMessage(e) {
    this._state = JSON.parse(e.data)

    this._updateState();
  }

  _handleWebsocketClose() {
    setTimeout(() => { this._openWebsocket()}, 500);
  }

  _handleTimer() {
    this._joystick.update();

    this._processJoystickCommands();
    this._processKeyboardCommands();

    this._updateState();
    this._sendCommand();
  }

  _moveMode(direction) {
    const mode_checks = [...document.getElementsByClassName("mode_check")];
    const checked = mode_checks.map(x => x.checked);
    const [first_selected, ...rest] = iota(mode_checks.length).filter(x => checked[x]);

    if (_isUndefined(first_selected)) {
      getElement("stop").checked = true;
    } else {
      const new_selected =
            Math.max(0, Math.min(
              mode_checks.length - 1, first_selected + direction));
      mode_checks[new_selected].checked = true;
    }
  }

  _processJoystickCommands() {
    const mode_check = getElement('mode_expander');
    if (this._joystick.pressed(Joystick.BUTTON_4)) {
      mode_check.checked = true;
    }
    if (this._joystick.released(Joystick.BUTTON_4)) {
      mode_check.checked = false;
      this._updateMode();
    }

    if (this._joystick.down(Joystick.BUTTON_4) &&
        mode_check.checked) {
      if (this._joystick.pressed(Joystick.BUTTON_HAT_UP)) {
        this._moveMode(-1);
      } else if (this._joystick.pressed(Joystick.BUTTON_HAT_DOWN)) {
        this._moveMode(1);
      }
    }
  }

  _updateKey(oldValue, upKey, downKey, minValue, maxValue) {
    const updateValue = (() => {
      if (this._keys[upKey]) { return maxValue; }
      if (this._keys[downKey]) { return minValue; }
      return 0.0;
    })();
    const alpha = 0.95;
    return oldValue * alpha + (1.0 - alpha) * updateValue;
  }

  _processKeyboardCommands() {
    if (this._joystick.present) {
      // We don't allow keyboard operation when a joystick is
      // connected.
      this._kbd_v = [0, 0];
      this._kbd_w = [0];
      return;
    }

    this._kbd_v[0] = this._updateKey(this._kbd_v[0], 'w', 's',
                                     -CMD_MAX_RATE_X_REVERSE,
                                     CMD_MAX_RATE_X_FORWARD);
    this._kbd_v[1] = this._updateKey(this._kbd_v[1], 'd', 'a',
                                     -CMD_MAX_RATE_Y, CMD_MAX_RATE_Y);
    this._kbd_w[0] = this._updateKey(this._kbd_w[0], 'e', 'q',
                                     -CMD_MAX_RATE_Z, CMD_MAX_RATE_Z);
  }

  _sendCommand() {
    // If we don't yet know our mode, then all we do is send an empty
    // command.
    if (this._mode == "") {
      if (this._websocket.readyState == WebSocket.OPEN) {
        this._websocket.send("{}");
      }
      return;
    }

    const [v_R_strafe, w_R, pose_RB] = (() => {
      // Are we in keyboard mode?
      if (!this._joystick.present) {
        const v_R = [this._kbd_v[0], this._kbd_v[1], 0];
        const w_R = [0, 0, this._kbd_w[0]];
        return [v_R, w_R, null];
      }
      // Are we in body mode?
      if (this._joystick.down(Joystick.BUTTON_SHOULDER_LB)) {
        const v_R = [0, 0, 0];
        const w_R = [0, 0, 0];

        const yaw = this._joystick.axis(Joystick.AXES_RIGHT_X) *
              CMD_MAX_POSE_YAW;
        const pitch = -this._joystick.axis(Joystick.AXES_RIGHT_Y) *
              CMD_MAX_POSE_PITCH;
        const pose_RB_so3 = quaternionMultiply(
          quaternionMultiply(
            makeUnitQuaternion(),
            makeQuaternionAxisRotate(0, 1, 0, pitch)),
          makeQuaternionAxisRotate(0, 0, 1, yaw));
        const pose_RB = {
          translation : [ 0, 0, 0],
          so3 : quaternionJs(pose_RB_so3),
        };
        return [v_R, w_R, pose_RB];
      } else {
        // Normal movement mode.
        const v_R = [
          asymmetricScale(-this._joystick.axis(Joystick.AXES_LEFT_Y),
                          CMD_MAX_RATE_X_REVERSE, CMD_MAX_RATE_X_FORWARD),
          this._joystick.axis(Joystick.AXES_LEFT_X) * CMD_MAX_RATE_Y,
          0.0
        ];
        const w_R = [
          0.0,
          0.0,
          this._joystick.axis(Joystick.AXES_RIGHT_X) * CMD_MAX_RATE_Z,
        ];
        return [v_R, w_R, null];
      }
    })();


    const disable_strafe = getElement("disable_strafe").checked;
    const v_R = disable_strafe ? [v_R_strafe[0], 0, 0] : v_R_strafe;

    {
      const desired_rot_cmd = getElement('desired_rot_cmd');
      const scaled_w =
            Math.max(-1.0, Math.min(1.0, w_R[2] / CMD_MAX_RATE_Z));
      desired_rot_cmd.setAttribute('x', `${scaled_w * 38 + 48}%`);

      const desired_trans_cmd = getElement('desired_trans_cmd');
      const scaled_x =
            Math.max(
              -1.0,
              Math.min(
                1.0,
                asymmetricUnscale(
                  v_R[0], CMD_MAX_RATE_X_REVERSE, CMD_MAX_RATE_X_FORWARD)));
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
          if (this._mode == "off") { return "stopped"; }
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

    const record_data = getElement("record_data").checked;
    // Unset leaves the current value alone, but we can just send it
    // every time.
    command["command"]["log"] = record_data ? "enable" : "disable";

    command["command"]["v_R"] = v_R;
    command["command"]["w_R"] = w_R;

    if (pose_RB) {
      command["command"]["rest"] = {
        offset_RB : pose_RB,
      };
    }

    if (command["command"]["mode"] == "jump") {
      command["command"]["jump"] = {
        "acceleration" : Number(getElement("jump_acceleration").value),
        "repeat" : getElement("jump_repeat").checked,
      };
    }

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

    if (this._mode == "") {
      // This is our first update, so set our mode to be whatever the
      // robot is at now.
      this._mode = (() => {
        const cur = this._state.mode;
        if (cur == "stopped") { return "off"; }
        if (cur == "zero_velocity") { return "stop"; }
        if (cur == "rest") { return "idle" };
        if (cur == "walk") { return "walk" };
        if (cur == "jump") { return "pronk" };
        return "stop";
      })();
      for (const mode_check  of document.getElementsByClassName("mode_check")) {
        if (mode_check.value == this._mode) {
          mode_check.checked = true;
        } else {
          mode_check.checked = false;
        }
      }
      getElement('mode_text').innerHTML = this._mode;
    }

    // Primary mode
    {
      const mode_sub = getElement('mode_sub');
      mode_sub.innerHTML = '(' + (() => {
        const cur = this._state.mode;
        if (cur == "stopped") { return "power off"; }
        if (cur == "zero_velocity") { return "damped"; }
        if (cur == "rest") { return "idle"; }
        if (cur == "jump") { return "pronk"; }
        return cur;
      })() + ')';
    }

    // Fault status.
    {
      const fault_container = getElement('fault_text_container');
      const fault_text = getElement('fault_text');
      if (this._state.fault.length > 0) {
        fault_text.innerHTML = this._state.fault;
        fault_container.style.display = "block";
      } else {
        fault_container.style.display = "none";
      }
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
            Math.max(-1.0, Math.min(
              1.0,
              asymmetricUnscale(
                desired_R.v[0], CMD_MAX_RATE_X_REVERSE, CMD_MAX_RATE_X_FORWARD)));
      const scaled_y =
            Math.max(-1.0, Math.min(1.0, desired_R.v[1] / CMD_MAX_RATE_Y));
      desired_trans_act.setAttribute('x', `${scaled_y * 38 + 49}%`);
      desired_trans_act.setAttribute('y', `${-scaled_x * 38 + 49}%`);
    }

    // Temperature
    {
      const temperature_container = getElement('temperature_container');
      const temperature_text = getElement('temperature_text');
      const joints = this._state.state.joints;
      const max_temp = Math.max(...joints.map(x => x.temperature_C));
      temperature_text.innerHTML = `${max_temp.toFixed(1)}C`;

      if (max_temp < 53.0) {
        temperature_container.style.backgroundColor = "#c0c0c0";
      } else if (max_temp < 63.0) {
        temperature_container.style.backgroundColor = "#d0d000";
      } else {
        temperature_container.style.backgroundColor = "#d00000";
      }
    }

    // Battery
    {
      const battery_level = getElement('battery_level');
      const battery_text = getElement('battery_text');

      const voltage = this._state.state.robot.voltage;
      battery_text.innerHTML = `${voltage.toFixed(1)}V`;
      const percentage =
            100.0 * Math.min(1.0, Math.max(0.0, (voltage - 16.0) / (19.5 - 16.0)));
      battery_level.style.height = `${percentage}%`;
      if (percentage < 10.0) {
        battery_level.style.backgroundColor = "#d00000";
      } else if (percentage < 30.0) {
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
