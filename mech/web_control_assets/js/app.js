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

const getElement = (v) => document.getElementById(v);

class Application {
  constructor() {
    this._websocket = null;
    this._mode = "stopped";
    this._state = null;

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

    for (const mode_label of document.getElementsByClassName("mode_label")) {
      mode_label.addEventListener('click', () => {
        checkbox.checked = false;
        this._updateCommand();
      });
    }
  }

  start() {
    setInterval(() => this._handleTimer(), 100);
    this._openWebsocket();
  }

  _updateCommand() {
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
    this._updateState();

    if (this._websocket.readyState == WebSocket.OPEN) {
      this._websocket.send('{}')
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
      desired_rot_act.setAttribute('x', `${scaled_w * 40 + 49}%`);

      const desired_trans_act = getElement('desired_trans_act');
      const scaled_x =
            Math.max(-1.0, Math.min(1.0, desired_R.v[0] / CMD_MAX_RATE_X));
      const scaled_y =
            Math.max(-1.0, Math.min(1.0, desired_R.v[1] / CMD_MAX_RATE_Y));
      desired_trans_act.setAttribute('x', `${scaled_y * 40 + 49}%`);
      desired_trans_act.setAttribute('y', `${-scaled_x * 40 + 49}%`);
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
}

const app = async () => {
  const app = new Application();
  app.start();
};

document.addEventListener("DOMContentLoaded", app)
