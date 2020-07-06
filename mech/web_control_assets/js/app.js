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

class Application {
  constructor() {
    this._websocket = null;
    this._mode = "stopped";
    this._state = null;
  }

  start() {
    setInterval(() => this._handleTimer(), 100);
    this._openWebsocket();
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
    const connected_container = document.getElementById('connected_state_container');
    const connected_text = document.getElementById('connected_state');

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

    const mode_sub = document.getElementById('mode_sub');
    mode_sub.innerHTML = '(' + (() => {
      const cur = this._state.mode;
      if (cur == "zero_velocity") { return "damped"; }
      if (cur == "rest") { return "idle"; }
      return cur;
    })() + ')';

    const battery_level = document.getElementById('battery_level');
    const battery_text = document.getElementById('battery_text');

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

const app = async () => {
  const app = new Application();
  app.start();
};

document.addEventListener("DOMContentLoaded", app)
