'use strict';

// Copyright 2019 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

(function () {
  var websocket = null;
  var active = false;

  var BUTTON_START = 9;
  var BUTTON_SELECT = 8;
  var BUTTON_2 = 0;
  var BUTTON_3 = 1;
  var BUTTON_1 = 2;
  var BUTTON_4 = 3;
  var BUTTON_SHOULDER_LT = 4;
  var BUTTON_SHOULDER_RT = 5;
  var BUTTON_SHOULDER_LB = 6;
  var BUTTON_SHOULDER_RB = 7;
  var BUTTON_LEFT_JOY = 10;
  var BUTTON_RIGHT_JOY = 11;
  var BUTTON_HAT_UP = 12;
  var BUTTON_HAT_DOWN = 13;
  var BUTTON_HAT_LEFT = 14;
  var BUTTON_HAT_RIGHT = 15;

  var NUM_BUTTONS = 16;

  var RB_MAX_X = 50;
  var RB_MAX_Y = 50;
  var RB_MAX_ROLL = 0.35;
  var RB_MAX_PITCH = 0.35;

  var VEL_MAX_X = 300.0;
  var VEL_MAX_Y = 200.0;
  var RATE_MAX_Z = 1.0;

  var handleEvent = function(e) {
    var state = document.getElementById('robot_state');
    state.innerHTML = "Status: " + e.data;
  };

  var openWebsocket = function() {
    var loc = window.location;
    websocket = new WebSocket("ws://" + loc.host + "/control");

    websocket.addEventListener('message', handleEvent);
    websocket.addEventListener('close', openWebsocket);
  };

  window.addEventListener('load', function (e) {
    openWebsocket()
  });

  var renderJoystick = function(gp) {
    var ax = gp.axes;
    var buttons = gp.buttons;
    var axs = [];
    for (var i = 0; i < ax.length; i++) {
      axs[i] = ax[i].toFixed(3);
    }
    var bts = [];
    for (var i = 0; i < buttons.length; i++) {
      bts[i] = buttons[i].pressed;
    }
    var text = `JS ax=${axs} bt=${bts}`;
    return text;
  };

  var getJoystick = function() {
    var gp = navigator.getGamepads();
    for (var index = 0; index < gp.length; index++) {
      if (gp[index]) { return gp[index]; }
    }
    return null;
  };

  var old_buttons = null;

  var checkButtons = function(gp) {
    var result = [];
    if (old_buttons) {
      for (var i = 0; i < NUM_BUTTONS; i++) {
        result[i] = gp.buttons[i].pressed && !old_buttons[i];
      }
    } else {
      for (var i = 0; i < NUM_BUTTONS; i++) {
        result[i] = false;
      }
    }
    if (gp) {
      old_buttons = [];
      for (var i = 0; i < NUM_BUTTONS; i++) {
        old_buttons[i] = gp.buttons[i].pressed;
      }
    }
    return result;
  }

  var downButtons = function(gp) {
    var result = [];
    for (var i = 0; i < NUM_BUTTONS; i++) {
      result[i] = gp ? gp.buttons[i].pressed : false;
    }
    return result;
  };

  var normalizeQuaternion = function(q) {
    var norm = Math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    return [
      q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm
    ];
  };

  var makeUnitQuaternion = function() {
    return [1, 0, 0, 0];
  };

  var quaternionMultiply = function(lhs, rhs) {
    return normalizeQuaternion([
      lhs[0] * rhs[0] - lhs[1] * rhs[1] - lhs[2] * rhs[2] - lhs[3] * rhs[3],
      lhs[0] * rhs[1] + lhs[1] * rhs[0] + lhs[2] * rhs[3] - lhs[3] * rhs[2],
      lhs[0] * rhs[2] - lhs[1] * rhs[3] + lhs[2] * rhs[0] + lhs[3] * rhs[1],
      lhs[0] * rhs[3] + lhs[1] * rhs[2] - lhs[2] * rhs[1] + lhs[3] * rhs[0]
    ]);
  };

  var quaternionJs = function(q) {
    return {
      w: q[0],
      x: q[1],
      y: q[2],
      z: q[3],
    };
  }

  var makeQuaternionAxisRotate = function(x, y, z, angle_rad) {
    var factor = Math.sin(angle_rad / 2.0);

    return normalizeQuaternion([
      Math.cos(angle_rad / 2.0), x * factor, y * factor, z * factor])
  };

  var makeCommand = function(gp) {
    var pressed = checkButtons(gp);
    var down = downButtons(gp);

    var mode_selector = document.getElementById("mode_selector");
    var old_value = mode_selector.value;

    if (pressed[BUTTON_START]) {
      // If we have no value, are stopped, or at zero, switch to rest.
      if (old_value == "" ||
          old_value == "stopped" ||
          old_value == "zero_velocity") {
        mode_selector.value = "rest";
      } else {
        // Otherwise, we switch to zero.
        mode_selector.value = "zero_velocity";
      }
    } else if (pressed[BUTTON_SELECT]) {
      // If we aren't in jump, switch to that.  If we are, then just
      // switch to rest.
      if (old_value != "jump") {
        mode_selector.value = "jump";
      } else {
        mode_selector.value = "rest";
      }
    }

    var pose_mm_RB = null;

    var v_mm_s_R = null;
    var w_LR = null;

    if (down[BUTTON_SHOULDER_LB]) {
      // This means the joystick axes are used to change the RB
      // transform.
      var pose_mm_RB_x = -gp.axes[1] * RB_MAX_X;
      var pose_mm_RB_y = gp.axes[0] * RB_MAX_Y;

      var pose_mm_RB_so3 = quaternionMultiply(
        quaternionMultiply(
          makeUnitQuaternion(),
          makeQuaternionAxisRotate(1, 0, 0, gp.axes[2] * RB_MAX_ROLL)),
        makeQuaternionAxisRotate(0, 1, 0, -gp.axes[3] * RB_MAX_PITCH));

      pose_mm_RB = {
        translation : [ pose_mm_RB_x, pose_mm_RB_y, 0.0 ],
        so3 : quaternionJs(pose_mm_RB_so3),
      };
    } else if (gp) {
      v_mm_s_R = [ -gp.axes[1] * VEL_MAX_X, gp.axes[0] * VEL_MAX_Y, 0.0 ];
      w_LR = [ 0, 0, gp.axes[2] * RATE_MAX_Z ];
    }

    var value = mode_selector.value;

    var cmd = {};
    if (value == "stopped" ||
        value == "zero_velocity" ||
        value == "rest" ||
        value == "jump" ||
        value == "walk") {
      // We always send this command out.
      var command = {
        "mode" : value,
      };
      cmd['command'] = command;
    }

    if ('command' in cmd) {
      if (pose_mm_RB) {
        cmd['command']['pose_mm_RB'] = pose_mm_RB;
      }
      if (v_mm_s_R) {
        cmd['command']['v_mm_s_R'] = v_mm_s_R;
      }
      if (w_LR) {
        cmd['command']['w_LR'] = w_LR;
      }
      if (value == 'jump') {
        cmd['command']['jump'] = {
          "acceleration_mm_s2" : 2000.0,
          "repeat" : true,
        };
      }
    }

    return JSON.stringify(cmd);
  };

  var step = function(timestamp) {
    var jsui = document.getElementById('current_joystick');

    var gp = getJoystick();
    jsui.innerHTML = gp ? renderJoystick(gp) : "no joystick";

    var cmd = makeCommand(gp);
    var ccui = document.getElementById('current_command');
    ccui.innerHTML = "Command: " + cmd;

    websocket.send(cmd);
  };

  window.setInterval(step, 1000 / 10);
}());
