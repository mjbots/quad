'use strict';

(function () {
  var websocket = null;

  var handleEvent = function(e) {
    console.log("got websocket event:" + e.data)
  };

  var openWebsocket = function() {
    var loc = window.location;
    websocket = new WebSocket("ws://" + loc.host + "/control");

    websocket.addEventListener('message', handleEvent);
    websocket.addEventListener('close', openWebsocket);
  };

  window.addEventListener('load', function (e) {
    var root = document.getElementById('app');
    root.innerHTML = "I have started!";

    openWebsocket()
  });

  function step(timestamp) {
    var gp = navigator.getGamepads();

    var root = document.getElementById('app');

    for (var index = 0; index < gp.length; index++) {
      if (!gp[index]) { continue; }

      var ax = gp[index].axes;
      var buttons = gp[index].buttons;
      var axs = [];
      for (var i = 0; i < ax.length; i++) {
        axs[i] = ax[i].toFixed(3);
      }
      var bts = [];
      for (var i = 0; i < buttons.length; i++) {
        bts[i] = buttons[i].pressed;
      }
      var text = `JS ax=${axs} bt=${bts}`;
      root.innerHTML = text;

      websocket.send(text);
      return;
    }

    root.innerHTML = "no joystick";
  };

  window.setInterval(step, 1000 / 10);
}());
