'use strict';

(function () {
  window.addEventListener('load', function (e) {
    var root = document.getElementById('app');
    root.innerHTML = "I have started!";
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
      root.innerHTML = `JS ax=${axs} bt=${bts}`;
      return;
    }

    root.innerHTML = "no joystick";
  };

  window.setInterval(step, 1000 / 10);
}());
