// const PAINT_DELAY_S = 0.5;
const PAINT_DELAY_S = 0;
// const HOST = '192.168.0.15'
// const HOST = '128.61.74.232'
const HOST = 'localhost'

COLORS = {
  '#000000': 0, // black
  '#ff0000': 1, // red
  '#ffff00': 2, // yellow
  '#008000': 3, // green
  '#0000ff': 4, // blue
  '#800080': 5  // purple
}

let connected = false;
function try_connect() {
  if (!connected) {
    var websocket = new WebSocket('ws://' + HOST + ':5906');
    // var websocket = new WebSocket('ws://' + HOST + ':5904');

    websocket.onerror = function (event) {
      console.log('WebSocket error: ', event);
      connected = false;
    };

    websocket.onopen = function (event) {
      console.log("Connected to ipad!!!");
      console.log(event);
      connected = true;
      // cdpr.control_mode = ControlMode.POSITION;
      cdpr.setSwitchableControllerMode(SwitchableControllerMode.SPLINE);
      websocket.send('F' + [cdpr.frame.w - cdpr.ee.w, cdpr.frame.h - cdpr.ee.h]);
    }
    
    websocket.onclose = function (event) {
      if (connected) {
        console.log("Disconnected from ipad!!!");
        console.log(event);
        connected = false;
        // cdpr.control_mode = ControlMode.VELOCITY;
        cdpr.setSwitchableControllerMode(SwitchableControllerMode.TRACKING);
      }
    }

    websocket.onmessage = function (event) {
      // console.log(event.data);
      let command = event.data[0];
      txy = event.data.slice(1).split(',')
      t = parseFloat(txy[0]);
      let [w1, h1] = [cdpr.frame.w - cdpr.ee.w, cdpr.frame.h - cdpr.ee.h];
      x = parseFloat(txy[1]) * w1
      y = ((h1 / w1) - parseFloat(txy[2])) * w1
      console.log(command, x, y)

      // cdpr.set_x = x;
      // cdpr.set_y = y;
      if (command == 'M') {
        cdpr.add_to_queue(x, y, false);
        // for (let i = 0; i < PAINT_DELAY_S * 150; i++) {
        //   cdpr.add_to_queue(x, y, true, true);
        // }
      } else if (command == 'L') {
      // } else if (command == 'N') {
        cdpr.add_to_queue(x, y, true);
      } else if (command == 'U') {
        cdpr.add_to_queue(x, y, false);
        // TODO(gerry): make back-setting work
        // // back-set the past PAINT_DELAY seconds of paint to be false
        // const l = Math.min(cdpr.set_queue.length, Math.floor(PAINT_DELAY_S * 150));
        // for (let i = cdpr.set_queue.length - l; i < cdpr.set_queue.length; i++) {
        //   cdpr.set_queue[i][2] = false;
        // }
        // forward set the next PAINT_DELAY seconds of paint to be false
        // const l = 0;
        // for (let i = l; i < Math.floor(PAINT_DELAY_S * 150); i++) {
        //   cdpr.add_to_queue(x, y, false);
        // }
      } else if (command == 'C') {
        let c1 = COLORS[txy[1]];
        console.log(event.data, c1);
        cdpr.set_color(c1);
        // cdpr.frame.w = parseFloat(txy[1]);
        // cdpr.frame.h = parseFloat(txy[2]);
      }
    }
  }
}
setInterval(try_connect, 500);