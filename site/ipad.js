// const PAINT_DELAY_S = 0.5;
const PAINT_DELAY_S = 0;
// const HOST = '192.168.0.15'
// const HOST = '128.61.74.232'
// const HOST = 'localhost'
// const WS_ADDR = 'ws://' + HOST + ':5906'
const WS_ADDR = {
  iPad: "wss://paint.gerry-chen.com/whiteboard_passthrough/ws",
  diffusion: "wss://paint.gerry-chen.com/diffusion_output/ws"
};
let ws_select = 'iPad';

COLORS = {
  '#000000': 0, // black
  '#ff0000': 1, // red
  '#ffff00': 2, // yellow
  '#008000': 3, // green
  '#0000ff': 4, // blue
  '#800080': 5  // purple
}


// CANVAS_BOUNDS = { w: [1.0, 1.8], h: [0.65, 1.0] };
CANVAS_BOUNDS = { w: [0.9, 3.25], h: [0.63, 2.7] };
// CANVAS_BOUNDS = { w: [0.72, 3.49], h: [0.63, 2.7] };

function retargeting_toggle() {
  const elem = document.getElementById("retargetingToggle");
  if (ws_select == 'iPad') {
    ws_select = "diffusion";
    elem.classList.add("active");
    elem.textContent = "Retargeting ON";
  } else {
    ws_select = "iPad";
    elem.classList.remove("active");
    elem.textContent = "Retargeting Off";
  }
}

let connected = Object.fromEntries(Object.keys(WS_ADDR).map(key => [key, false]));
function try_connect(key) {
  if (!connected[key]) {
    var websocket = new WebSocket(WS_ADDR[key]);

    websocket.onerror = function (event) {
      console.log(`WebSocket error for ${key}: `, event);
      connected[key] = false;
    };

    websocket.onopen = function (event) {
      console.log(`Connected to ${key}!!!`);
      console.log(event);
      connected[key] = true;
      // cdpr.control_mode = ControlMode.POSITION;
      cdpr.setSwitchableControllerMode(SwitchableControllerMode.WAYPOINTS);
      // cdpr.setSwitchableControllerMode(SwitchableControllerMode.SPLINE);
      websocket.send('F' + [cdpr.frame.w - cdpr.ee.w, cdpr.frame.h - cdpr.ee.h]);
    }
    
    websocket.onclose = function (event) {
      if (connected) {
        console.log(`Disconnected from ${key}!!!`);
        console.log(event);
        connected[key] = false;
        // cdpr.control_mode = ControlMode.VELOCITY;
        cdpr.setSwitchableControllerMode(SwitchableControllerMode.TRACKING);
      }
    }

    websocket.onmessage = function (event) {
      if (ws_select != key) return;
      // console.log(event.data);
      let command = event.data[0];
      txy = event.data.slice(1).split(',')
      t = parseFloat(txy[0]);
      let [w1, h1] = [cdpr.frame.w - cdpr.ee.w, cdpr.frame.h - cdpr.ee.h];
      // x = parseFloat(txy[1]) * w1
      // y = ((h1 / w1) - parseFloat(txy[2])) * w1

      // scales x, y to CANVAS_BOUNDS
      console.log('this is txy', txy[2], h1 / w1)
      if (false) { // iPad
        x = parseFloat(txy[1]) * (CANVAS_BOUNDS.w[1] - CANVAS_BOUNDS.w[0]) + CANVAS_BOUNDS.w[0];
        // y = parseFloat(txy[2]) * (CANVAS_BOUNDS.h[1] - CANVAS_BOUNDS.h[0]) + CANVAS_BOUNDS.h[0]; // this will squish aspect ratio
        // console.log('txy is :', txy[2], h1 / w1);
        // y = ((h1 / w1) - parseFloat(txy[2])) * (CANVAS_BOUNDS.w[1] - CANVAS_BOUNDS.w[0]) + CANVAS_BOUNDS.h[0]; // this will squish aspect ratio
        // y = ((0.67) - parseFloat(txy[2])) * (CANVAS_BOUNDS.w[1] - CANVAS_BOUNDS.w[0]) + CANVAS_BOUNDS.h[0]; // this will squish aspect ratio
        y = ((0.60) - parseFloat(txy[2])) * (CANVAS_BOUNDS.w[1] - CANVAS_BOUNDS.w[0]) + CANVAS_BOUNDS.h[0]; // this will squish aspect ratio
        // 0.60 currently needs to be tuned to the ipad's aspect ratio, but we should send this over socket.
      } else { // gcode
        x = txy[1];
        y = txy[2];
      }

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
function try_connect_both() {
  try_connect("iPad");
  try_connect("diffusion");
}
setInterval(try_connect_both, 500);
