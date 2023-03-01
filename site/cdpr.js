// const SPEED = 0.5 * 2.3;
const CDPR_SPEED = 0.5;
const SPEED = CDPR_SPEED * 2.3;
const BRUSH_WAIT_DELAY_MS = 1500;

function Dims(w, h) {
  this.w = w;
  this.h = h;
}

function Corners(dims) {
  return [[dims.w, 0], [dims.w, dims.h], [0, dims.h], [0, 0]];
}

function Pose2(x, y, th) { this.x = x; this.y = y; this.th = th; }
function Motor(error, state, l, ldot) { this.error = error; this.state = state; this.l = l; this.ldot = ldot; }

function ControllerState(t_us, state, est, set) { this.t_us = t_us; this.state = state; this.est = est; this.set = set; }
function CdprState(controller, motors, spray) { this.controller = controller; this.motors = motors; this.spray = spray; this.t = window.performance.now() / 1000; }

const ControlMode = { POSITION: "position", VELOCITY: "velocity" };
// const SwitchableControllerMode = { TRACKING: "tracking", SPLINE: "spline", ILQR: "ilqr" };
const SwitchableControllerMode = {
  TRACKING: "tracking",
  SPLINE: "spline",
  ILQR: "ilqr",
};
const Mode = { IDLE: "idle", HOLD: "hold", TRACKING: "tracking" };
const Status = {
  IDLE: "idle",
  DRAWING: "drawing",
  TRAVELING: "traveling",
  WAITING_FOR_BRUSH: "waiting_for_brush",
};

function Cdpr(frameDims, eeDims) {
  this.frame = frameDims;
  this.ee = eeDims;
  this.padding_w = 0.3;
  this.padding_h = 0.3;
  this.isSpray = false;
  this.color = 0;
  this.x = this.frame.w / 2;
  this.y = this.frame.h / 2;
  this.vx = 0;
  this.vy = 0;
  // this.set_x = this.x;
  // this.set_y = this.y;
  this.set_queue = [[this.x * 1, this.y * 1, false, 0, false]];
  this.tmp_queue = [];
  this.stroke_queue = [];
  this.stroke_being_drawn = [];
  this.prev_setpointState = -1;
  this.status = Status.IDLE;
  this.control_mode = ControlMode.VELOCITY;
  this.switchable_controller_mode = SwitchableControllerMode.TRACKING;
  this.lastState = new CdprState(
    new ControllerState(
      null,
      null,
      new Pose2(this.x, this.y, 0),
      new Pose2(this.x, this.y, 0)
    ),
    null,
    null
  );
  this.setpointStatus = { t_s: 0.0, state: -1, status: -1, isDone: -1 };
  this.max_dt = 0.1;
  this.logBuffer = "";
  this.mode = Mode.IDLE;
}

Cdpr.prototype.sendStartupMessages = function () {
  // Initializing Marker Changer parameters
  this.send("ss2,10"); // Set color changing speed
  // this.send('ss1,150');
};

function zip(a, b) {
  return a.map(function (e, i) {
    return [e, b[i]];
  });
}

Cdpr.prototype.draw = function (ctx) {
  // Draw frame
  ctx.lineWidth = 0.06;
  ctx.strokeRect(0, 0, this.frame.w, this.frame.h);

  // Draw ee & cables for both estimated and setpoint poses
  const poses_colors = [
    [this.lastState.controller.set, "#f008"],
    [new Pose2(this.x, this.y, 0), "#0003"],
    [this.lastState.controller.est, "#000"],
  ];
  // For loop
  for (const [pose, color] of poses_colors) {
    x = pose.x - this.ee.w / 2;
    y = pose.y - this.ee.h / 2;

    // Draw ee
    ctx.lineWidth = 0.06;
    ctx.strokeStyle = color;
    ctx.fillStyle = color;
    ctx.fillRect(x, y, this.ee.w, this.ee.h);

    // Draw cables
    ctx.beginPath();
    ctx.lineWidth = 0.02;
    for (const [frame, ee] of zip(Corners(this.frame), Corners(this.ee))) {
      ctx.moveTo(frame[0], frame[1]);
      ctx.lineTo(x + ee[0], y + ee[1]);
    }
    ctx.stroke();
  }
  // Draw setpoint queue
  ctx.beginPath();
  ctx.lineWidth = 0.25;
  ctx.strokeStyle = "#afa";
  ctx.lineCap = "round";
  // first draw the stroke being drawn
  if (this.stroke_being_drawn.length > 0) {
    ctx.moveTo(this.stroke_being_drawn[0][0], this.stroke_being_drawn[0][1]);
    for (const [x, y] of this.stroke_being_drawn) {
      ctx.lineTo(x, y);
    }
    ctx.stroke();
  }
  ctx.strokeStyle = "#aaa";
  for (const stroke of this.stroke_queue) {
    ctx.moveTo(stroke[0][0], stroke[0][1]);
    for (const [x, y] of stroke) {
      ctx.lineTo(x, y);
    }
    ctx.stroke();
  }
};

Cdpr.prototype.setControls = function (vx, vy) {
  this.vx = vx;
  this.vy = vy;
};

function clamp(x, min, max) {
  return Math.min(Math.max(x, min), max);
}
function dist(x1, y1, x2, y2) {
  return Math.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}
function towards(x, y, newx, newy, thresh) {
  dx = newx - x;
  dy = newy - y;
  d = Math.sqrt(dx * dx + dy * dy);
  if (d < thresh) {
    return [newx, newy, true];
  }
  return [x + (dx * thresh) / d, y + (dy * thresh) / d, false];
}

let paint_out = false;
let switching_spray = false;
Cdpr.prototype.update = function (dt) {
  if (this.mode != Mode.TRACKING && this.lastState.t_us !== null) {
    this.x = this.lastState.controller.est.x;
    this.y = this.lastState.controller.est.y;
    return;
  }
  if (this.control_mode == ControlMode.VELOCITY) {
    if (this.vx == 0 && this.vy == 0) return;
    this.x += this.vx * dt;
    this.y += this.vy * dt;
    [this.x, this.y] = towards(
      this.lastState.controller.est.x,
      this.lastState.controller.est.y,
      this.x,
      this.y,
      CDPR_SPEED * 0.25 // max 0.25s worth of lead
    );
    // this.x = this.lastState.controller.est.x + this.vx * this.max_dt * 2.0;
    // this.y = this.lastState.controller.est.y + this.vy * this.max_dt * 2.0;

    this.x = clamp(
      this.x,
      this.ee.w / 2 + this.padding_w,
      this.frame.w - this.ee.w / 2 - this.padding_w
    );
    this.y = clamp(
      this.y,
      this.ee.h / 2 + this.padding_h,
      this.frame.h - this.ee.h / 2 - this.padding_h
    );
    console.log(
      "WE ARE IN VELOCITY MODE",
      this.mode,
      this.switchable_controller_mode,
      this.control_mode
    );
    if (
      this.mode === Mode.TRACKING &&
      this.switchable_controller_mode == SwitchableControllerMode.TRACKING
    ) {
      this.sendPosition();
    }
  } else if (this.control_mode == ControlMode.POSITION) {
    // TODO: this!
    console.log(
      "WE ARE IN POSITION MODE",
      this.status,
      this.setpointStatus.state,
      this.stroke_queue.length
    );
    // this.spray(this.setpointStatus.state == 2);

    if (this.status == Status.WAITING_FOR_BRUSH) {
      return;
    }
    if (this.status == Status.TRAVELING) {
      if (this.setpointStatus.state == 0) {  // finished traveling
        this.spray(true); // extend the brush
        this.status = Status.WAITING_FOR_BRUSH;
        setTimeout(() => {
          this.send("x2"); // start drawing
          this.send("x?"); // poll status
          setTimeout(() => {
            this.status = Status.DRAWING;
          }, 100); // give enough time for poll to respond
        }, BRUSH_WAIT_DELAY_MS);
        return;
      }
    }
    if (this.status == Status.DRAWING) {
      if (this.setpointStatus.state == 0) {  // finished drawing
        this.send("xc", false); // clear waypoints
        this.spray(false);
        this.status = Status.WAITING_FOR_BRUSH;
        setTimeout(() => {
          this.status = Status.IDLE;
          this.send("x?"); // poll status
        }, BRUSH_WAIT_DELAY_MS);
        return;
      }
    }
    if (this.status == Status.IDLE) {
      // Send new waypoints
      if (this.stroke_queue.length > 0) {
        this.stroke_being_drawn = this.stroke_queue.shift();
      } else {
        this.stroke_being_drawn = [];
        return; // ??? wait until there is a stroke
      }
      this.send("xc", false); // clear waypoints
      for (const [x, y] of this.stroke_being_drawn) {
        this.send("xn" + x + "," + y + "," + 0.0);
      }
      this.send("x0;x1"); // restart and start traveling
      this.send("x?"); // poll status
      this.status = Status.WAITING_FOR_BRUSH; // we're not actually waiting on brush but this is convenient
      setTimeout(() => {this.status = Status.TRAVELING;}, 100); // give enough time for poll to respond
    }

    this.prev_setpointState = this.setpointStatus.state;
    return;

    /*
    var next;
    // do {
      [this.x, this.y, spray, color, force] = this.set_queue[0];

      // this.spray(spray); // TODO: figure out how to queue color commands in Teensy...

      // Logic to wait before moving to next setpoint
      let cur_xy = this.lastState.controller.est;
      let dist_to_goal = dist(this.x, this.y, cur_xy.x, cur_xy.y);
      // [dummy1, dummy2, next_spray, next_color, dummy3] = this.set_queue[1];
      console.log(spray, paint_out, dist_to_goal);
      if (paint_out != spray) { // push marker in/out
        if ((!switching_spray) && (dist_to_goal < 0.02)) { // wait until reaching position before spraying
          switching_spray = true;
          this.spray(spray);
          setTimeout(() => { paint_out = spray; switching_spray = false; }, 2000);
        }
      }

      if (this.set_queue.length > 1) {
        if (paint_out == spray) {
          this.set_queue.shift();
        }
      }

      // let unreachable = ((set_x < this.ee.w / 2 + this.padding_w)
      //   || (set_x > this.frame.w - this.ee.w / 2 - this.padding_w)
      //   || (set_y < this.ee.h / 2 + this.padding_h)
      //   || (set_y > this.frame.h - this.ee.h / 2 - this.padding_h));

      // [this.x, this.y, _] = towards(
      //   this.lastState.controller.est.x, this.lastState.controller.est.y,
      //   this.x, this.y, 
      //   SPEED * this.max_dt * 1.5);
      // [this.x, this.y, success] = towards(this.x, this.y, set_x, set_y, SPEED * dt);

      // next = (this.set_queue.length > 1) && (success || unreachable);
      // if (next) {
      //   this.set_queue.shift();
      //   if (force) {
      //     break;
      //   }
      // }
    // } while (next);

    */
  }
};

Cdpr.prototype.add_to_queue = function (x, y, spray, force = false) {
  // [x2, y2, spray2] = this.set_queue[this.set_queue.length - 1];
  // if ((dist(x, y, x2, y2) > 0.025) || (!spray)) {
  // this.set_queue.push([x, y, spray, this.color, force]);
  // }
  this.tmp_queue.push([x, y, spray, this.color, force]);
  if (!spray) {
    // user finished inputting stroke
    if (this.tmp_queue.length > 1) {
      this.stroke_queue.push([...this.tmp_queue]);
    }
    this.tmp_queue = [];
  }
  console.log(
    "add to queue",
    this.stroke_queue.length,
    this.tmp_queue.length,
    this.tmp_queue
  );
};

/*********** CDPR CONTROL CODE ***************/
Cdpr.prototype.clearErrors = function () {
  this.send("g0");
};
Cdpr.prototype.estop = function () {
  this.send("0n2n");
  this.send("1n2n");
  this.send("2n2n");
  this.send("3n2n");
  this.send("0n2c");
  this.send("1n2c");
  this.send("2n2c");
  this.send("3n2c");
};
Cdpr.prototype.setMode = function (mode) {
  const pos = (this.lastState.t_us !== null) ? this.lastState.controller.est : new Pose2(this.x, this.y, 0);
  const MSGS = { [Mode.IDLE]: 'k0', [Mode.HOLD]: `xw${pos.x},${pos.y},0;k1;g8`, [Mode.TRACKING]: `k2;x2` };
  // const MSGS = { [Mode.IDLE]: 'g7', [Mode.HOLD]: `ta${pos.x},${pos.y};g6;g8`, [Mode.TRACKING]: `g1;g2` };
  this.send(MSGS[mode]);
  this.mode = mode;
}
Cdpr.prototype.resetTraj = function () { this.send('x4'); };
Cdpr.prototype.setSwitchableControllerMode = function (mode) {
  const LOOKUP_TABLE = {
    [SwitchableControllerMode.TRACKING]: 0,
    [SwitchableControllerMode.SPLINE]: 1,
    [SwitchableControllerMode.ILQR]: 2,
  };
  this.setMode(Mode.HOLD);
  this.send(`gs${LOOKUP_TABLE[mode]}`);
  if (mode == SwitchableControllerMode.TRACKING) {
    this.control_mode = ControlMode.VELOCITY;
  } else if (mode == SwitchableControllerMode.SPLINE) {
    this.control_mode = ControlMode.POSITION;
  }
  this.switchable_controller_mode = mode;
};
Cdpr.prototype.spray = function (on, force = false) {
  if (!force && this.isSpray != on) {
    this.send(`s${on ? 1 : 0}`);
    // this.send(`sM1,${on ? 7500 : 3000}`);
    this.send(`sM1,${on ? 2000 : 0}`);
    this.isSpray = on;
  }
};
var color_timer = null;
Cdpr.prototype.set_color = function (color, retry = false) {
  if (!retry) {
    clearTimeout(color_timer);
  }
  console.log(this.lastState.spray);
  if (!this.lastState.spray) {
    // Don't allow changing colors when we are currently "spraying"
    this.color = color;
    // this.send('sM1,-150');
    this.send("sc" + color);
    // this.send('sM1,0');
  } else {
    color_timer = setTimeout(() => this.set_color(color, true), 100); // Try again soon
  }
};
Cdpr.prototype.next_color = function () {
  this.set_color((this.color + 1) % 6);
};
Cdpr.prototype.prev_color = function () {
  this.set_color((this.color + 5) % 6);
};

/*********** CDPR CONTROL CODE ***************/

/*********** SERIAL SENDING CODE ***************/
Cdpr.prototype.sendPosition = function () {
  toSend = `xw${this.x},${this.y},0`;
  writer.write(toSend + "\n");
  println(toSend, 1);
};
Cdpr.prototype.send = function (msg, print_out = true) {
  writer.write(msg + "\n");
  if (print_out) println(msg, 0);
};
/*********** END SERIAL SENDING CODE ***************/

/*********** SERIAL PARSING CODE ***************/
function joinRegexWith(regexes, sep) {
  return new RegExp(regexes.map((x) => x.source).join(sep.source));
}
const INT_REGEX = /([-+]?\d+)/;
const FLOAT_REGEX = /([-+]?\d*\.?\d*)/;
const XYZ_REGEX = joinRegexWith([FLOAT_REGEX, FLOAT_REGEX, FLOAT_REGEX], /\s+/);
const PREAMBLE_REGEX = new RegExp(INT_REGEX.source + " - " + INT_REGEX.source + ": " + XYZ_REGEX.source + " - " + XYZ_REGEX.source);
const MOTOR_REGEX = joinRegexWith([INT_REGEX, INT_REGEX, FLOAT_REGEX, FLOAT_REGEX], /\s+/);
const LINE_REGEX = new RegExp("^\\s*" + joinRegexWith([PREAMBLE_REGEX, MOTOR_REGEX, MOTOR_REGEX, MOTOR_REGEX, MOTOR_REGEX, INT_REGEX], /\s\|\s/).source + "\\s*$");
// 2000003 - 0: 0.0000 1.5547 0.2296 - 0.0000 1.4315 1.1590	|	0 0 1.8350 0.0000	|	0 0 2.7660 0.0000	|	0 0 2.7839 0.0000	|	0 0 1.6564 -0.0000	|	0

const SETPOINT_STATUS_REGEX = new RegExp(
  "^setpoint: STATUS: " +
    joinRegexWith([FLOAT_REGEX, INT_REGEX, INT_REGEX, INT_REGEX], /\s/).source +
    "\\s*$"
);

Pose2.fromStrArr = function (arr) {
  return new Pose2(parseFloat(arr[1]), parseFloat(arr[2]), parseFloat(arr[0]));
};
Motor.fromStrArr = function (arr) {
  return new Motor(parseInt(arr[0]), parseInt(arr[1]), parseFloat(arr[2]), parseFloat(arr[3]));
}

function parseLine(line) {
  result = line.match(LINE_REGEX);
  if (result && result.length == 26) {
    return new CdprState(
      (controller = {
        t_us: parseInt(result[1]),
        state: parseInt(result[2]),
        est: Pose2.fromStrArr(result.slice(3, 6)),
        set: Pose2.fromStrArr(result.slice(6, 9)),
      }),
      (motors = [
        Motor.fromStrArr(result.slice(9, 13)),
        Motor.fromStrArr(result.slice(13, 17)),
        Motor.fromStrArr(result.slice(17, 21)),
        Motor.fromStrArr(result.slice(21, 25)),
      ]),
      (spray = parseInt(result[25]))
    );
  }
  return null;
}

var lastMatch = null;
Cdpr.prototype.parseLogString = function (logString, printToTerminal_cb) {
  this.logBuffer += logString;
  if (this.logBuffer.includes("\n")) {
    const lines = this.logBuffer.split("\n");
    this.logBuffer = lines.pop();
    for (const line of lines) {
      const result = parseLine(line);
      if (result) {
        printToTerminal_cb(line + "\n");
        this.max_dt =
          (result.controller.t_us - this.lastState.controller.t_us) / 1e6;
        while (this.max_dt < 0) {
          this.max_dt += 10;
        }
        this.lastState = result;
        // console.log(result.controller.est, result.controller.set, this.max_dt);
        // [this.x, this.y, met] = towards(result.controller.est.x, result.controller.est.y, this.x, this.y, SPEED * 1.5 / 10);
      } else {
        if (line.toLowerCase().startsWith("setpoint")) {
          printlnSetpoint(line);
          const resultStatus = line.match(SETPOINT_STATUS_REGEX);
          if (resultStatus && resultStatus.length == 5) {
            this.setpointStatus = {
              t_s: parseFloat(resultStatus[1]),
              state: parseInt(resultStatus[2]),
              status: parseInt(resultStatus[3]),
              isDone: parseInt(resultStatus[4]),
            };
          }
        } else if (line.toLowerCase().startsWith("tracking")) {
          printlnTracker(line);
        } else if (line.toLowerCase().startsWith("waypoint")) {
          printlnWaypoint(line);
        } else {
          printToTerminal_cb(line + "\n");
        }
      }
    }
  }
};
/*********** END SERIAL PARSING CODE ***************/
