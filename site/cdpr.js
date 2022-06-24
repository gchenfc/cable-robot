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
function CdprState(controller, motors, spray) { this.controller = controller; this.motors = motors; this.spray = spray; }

function Cdpr(frameDims, eeDims) {
  this.frame = frameDims;
  this.ee = eeDims;
  this.x = this.frame.w / 2;
  this.y = this.frame.h / 2;
  this.vx = 0;
  this.vy = 0;
  this.lastState = new CdprState(new ControllerState(null, null, new Pose2(this.x, this.y, 0), new Pose2(this.x, this.y, 0)), null, null);
  this.logBuffer = "";
}

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
    [this.lastState.controller.set, '#f008'],
    [new Pose2(this.x, this.y, 0), '#0003'],
    [this.lastState.controller.est, '#000']
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
}

Cdpr.prototype.setControls = function (vx, vy) {
  this.vx = vx;
  this.vy = vy;
}

function clamp(x, min, max) {
  return Math.min(Math.max(x, min), max);
}

Cdpr.prototype.update = function (dt) {
  this.x += this.vx * dt;
  this.y += this.vy * dt;
  this.x = clamp(this.x, this.ee.w / 2, this.frame.w - this.ee.w / 2);
  this.y = clamp(this.y, this.ee.h / 2, this.frame.h - this.ee.h / 2);
}

/*********** SERIAL PARSING CODE ***************/
function joinRegexWith(regexes, sep) {
  return new RegExp(regexes.map(x => x.source).join(sep.source));
}
const INT_REGEX = /([-+]?\d+)/;
const FLOAT_REGEX = /([-+]?\d*\.?\d*)/;
const XYZ_REGEX = joinRegexWith([FLOAT_REGEX, FLOAT_REGEX, FLOAT_REGEX], /\s+/);
const PREAMBLE_REGEX = new RegExp(INT_REGEX.source + " - " + INT_REGEX.source + ": " + XYZ_REGEX.source + " - " + XYZ_REGEX.source);
const MOTOR_REGEX = joinRegexWith([INT_REGEX, INT_REGEX, FLOAT_REGEX, FLOAT_REGEX], /\s+/);
const LINE_REGEX = new RegExp("^\\s*" + joinRegexWith([PREAMBLE_REGEX, MOTOR_REGEX, MOTOR_REGEX, MOTOR_REGEX, MOTOR_REGEX, INT_REGEX], /\s\|\s/).source + "\\s*$");
// 2000003 - 0: 0.0000 1.5547 0.2296 - 0.0000 1.4315 1.1590	|	0 0 1.8350 0.0000	|	0 0 2.7660 0.0000	|	0 0 2.7839 0.0000	|	0 0 1.6564 -0.0000	|	0

Pose2.fromStrArr = function (arr) {
  return new Pose2(parseFloat(arr[1]), parseFloat(arr[2]), parseFloat(arr[0]));
}
Motor.fromStrArr = function (arr) {
  return new Motor(parseInt(arr[0]), parseInt(arr[1]), parseFloat(arr[2]), parseFloat(arr[3]));
}

function parseLine(line) {
  result = line.match(LINE_REGEX);
  if (result) {
    console.log(result.length);
    return new CdprState(
      controller = {
        t_us: parseInt(result[1]),
        state: parseInt(result[2]),
        est: Pose2.fromStrArr(result.slice(3, 6)),
        set: Pose2.fromStrArr(result.slice(6, 9))
      },
      motors = [
        Motor.fromStrArr(result.slice(9, 13)),
        Motor.fromStrArr(result.slice(13, 17)),
        Motor.fromStrArr(result.slice(17, 21)),
        Motor.fromStrArr(result.slice(21, 25))
      ],
      spray = parseInt(result[25])
    );
  }
  return null;
}

var lastMatch = null;
Cdpr.prototype.parseLogString = function (logString) {
  this.logBuffer += logString;
  if (this.logBuffer.includes("\n")) {
    const lines = this.logBuffer.split("\n");
    this.logBuffer = lines.pop();
    for (const line of lines) {
      this.lastState = parseLine(line);
    }
  }
}
