
const BUTTONS = {
  A: 0, B: 1, X: 2, Y: 3,
  LB: 4, RB: 5, LT: 6, RT: 7,
  BACK: 8, START: 9,
  LSTICK: 10, RSTICK: 11,
  DPAD_UP: 12, DPAD_DOWN: 13, DPAD_LEFT: 14, DPAD_RIGHT: 15,
  HOME: 16,
};

const JOYSTICKS = {
  LHORIZ: 0, LVERT: 1,
  RHORIZ: 2, RVERT: 3,
};

function MyGamepad(gamepad) {
  this.gamepad = gamepad;
  for (const [BUTTON, index] of Object.entries(BUTTONS)) {
    this[BUTTON] = gamepad.buttons[index].pressed;
  }
  this.joyleft = { x: gamepad.axes[JOYSTICKS.LHORIZ], y: gamepad.axes[JOYSTICKS.LVERT] };
  this.joyright = { x: gamepad.axes[JOYSTICKS.RHORIZ], y: gamepad.axes[JOYSTICKS.RVERT] };
}

function GamepadDrawing() {
  function Circle(cx, cy, r) {
    circle = new Path2D();
    circle.arc(cx, cy, r, 0, 2 * Math.PI);
    return circle;
  }
  function Rect(x, y, w, h, r) {
    rect = new Path2D();
    rect.rect(x, y, w, h);
    return rect;
  }

  const LOutline = new Path2D('M220.5 294.5C220.5 294.5 195 294.5 150 294.5C105 294.5 81.5 378.5 49.5 378.5C17.5 378.5 4 363.9 4 317.5C4 271.1 43.5 165.5 55 137.5C66.5 109.5 95.5 92.0001 128 92.0001C154 92.0001 200.5 92.0001 220.5 92.0001'); //  stroke="#C6D9EC" stroke-width="5" stroke-opacity="1"></path>
  const ROutline = new Path2D('M220 294.5C220 294.5 245.5 294.5 290.5 294.5C335.5 294.5 359 378.5 391 378.5C423 378.5 436.5 363.9 436.5 317.5C436.5 271.1 397 165.5 385.5 137.5C374 109.5 345 92.0001 312.5 92.0001C286.5 92.0001 240 92.0001 220 92.0001'); //  stroke="#C6D9EC" stroke-width="5" stroke-opacity="1"></path>

  const LStickOutline = new Circle(163, 238, 37.5);
  const LeftStick = new Circle(163, 238, 28.0);
  const RStickOutline = new Circle(278, 238, 37.5);
  const RightStick = new Circle(278, 238, 28.0);
  const DOutline = new Circle(112, 160, 37.5);
  const BOutline = new Circle(329, 160, 37.5);

  const DUp = new Path2D('M177.669 222.335C180.793 219.21 180.816 213.997 176.868 212.014C176.327 211.743 175.776 211.491 175.215 211.258C172.182 210.002 168.931 209.355 165.648 209.355C162.365 209.355 159.114 210.002 156.081 211.258C155.521 211.491 154.969 211.743 154.429 212.014C150.48 213.997 150.503 219.21 153.627 222.335L159.991 228.698C163.116 231.823 168.181 231.823 171.305 228.698L177.669 222.335Z');
  const DRight = new Path2D('M181.447 249.669C184.571 252.793 189.785 252.816 191.768 248.868C192.039 248.327 192.291 247.776 192.523 247.215C193.78 244.182 194.426 240.931 194.426 237.648C194.426 234.365 193.78 231.114 192.523 228.081C192.291 227.521 192.039 226.969 191.768 226.429C189.785 222.48 184.571 222.503 181.447 225.627L175.083 231.991C171.959 235.116 171.959 240.181 175.083 243.305L181.447 249.669Z');
  const DDown = new Path2D('M154.113 253.447C150.989 256.571 150.966 261.785 154.914 263.767C155.455 264.039 156.006 264.291 156.566 264.523C159.6 265.78 162.85 266.426 166.134 266.426C169.417 266.426 172.667 265.78 175.701 264.523C176.261 264.291 176.812 264.039 177.353 263.767C181.301 261.785 181.279 256.571 178.154 253.447L171.79 247.083C168.666 243.959 163.601 243.959 160.477 247.083L154.113 253.447Z');
  const DLeft = new Path2D('M150.335 226.113C147.21 222.989 141.997 222.966 140.014 226.914C139.743 227.455 139.491 228.006 139.258 228.566C138.002 231.6 137.355 234.85 137.355 238.134C137.355 241.417 138.002 244.667 139.258 247.701C139.491 248.261 139.743 248.812 140.014 249.353C141.997 253.301 147.21 253.279 150.335 250.154L156.698 243.79C159.823 240.666 159.823 235.601 156.698 232.477L150.335 226.113Z');

  const BTop = new Path2D('M340.669 144.335C343.793 141.21 343.816 135.997 339.868 134.014C339.327 133.743 338.776 133.491 338.215 133.258C335.182 132.002 331.931 131.355 328.648 131.355C325.365 131.355 322.114 132.002 319.081 133.258C318.521 133.491 317.969 133.743 317.429 134.014C313.48 135.997 313.503 141.21 316.627 144.335L322.991 150.698C326.116 153.823 331.181 153.823 334.305 150.698L340.669 144.335Z');
  const BRight = new Path2D('M344.447 171.669C347.571 174.793 352.785 174.816 354.768 170.868C355.039 170.327 355.291 169.776 355.523 169.215C356.78 166.182 357.426 162.931 357.426 159.648C357.426 156.365 356.78 153.114 355.523 150.081C355.291 149.521 355.039 148.969 354.768 148.429C352.785 144.48 347.571 144.503 344.447 147.627L338.083 153.991C334.959 157.116 334.959 162.181 338.083 165.305L344.447 171.669Z');
  const BBottom = new Path2D('M317.113 175.447C313.989 178.571 313.966 183.785 317.914 185.767C318.455 186.039 319.006 186.291 319.566 186.523C322.6 187.78 325.85 188.426 329.134 188.426C332.417 188.426 335.667 187.78 338.701 186.523C339.261 186.291 339.812 186.039 340.353 185.767C344.301 183.785 344.279 178.571 341.154 175.447L334.79 169.083C331.666 165.959 326.601 165.959 323.477 169.083L317.113 175.447Z');
  const BLeft = new Path2D('M313.335 148.113C310.21 144.989 304.997 144.966 303.014 148.914C302.743 149.455 302.491 150.006 302.258 150.566C301.002 153.6 300.355 156.851 300.355 160.134C300.355 163.417 301.002 166.668 302.258 169.701C302.491 170.261 302.743 170.812 303.014 171.353C304.997 175.301 310.21 175.279 313.335 172.154L319.698 165.79C322.823 162.666 322.823 157.601 319.698 154.477L313.335 148.113Z');

  const LMeta = new Circle(cx = 185, cy = 162, r = 10);
  const RMeta = new Circle(cx = 259, cy = 162, r = 10);
  const CMeta = new Circle(cx = 220.5, cy = 190, r = 10);

  const L1 = new Rect(111.5, 61.5, 41, 13, 6.5);
  const R1 = new Rect(289.5, 61.5, 41, 13, 6.5);

  const L2 = new Path2D('M152.5 37C152.5 41.1421 149.142 44.5 145 44.5H132C127.858 44.5 124.5 41.1421 124.5 37V16.5C124.5 8.76801 130.768 2.5 138.5 2.5C146.232 2.5 152.5 8.76801 152.5 16.5V37Z');//  fill="rgba(0,0,0,0)" stroke="rgba(0,0,0,1)" stroke-width="5"></path>
  const R2 = new Path2D('M317.5 37C317.5 41.1421 314.142 44.5 310 44.5H297C292.858 44.5 289.5 41.1421 289.5 37V16.5C289.5 8.76801 295.768 2.5 303.5 2.5C311.232 2.5 317.5 8.76801 317.5 16.5V37Z');//  fill="rgba(0,0,0,0)" stroke="rgba(0,0,0,1)" stroke-width="5"></path>

  this.outlines = [LOutline, ROutline, LStickOutline, RStickOutline, DOutline, BOutline];
  this.joysticks = { LeftStick: [LeftStick, 0, 0, false], RightStick: [RightStick, 0, 0, false] };
  this.buttons = {
    DPAD_UP: [DUp, false], DPAD_RIGHT: [DRight, false], DPAD_DOWN: [DDown, false], DPAD_LEFT: [DLeft, false],
    Y: [BTop, false], B: [BRight, false], A: [BBottom, false], X: [BLeft, false],
    BACK: [LMeta, false], START: [RMeta, false], HOME: [CMeta, false],
    LB: [L1, false], RB: [R1, false], LT: [L2, false], RT: [R2, false],
  };
}

GamepadDrawing.prototype.draw = function (ctx) {
  ctx.save();
  // 441 383
  ctx.scale(1 / 441, 1 / 441);
  for (const outline of this.outlines) {
    ctx.stroke(outline);
  }
  for (const [_, joystick] of Object.entries(this.joysticks)) {
    ctx.save();
    ctx.translate(joystick[1] * 20, joystick[2] * 20);
    ctx.stroke(joystick[0]);
    ctx.fillStyle = joystick[3] ? '#000' : '#e0e0e0';
    ctx.fill(joystick[0]);
    ctx.restore();
  }
  for (const [name, obj] of Object.entries(this.buttons)) {
    ctx.save();
    if (name[0] === 'D') {
      ctx.translate(-54, -78);
    }
    if (obj[1]) ctx.fill(obj[0]);
    else ctx.stroke(obj[0]);
    ctx.restore();
  }
  ctx.restore();
}

GamepadDrawing.prototype.update = function (gamepad) {
  for (const [name, button] of Object.entries(this.buttons)) {
    button[1] = gamepad[name];
  }
  this.joysticks.LeftStick[3] = gamepad.LSTICK;
  this.joysticks.LeftStick[1] = gamepad.joyleft.x;
  this.joysticks.LeftStick[2] = gamepad.joyleft.y;
  this.joysticks.RightStick[3] = gamepad.RSTICK;
  this.joysticks.RightStick[1] = gamepad.joyright.x;
  this.joysticks.RightStick[2] = gamepad.joyright.y;
}
