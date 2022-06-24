const cdpr_canvas = document.getElementById("cdpr_canvas");
const cdpr_ctx = cdpr_canvas.getContext("2d");

const gamepad_canvas = document.getElementById("gamepad_canvas");
const gamepad_ctx = gamepad_canvas.getContext("2d");

var gamepad_idx = null;
const cdpr = new Cdpr(new Dims(3.0, 2.3), new Dims(0.2, 0.3));
const drawing = new Drawing();
const gamepad = new MyGamepad();
const gamepad_drawing = new GamepadDrawing();

var animation_interval, cdpr_interval, serial_interval;

function init() {
  // register callbacks
  gamepad.onpress["A"] = function () { cdpr.clearErrors(); };
  gamepad.onpress["X"] = function () { cdpr.setMode(Mode.HOLD); };
  gamepad.onpress["Y"] = function () { cdpr.setMode(Mode.TRACKING); };
  gamepad.onchange["RT"] = function (state) { cdpr.send(`s${state ? 1 : 0}`); };
  gamepad.onchange["LT"] = function (state) { cdpr.send(`s${state ? 1 : 0}`); };
  // start updates
  cdpr_interval = setInterval(function () { cdpr.update(1 / 150); }, 1000 / 150);
  animation_interval = setInterval(draw, 1000 / 30);
  // Connect serial
  attemptAutoconnect();
  // initial draw
  draw();
}

function draw() {
  draw_drawing();
  draw_cdpr();
  draw_gamepad();
}
function draw_drawing() {
  cdpr_ctx.clearRect(0, 0, cdpr_canvas.width, cdpr_canvas.height);
  cdpr_ctx.save();
  cdpr_ctx.drawAtLoc(0, cdpr_canvas.height, 100, drawing, -100);
  cdpr_ctx.restore();
}
function draw_cdpr() {
  // cdpr_ctx.clearRect(0, 0, cdpr_canvas.width, cdpr_canvas.height);
  cdpr_ctx.save();
  cdpr_ctx.drawAtLoc(0, cdpr_canvas.height, 100, cdpr, -100);
  cdpr_ctx.restore();
}
function draw_gamepad() {
  gamepad_ctx.clearRect(0, 0, gamepad_canvas.width, gamepad_canvas.height);
  gamepad_ctx.save();
  if (gamepad_idx !== null) {
    gamepad.update(navigator.getGamepads()[gamepad_idx]);
    cdpr.setControls(0.45 / Math.sqrt(2) * gamepad.joyleft.x, 0.45 / Math.sqrt(2) * gamepad.joyleft.y);
    drawing.update(cdpr.x, cdpr.y, gamepad.LT || gamepad.RT);
    gamepad_drawing.update(gamepad);
  } else {
    gamepad_ctx.fillStyle = 'rgb(0, 0, 0)';
    gamepad_ctx.font = "40px Serif";
    gamepad_ctx.textBaseline = "top";
    gamepad_ctx.fillText("Please Connect Gamepad", 10, 10);
    gamepad_ctx.font = "12px Serif";
    gamepad_ctx.fillStyle = '#0004'
    gamepad_ctx.fillRect(0, 0, 300, 260);
  }
  gamepad_ctx.drawAtLoc(0, 0, 300, gamepad_drawing);
  gamepad_ctx.restore();
}

CanvasRenderingContext2D.prototype.drawAtLoc = function (x, y, scalex, obj, scaley = scalex) {
  this.save();
  this.translate(x, y);
  this.scale(scalex, scaley);
  obj.draw(this);
  this.restore();
}

window.addEventListener("gamepadconnected", (event) => {
  console.log("gamepad connected");
  console.log(event.gamepad);
  gamepad_idx = event.gamepad.index;
});

window.addEventListener("gamepaddisconnected", (event) => {
  gamepad_idx = null;
  console.log("gamepad disconnected");
  cdpr.setControls(0, 0);
});
