const cdpr_canvas = document.getElementById("cdpr_canvas");
const cdpr_ctx = cdpr_canvas.getContext("2d");

const gamepad_canvas = document.getElementById("gamepad_canvas");
const gamepad_ctx = gamepad_canvas.getContext("2d");

var gamepad_idx = null;
const cdpr = new Cdpr(new Dims(3.0, 2.3), new Dims(0.2, 0.3));
const drawing = new Drawing();
const gamepad_drawing = new GamepadDrawing();

var animation_interval, cdpr_interval, serial_interval;

function init() {
  cdpr_interval = setInterval(function () { cdpr.update(1 / 60); }, 1000 / 60);
  animation_interval = setInterval(draw, 1000 / 30);
  serial_interval = setInterval(function () {appendToTerminal((new Date()).getUTCMilliseconds() + 'dummy data dummy data dummy data dummy data dummy data dummy data dummy data dummy data dummy data\n');}, 100);
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
  cdpr_ctx.drawAtLoc(0, 0, 100, drawing);
  cdpr_ctx.restore();
}
function draw_cdpr() {
  // cdpr_ctx.clearRect(0, 0, cdpr_canvas.width, cdpr_canvas.height);
  cdpr_ctx.save();
  cdpr_ctx.drawAtLoc(0, 0, 100, cdpr);
  cdpr_ctx.restore();
}
function draw_gamepad() {
  gamepad_ctx.clearRect(0, 0, gamepad_canvas.width, gamepad_canvas.height);
  gamepad_ctx.save();
  if (gamepad_idx !== null) {
    const gamepad = new MyGamepad(navigator.getGamepads()[gamepad_idx]);
    cdpr.setControls(gamepad.joyleft.x, gamepad.joyleft.y);
    drawing.update(cdpr.x, cdpr.y, gamepad.A);
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

CanvasRenderingContext2D.prototype.drawAtLoc = function (x, y, scale, obj) {
  this.save();
  this.translate(x, y);
  this.scale(scale, scale);
  obj.draw(this);
  this.restore();
}

window.addEventListener("gamepadconnected", (event) => {
  console.log("gamepad connected");
  console.log(event.gamepad);
  gamepad_idx = event.gamepad.index;
});

window.addEventListener("gamepaddisconnected", (event) => {
  console.log("gamepad disconnected");
  gamepad_idx = null;
});
