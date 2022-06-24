function Dims(w, h) {
  this.w = w;
  this.h = h;
}

function Corners(dims) {
  return [[dims.w, 0], [dims.w, dims.h], [0, dims.h], [0, 0]];
}

function Cdpr(frameDims, eeDims) {
  this.frame = frameDims;
  this.ee = eeDims;
  this.x = this.frame.w / 2;
  this.y = this.frame.h / 2;
  this.vx = 0;
  this.vy = 0;
}

function zip(a, b) {
  return a.map(function (e, i) {
    return [e, b[i]];
  });
}

Cdpr.prototype.draw = function (ctx) {
  x = this.x - this.ee.w / 2;
  y = this.y - this.ee.h / 2;

  // Draw frame & ee
  ctx.lineWidth = 0.06;
  ctx.strokeRect(0, 0, this.frame.w, this.frame.h);
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

Cdpr.prototype.setControls = function (vx, vy) {
  this.vx = vx;
  this.vy = vy;
}

Cdpr.prototype.update = function (dt) {
  this.x += this.vx * dt;
  this.y += this.vy * dt;
}
