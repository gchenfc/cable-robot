function Drawing() {
  this.path = new Path2D();
}

Drawing.prototype.update = function (x, y, pressed) {
  if (pressed) {
    this.path.lineTo(x, y);
  } else {
    this.path.moveTo(x, y);
  }
}

Drawing.prototype.draw = function(ctx) {
  for (var thickness = 0.02; thickness <= 0.2; thickness += 0.02) {
    ctx.lineWidth = thickness;
    ctx.strokeStyle = "#f002";
    ctx.lineCap = "round";
    ctx.stroke(this.path);
  }
}
