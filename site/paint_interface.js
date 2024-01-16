function hide_disabled_paint_controls() {
  if (PAINT_MODE == "spray") {
    document.getElementById("arm_controls").style.display = "none";
  } else if (PAINT_MODE == "arm") {
    document.getElementById("spray_controls").style.display = "none";
  } else {
    alert("Invalid paint mode: " + PAINT_MODE + "\nShould be 'spray' or 'arm'");
  }
}

function Painter() {
  /**
   * Painter has 8 functions.  They should be called like this:
   *    await Promise.all([
   *      Painter.prep_for_start_painting(),
   *      travel()
   *    ]);
   *
   *    await Painter.start_painting()
   *
   *    await execute_stroke(up_until_T_minus = Painter.time_to_finish_painting(),
   *                         distance_between_refills = Painter.distance_between_refills(),
   *                         refill_callback = Painter.refill);  // refill is async, so please await
   *
   *    await Promise.all([
   *      Painter.finish_painting();
   *      finish_executing_stroke();
   *    ])
   *
   *    await Painter.prep_for_travel();
   *
   *    // ok to move on to next stroke :)
   * 
   *    // after all strokes are done
   *    
   *    await Painter.rest();
   */

  if (PAINT_MODE == "spray") {
    this.prep_for_start_painting = async () => {};
    this.start_painting = Spray.sprayOn.bind(Spray);

    this.distance_between_refills = () => -1;
    this.refill = async () => {};

    this.time_to_finish_painting = Spray.time_to_finish_painting.bind(Spray);
    this.finish_painting = Spray.sprayOff.bind(Spray);
    this.prep_for_travel = Spray.sprayOff.bind(Spray); // this is a safety measure, just in case finish_painting was never called
    this.rest = async () => {};

    Spray.setup();
  } else if (PAINT_MODE == "arm") {
    // Although these functions are named blocking, they are all async!!!
    this.prep_for_start_painting = Arm.do_prep_paint_blocking.bind(Arm);
    this.start_painting = Arm.do_start_paint_blocking.bind(Arm);

    this.distance_between_refills = () => 1.0;
    this.refill = async () => {
      if (await Arm.check_overheat()) {
        await Arm.do_prep_paint();
      }
      await Arm.do_dip_blocking();
    };

    this.time_to_finish_painting = () => 0;
    this.finish_painting = async () => {};
    this.prep_for_travel = Arm.do_prep_paint_blocking.bind(Arm);
    this.rest = Arm.do_move_storage_blocking.bind(Arm);
  } else {
    alert("Invalid paint mode: " + PAINT_MODE + "\nShould be 'spray' or 'arm'");
  }
  hide_disabled_paint_controls();
  this.setGamepadCallbacks = setPainterGamepadCallbacks;
}

function setPainterGamepadCallbacks(gamepad) {
  call_if_true = (callback) =>
    function (state) {
      if (state) callback();
    };

  switch (PAINT_MODE) {
    case "spray":
      gamepad.onchange["RT"] = function (state) {
        Spray.spray(state);
      };
      break;
    case "arm":
      gamepad.onchange["RT"] = call_if_true(painter.start_painting); // go paint
      gamepad.onchange["RB"] = call_if_true(painter.prep_for_start_painting); // prep paint
      gamepad.onchange["LT"] = call_if_true(Arm.do_dip_blocking); // dip paint
      gamepad.onchange["LB"] = call_if_true(Arm.do_move_storage_blocking); // storage
      break;
    // case "revolver":
    //   gamepad.onchange["RT"] = function (state) { cdpr.spray(state); };
    //   gamepad.onchange["LT"] = function (state) { cdpr.spray(state); };
    //   gamepad.onpress["RB"] = function (state) { cdpr.next_color(); };
    //   gamepad.onpress["LB"] = function (state) { cdpr.prev_color(); };
    //   break;
  }
}
