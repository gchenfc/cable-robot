const DEFAULT_SPRAY_TRANSITION_ON_TIME_S = 0.7;
const DEFAULT_SPRAY_TRANSITION_OFF_TIME_S = 0.1;
const DEFAULT_SERVO_START_POS = 0;
const DEFAULT_SERVO_END_POS = 120;

const Spray = {
  transition_on_time_s: DEFAULT_SPRAY_TRANSITION_ON_TIME_S,
  transition_off_time_s: DEFAULT_SPRAY_TRANSITION_OFF_TIME_S,
  is_spraying: false,

  sprayOn: async function () {
    cdpr.send("s1");
    if (this.is_spraying) return; // no await needed
    this.is_spraying = true;
    setSprayToggle();
    await new Promise((r) => setTimeout(r, this.transition_on_time_s * 1000));
  },
  sprayOff: async function () {
    cdpr.send("s0");
    if (!this.is_spraying) return; // no await needed
    this.is_spraying = false;
    setSprayToggle();
    await new Promise((r) => setTimeout(r, this.transition_off_time_s * 1000));
  },
  time_to_finish_painting: function () {
    return this.transition_off_time_s;
  },
  setServoStartPos: (pos) => {
    cdpr.send(`s>${pos}`);
  },
  setServoEndPos: (pos) => {
    cdpr.send(`s<${pos}`);
  },
  spray: async function (on) {
    if (on) {
      await this.sprayOn();
    } else {
      await this.sprayOff();
    }
  },
  toggleSpray: async function () {
    await this.spray(!this.is_spraying);
  },
  setup: function () {
    this.transition_on_time_s = DEFAULT_SPRAY_TRANSITION_ON_TIME_S;
    this.transition_off_time_s = DEFAULT_SPRAY_TRANSITION_OFF_TIME_S;
    this.is_spraying = false;

    setTimeout(() => {
      this.setServoStartPos(DEFAULT_SERVO_START_POS);
      this.setServoEndPos(DEFAULT_SERVO_END_POS);
      document.getElementById("startPos").value = DEFAULT_SERVO_START_POS;
      document.getElementById("endPos").value = DEFAULT_SERVO_END_POS;
    }, 100); // give some delay for the cdpr to connect
    addEventListeners();
  },
};

function setSprayToggle() {
  var button = document.getElementById("sprayToggle");
  if (!Spray.is_spraying) {
    button.classList.remove("active");
    button.innerText = "Spray Off";
  } else {
    button.classList.add("active");
    button.innerText = "Spray On";
  }
}

function addEventListeners() {
  document
    .getElementById("startPos")
    .addEventListener("input", function (event) {
      var startPos = event.target.value;
      document.getElementById("startPosLabel").textContent = startPos;
      Spray.setServoStartPos(startPos);
    });

  document.getElementById("endPos").addEventListener("input", function (event) {
    var endPos = event.target.value;
    document.getElementById("endPosLabel").textContent = endPos;
    Spray.setServoEndPos(endPos);
  });
}
