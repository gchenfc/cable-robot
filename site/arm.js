// A mutex that immediately returns false if it fails to acquire the lock
const locked = 1;
const unlocked = 0;
class MutexImmediateFail {
  /**
   * Instantiate Mutex.
   * If opt_sab is provided, the mutex will use it as a backing array.
   * @param {ArrayBuffer} opt_sab Optional ArrayBuffer.
   */
  constructor(opt_sab) {
    this._sab = opt_sab || new ArrayBuffer(4);
    this._mu = new Int32Array(this._sab);
  }

  lock() {
    return Atomics.compareExchange(this._mu, 0, unlocked, locked) == unlocked;
  }

  unlock() {
    if (Atomics.compareExchange(this._mu, 0, locked, unlocked) != locked) {
      throw new Error(
        "Mutex is in inconsistent state: unlock on unlocked Mutex."
      );
    }
  }
}

// Usage:
//  const unlock = await mutex.lock();
//  // do work
//  unlock();
function Mutex() {
  let current = Promise.resolve();
  this.lock = () => {
      let _resolve;
      const p = new Promise(resolve => {
          _resolve = () => resolve();
      });
      // Caller gets a promise that resolves when the current outstanding
      // lock resolves
      const rv = current.then(() => _resolve);
      // Don't allow the next request until the new promise is done
      current = p;
      // Return the new promise
      return rv;
  };
}

// const mutex = new Mutex();
const armSerialMutex = new MutexImmediateFail();
// const armSerialMutex = new Mutex();

const createArm = function () {
  let socket = new WebSocket("ws://localhost:5910/aoeuPATH");
  
  socket.onopen = () => {
    console.log("Arm WebSocket connection established");
  };
  
  socket.onerror = (error) => {
    console.error(`Arm WebSocket error: ${error}`);
  };

  function rpc(method, args = [], kwargs = {}) {
    const request = { method, params: { args, kwargs } };
    return new Promise(async (resolve, reject) => {


      if (false) {
        resolve("no arm connected, automatically resolving");
      }



      // Try to acquire the mutex, but on fail, reject the Promise with an error message
      console.log("about to try to acquire the mutex!!!");
      if (armSerialMutex.lock() === false) {
        reject(new Error("Arm RPC function is already in use"));
        return;
      }
      console.log("Success!")
      try {
        // Send the RPC request to the server
        socket.send(JSON.stringify(request));

        // Wait for the response from the server
        socket.onmessage = (event) => {
          armSerialMutex.unlock();
          const response = JSON.parse(event.data);
          if (response.error) {
            // If the response contains an error object, throw it as an exception
            reject(new Error(response.error));
          } else {
            // Otherwise, resolve with the result object
            resolve(response.result);
          }
        };

        // Sleep for 5 seconds as a timeout
        // await new Promise(r => setTimeout(r, 5000));
      } finally {
        // armSerialMutex.unlock();
      }
    });
  }
  function rpc_blocking(method, args = [], kwargs = {}) {
    // just shove it on the queue
    const request = { method, params: { args, kwargs } };
    return new Promise(async (resolve, reject) => {

      if (false) {
        resolve("no arm connected, automatically resolving");
      }


      // Send the RPC request to the server
      socket.send(JSON.stringify(request));

      // Wait for the response from the server
      socket.onmessage = (event) => {
        const response = JSON.parse(event.data);
        if (response.error) {
          // If the response contains an error object, throw it as an exception
          reject(new Error(response.error));
        } else {
          // Otherwise, resolve with the result object
          resolve(response.result);
        }
      };
    });
  }
  async function check_overheat() {
    const SOFT_LIMIT = 65;
    const DEADBAND = 5;  // re-enables when SOFT_LIMIT - DEADBAND
    var temps = await read_temperatures_blocking();
    temps[temps.length - 1] += 7;  // Make this motor stop earlier
    // Hard limit is 70C, so we set a soft limit at 65C.
    if (temps.some((t) => t > SOFT_LIMIT)) {
      do_move_storage_blocking();
      document.getElementById("arm_error_div").style.display = "block";
      document.getElementById("arm_error").textContent =
        `Close to overheating.  Pausing.  ` +
        `Last checked at ${new Date().toLocaleTimeString()}.  ` + 
        `Motor Temps: (${temps}) > ${SOFT_LIMIT} (restart @ ${SOFT_LIMIT - DEADBAND}, hard @ 70)`;
      // alert("Arm is close to overheating.  Pausing for a bit.");
      await new Promise((r) => setTimeout(r, 10000));
    }
    else {
      return false;
    }
    temps = await read_temperatures_blocking();
    temps[temps.length - 1] += 7;  // Make this motor stop earlier
    while (temps.some((t) => t > (SOFT_LIMIT - DEADBAND))) {
      document.getElementById("arm_error").textContent =
      `Close to overheating.  Pausing.  ` +
      `Last checked at ${new Date().toLocaleTimeString()}.  ` + 
      `Motor Temps: (${temps}) > ${SOFT_LIMIT} (restart @ ${SOFT_LIMIT - DEADBAND}, hard @ 70)`;
      await new Promise((r) => setTimeout(r, 10000));
      temps = await read_temperatures_blocking();
      temps[temps.length - 1] += 7;  // Make this motor stop earlier
    }
    document.getElementById("arm_error_div").style.display = "none";
    return true;
  }

  const do_move_home = async () => await rpc("arm.do_move_home");
  const do_move_storage = async () => await rpc("arm.do_move_storage");
  const do_dip = async () => await rpc("arm.do_dip");
  const do_prep_paint = async () => await rpc("arm.do_prep_paint");
  const do_start_paint = async () => await rpc("arm.do_start_paint");

  const do_move_home_blocking = async () => await rpc_blocking("arm.do_move_home");
  const do_move_storage_blocking = async () => await rpc_blocking("arm.do_move_storage");
  const do_dip_blocking = async () => await rpc_blocking("arm.do_dip");
  const do_prep_paint_blocking = async () => await rpc_blocking("arm.do_prep_paint");
  const do_start_paint_blocking = async () => await rpc_blocking("arm.do_start_paint");

  const enable_all_blocking = async () => await rpc_blocking("arm.enable_all");
  const disable_all_blocking = async () => await rpc_blocking("arm.disable_all");

  const cur_pose_blocking = async () => await rpc_blocking("arm.cur_pose");
  const cur_point_blocking = async () => await rpc_blocking("arm.cur_point");
  const cur_canvas_pose_blocking = async () => await rpc_blocking("arm.cur_canvas_pose");
  const cur_canvas_point_blocking = async () => await rpc_blocking("arm.cur_canvas_point");

  const joint_angles_deg_blocking = async () => await rpc_blocking("arm.joint_angles_deg");
  const joint_angles_string_blocking = async () => await rpc_blocking("arm.joint_angles_string");
  const go_to_blocking_blocking = async (goal) => await rpc_blocking("arm.go_to_blocking", args=[goal]);
  const go_to_pose_blocking_blocking = async (goal) => await rpc_blocking("arm.go_to_pose_blocking", args=[goal]);
  const go_to_canvas_blocking_blocking = async (goal) => await rpc_blocking("arm.go_to_canvas_blocking", args=[goal]);
  const reached_goal_blocking = async (goal) => await rpc_blocking("arm.reached_goal", args=[goal]);
  const read_temperatures_blocking = async () => await rpc_blocking("arm.read_temperatures");
  const ping_all_blocking = async () => await rpc_blocking("arm.ping_all");

  
  // Low-level, recommend not using
  const read_all_joint_angles_deg_blocking = async () => await rpc_blocking("arm.read_all_joint_angles_deg");
  const read_all_blocking = async (addr) => await rpc_blocking("arm.read_all", args=[addr]);
  const command_angles_deg_blocking = async (angles) => await rpc_blocking("arm.command_angles_deg", (args = angles));
  const command_angle_blocking = async (id, angle) => await rpc_blocking("arm.command_angle", args=[id, angle]);
  const set_speed_blocking = async (speed_counts) => await rpc_blocking("arm.set_speed", args=[speed_counts]);
  const set_speeds_blocking = async (speeds_counts) => await rpc_blocking("arm.set_speeds", args=[speeds_counts]);

  // Below functions are untested
  const execute_joint_path_blocking = async (path) => await rpc_blocking("arm.execute_joint_path", args=[path]);
  const write_all_blocking = async (addr, value) => await rpc_blocking("arm.write_all", args=[addr, value]);
  const set_compliance_margins_blocking = async (margin) => await rpc_blocking("arm.set_compliance_margins", args=[margin]);
  const set_compliance_slopes_blocking = async (slope) => await rpc_blocking("arm.set_compliance_slopes", args=[slope]);
  // ----------------------------

  return {
    get socket() {
      return socket;
    },
    set socket(value) {
      socket = value;
    },
    rpc,
    do_move_home,
    do_move_storage,
    do_dip,
    do_prep_paint,
    do_start_paint,
    do_move_home_blocking,
    do_move_storage_blocking,
    do_dip_blocking,
    do_prep_paint_blocking,
    do_start_paint_blocking,
    check_overheat,

    enable_all_blocking, // void -> # bytes written
    disable_all_blocking, // void -> # bytes written

    cur_pose_blocking, // void -> 4x4 homogeneous transformation matrix
    cur_point_blocking, // void -> [x, y, z]
    cur_canvas_pose_blocking, // void -> 4x4 homogeneous transformation matrix
    cur_canvas_point_blocking, // void -> [x, y, z]
    joint_angles_deg_blocking, // void -> [q1, q2, q3, q4, q5]
    joint_angles_string_blocking, // void -> string
    read_temperatures_blocking, // void -> [t0, t1, t2, t3, t4, t5]
    ping_all_blocking, // void -> [s0, s1, s2, s3, s4, s5]

    go_to_blocking_blocking, // [q1, q2, q3, q4, q5] -> void
    go_to_pose_blocking_blocking, // 4x4 homogeneous transformation matrix -> bool
    go_to_canvas_blocking_blocking, // [x, y, z] -> bool
    reached_goal_blocking, // [q1, q2, q3, q4, q5] -> bool
    execute_joint_path_blocking,

    // Low-level functions, recommend not using
    write_all_blocking,
    read_all_blocking, // addr -> 6-array of {id: #, error: #, value: #}
    set_speed_blocking, // int -> # bytes written, arg is same speed for all motors
    set_speeds_blocking, // [v0, v1, v2, v3, v4, v5] -> # bytes written
    set_compliance_margins_blocking,
    set_compliance_slopes_blocking,
    read_all_joint_angles_deg_blocking, // void -> [q1, q2, q3, q4, q5, q6]
    command_angle_blocking, // command_angle_blocking(id, angle_deg)
    command_angles_deg_blocking, // command_angles_deg_blocking([q1, q2, q3, q4, q5])
  };
};

let Arm = createArm();

// Logic to re-connect Arm on disconnect
setInterval(() => {
  if (Arm.socket.readyState !== WebSocket.OPEN) {
    console.log("Arm WebSocket connection closed, attempting to reconnect");
    Arm.socket = new WebSocket("ws://localhost:5910/aoeuPATH");
    Arm.socket.onopen = () => {
      console.log("Arm WebSocket connection established");
    };
    Arm.socket.onerror = (error) => {
      console.error(`Arm WebSocket error: ${error}`);
    };
  }
}, 1000);
