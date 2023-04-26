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

const Arm = (function () {
  const socket = new WebSocket("ws://localhost:5910/aoeuPATH");

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

  const do_move_home = async () => await rpc("arm.do_move_home");
  const do_dip = async () => await rpc("arm.do_dip");
  const do_prep_paint = async () => await rpc("arm.do_prep_paint");
  const do_start_paint = async () => await rpc("arm.do_start_paint");

  const do_move_home_blocking = async () => await rpc_blocking("arm.do_move_home");
  const do_dip_blocking = async () => await rpc_blocking("arm.do_dip");
  const do_prep_paint_blocking = async () => await rpc_blocking("arm.do_prep_paint");
  const do_start_paint_blocking = async () => await rpc_blocking("arm.do_start_paint");

  return {
    rpc,
    do_move_home,
    do_dip,
    do_prep_paint,
    do_start_paint,
    do_move_home_blocking,
    do_dip_blocking,
    do_prep_paint_blocking,
    do_start_paint_blocking,
  };
})();
