const Arm = (function () {
  const socket = new WebSocket("ws://localhost:5910/aoeuPATH");

  socket.onopen = () => {
    console.log("WebSocket connection established");
  };

  socket.onerror = (error) => {
    console.error(`WebSocket error: ${error}`);
  };

  function rpc(method, args = [], kwargs = {}) {
    const request = { method, params: { args, kwargs } };
    return new Promise((resolve, reject) => {
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

  return {
    rpc,
    do_move_home, do_dip, do_prep_paint, do_start_paint,
  };
})();
