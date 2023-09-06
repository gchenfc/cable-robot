/********************* WEBSOCKET CODE ****************************/
const HOST = 'localhost';
const PORT = 8765;

let connected = false;
let websocket = new WebSocket('ws://' + HOST + ':8765/');

websocket.onerror = function (event) {
  console.log('WebSocket error: ', event);
  connected = false;
};
websocket.onopen = function (event) {
  console.log("Serial Websocket Connected!!!", event);
  connected = true;
}
websocket.onclose = function (event) {
  if (connected) {
    console.log("Serial Websocket Disconnected!!!");
  }
}
websocket.onmessage = function (event) {
  got_ws_message(event);
}

async function blocking_read(command = null) {
  return new Promise((resolve) => {
    const messageHandler = (event) => {
      const resp = JSON.parse(event.data);
      if (command != null && resp["command"] != command) return;
      websocket.removeEventListener("message", messageHandler); // Remove the event listener
      resolve(resp);
    };

    websocket.addEventListener("message", messageHandler);
  });
}

async function blocking_send(object) {
  websocket.send(JSON.stringify(object));
  return await blocking_read(object['command']);
}

function got_ws_message(event) {
  // console.log(event);
}

