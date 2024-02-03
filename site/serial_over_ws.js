/********************* WEBSOCKET CODE ****************************/

const SERIAL_HOST = 'localhost';
const SERIAL_PORT = 8765;

let serial_connected = false;
let websocket = new WebSocket('ws://' + SERIAL_HOST + ':' + SERIAL_PORT);

websocket.onerror = function (event) {
  console.log('WebSocket error: ', event);
  serial_connected = false;
};
websocket.onopen = function (event) {
  console.log("Serial Websocket Connected!!!");
  serial_connected = true;
}
websocket.onclose = function (event) {
  if (serial_connected) {
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

/********************* Serial Interface Code ***************************/
var port, textEncoder, writableStreamClosed, writer = {write: ()=>{}}, historyIndex = -1, reader;
const lineHistory = [];
var readStreamDisplay, readStreamLogfile;

async function getPorts() {
  const resp = await blocking_send({ command: "getPorts" });
  if (resp['error'] != null) {
    alert("Requesting Serial Ports List Failed: " + resp['error']);
    return;
  }
  return resp['ports'];
}
async function requestPort() {
  const resp = await blocking_send({
    command: "open",
    port: document.getElementById("port").value,
    baudRate: 1000000, // shouldn't matter
    timeout: 1,
  });
  console.log(resp);
}

async function attemptAutoconnect() {
  if (!document.getElementById("autoConnect").checked) return;
  if (!serial_connected) {
    setTimeout(attemptAutoconnect, 500);  // Try again in 500ms
    return;
  }
  return await getPorts().then((ports) => {
    for (const port_ of ports) {
      // if ((port_.pid === 1163 || port_.pid === 1164) && port_.vid === 5824) {
      console.log(port_)
      if (port_.name === "ttyACM0") {
        connectSerialPort(port_.device);
        return true;
      }
    }
  });
}

async function connectSerial() {
  // Prompt user to select any serial port.
  if (document.getElementById("autoConnect").checked) {
    if (await attemptAutoconnect()) return;
  }
  try {
    // connectSerialPort();
  } catch {
    alert("Serial Connection Failed");
  }
}
async function connectSerialPort(port_path) {
  console.log('Serial Websocket ', port_path);
  const resp = await blocking_send({
    command: "open",
    port: port_path,
    baudRate: 1000000, // shouldn't matter
    timeout: 1,
  });

  if (resp['error'] != null || resp.status != 'connected') {
    alert("Serial Connection Failed: " + resp['error']);
    return;
  }
  console.log(resp);

  await listenToPort();

  writer = {
    write: async (data) => {
      return await blocking_send({ command: "write", data: data });
    },
  };

  // sendStartupmessages
}

async function closeSerial() {
  const resp = await blocking_send({ command: "close" });

  if (resp['error'] != null || resp.status != 'disconnected') {
    alert("Serial Disconnect Failed: " + resp.error);
    return;
  }

  // TODO: figure out how to close the event listener.
  // websocket.removeEventListener
}

async function sendSerialLine() {
  dataToSend = document.getElementById("lineToSend").value;
  lineHistory.unshift(dataToSend);
  historyIndex = -1; // No history entry selected
  if (document.getElementById("addLine").checked == true) dataToSend = dataToSend + "\r\n";
  if (document.getElementById("echoOn").checked == true) appendToTerminal("> " + dataToSend);
  await writer.write(dataToSend);
  document.getElementById("lineToSend").value = "";
}

async function listenToPort() {
  websocket.addEventListener('message', event => {
    const data = JSON.parse(event.data);
    if (data['command'] != 'data stream') return;
    cdpr.parseLogString(data.data, appendToTerminal);
  });

  const resp = await blocking_send({ command: "streaming_read" });
  if (resp['error'] != null || resp.status != 'streaming') {
    alert("Streaming Read Failed: " + resp.error);
    return;
  }
}

const serialResultsDiv = document.getElementById("serialResults");

async function appendToTerminal(newStuff) {
  logText(newStuff);
  serialResultsDiv.innerHTML += newStuff;
  if (serialResultsDiv.innerHTML.length > 12000) serialResultsDiv.innerHTML = serialResultsDiv.innerHTML.slice(serialResultsDiv.innerHTML.length - 12000);

  //scroll down to bottom of div
  serialResultsDiv.scrollTop = serialResultsDiv.scrollHeight;
}

function scrollHistory(direction) {
  // Clamp the value between -1 and history length
  historyIndex = Math.max(Math.min(historyIndex + direction, lineHistory.length - 1), -1);
  if (historyIndex >= 0) {
    document.getElementById("lineToSend").value = lineHistory[historyIndex];
  } else {
    document.getElementById("lineToSend").value = "";
  }
}

document.getElementById("lineToSend").addEventListener("keyup", async function (event) {
  if (event.keyCode === 13) {
    sendSerialLine();
  } else if (event.keyCode === 38) { // Key up
    scrollHistory(1);
  } else if (event.keyCode === 40) { // Key down
    scrollHistory(-1);
  }
})

document.getElementById("addLine").checked = (localStorage.addLine == "false" ? false : true);
document.getElementById("echoOn").checked = (localStorage.echoOn == "false" ? false : true);
document.getElementById("autoConnect").checked = (localStorage.autoConnect == "false" ? false : true);
