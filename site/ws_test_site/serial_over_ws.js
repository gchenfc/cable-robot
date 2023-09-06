var port, textEncoder, writableStreamClosed, writer, historyIndex = -1, reader;
const lineHistory = [];
var readStreamDisplay, readStreamLogfile;

async function getPorts() {
  return await blocking_send({ command: "getPorts" });
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
  return await getPorts().then((ports) => {
    for (const port_ of ports) {
      if (port_.pid === 1163 && port_.vid === 5824) {
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
    console.log(event.data);
    // cdpr.parseLogString(event.data, appendToTerminal);
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
