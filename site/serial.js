// MODIFIED FROM https://github.com/mmiscool/serialTerminal.com


var port, textEncoder, writableStreamClosed, writer, historyIndex = -1, reader;
const lineHistory = [];
var readStreamDisplay, readStreamLogfile;

async function attemptAutoconnect() {
  if (!document.getElementById("autoConnect").checked) return;
  return await navigator.serial.getPorts().then((ports) => {
    for (const port_ of ports) {
      info = port_.getInfo();
      if (info.usbProductId === 1163 && info.usbVendorId === 5824) {
        port = port_;
        connectSerialPort();
        return true;
      }
    }
  });
}

async function connectSerial() {
  // Prompt user to select any serial port.
  if (reader && reader.locked) await closeSerial();
  if (document.getElementById("autoConnect").checked) {
    if (await attemptAutoconnect()) return;
  }
  try {
    port = await navigator.serial.requestPort();
    connectSerialPort();
  } catch {
    alert("Serial Connection Failed");
  }
}
async function connectSerialPort() {
  await port.open({ baudRate: 9600 });
  // [readStreamDisplay, readStreamLogfile] = port.readable.tee();
  readStreamDisplay = port.readable;
  listenToPort().then(() => {
    console.log("Done listening");
  }).catch((e) => { console.log("Error closing serial port!", e); });

  textEncoder = new TextEncoderStream();
  writableStreamClosed = textEncoder.readable.pipeTo(port.writable);

  writer = textEncoder.writable.getWriter();
  cdpr.sendStartupMessages();
}
async function closeSerial() {
  writer.close();
  await writableStreamClosed;
  await reader.cancel(); // listenToPort will handle closing the actual serial device.
}

async function sendCharacterNumber() {
  document.getElementById("lineToSend").value = String.fromCharCode(document.getElementById("lineToSend").value);
}

async function sendSerialLine() {
  dataToSend = document.getElementById("lineToSend").value;
  lineHistory.unshift(dataToSend);
  historyIndex = -1; // No history entry selected
  if (document.getElementById("addLine").checked == true) dataToSend = dataToSend + "\r\n";
  if (document.getElementById("echoOn").checked == true) appendToTerminal("> " + dataToSend);
  await writer.write(dataToSend);
  document.getElementById("lineToSend").value = "";
  //await writer.releaseLock();
}

async function listenToPort() {
  const textDecoder = new TextDecoderStream();
  const readableStreamClosed = readStreamDisplay.pipeTo(textDecoder.writable);
  reader = textDecoder.readable.getReader();

  // Listen to data coming from the serial device.
  try {
    while (true) {
      const { value, done } = await reader.read();
      if (done) {
        // Allow the serial port to be closed later.
        break;
      }
      // value is a string.
      cdpr.parseLogString(value, appendToTerminal);
    }
  } finally {
    try { await readableStreamClosed; } catch (e) { console.log("error closing readableStream", e); }
    reader.releaseLock();
  }
  return await port.close();
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
