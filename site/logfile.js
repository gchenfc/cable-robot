var logfileHandle, logfileStream, logfileStreamPipe, logging = false;

async function openFile() {
  logfileHandle = await window.showSaveFilePicker(options = { suggestedName: "data.txt" });
  document.getElementById("beginLogging").disabled = false;
}

async function beginLogging() {
  if (document.getElementById("append").checked) {
    const fileData = await logfileHandle.getFile();
    logfileStream = await logfileHandle.createWritable({ keepExistingData: true });
    await logfileStream.seek(fileData.size);
  } else {
    logfileStream = await logfileHandle.createWritable();
  }
  // logfileStreamPipe = readStreamLogfile.pipeTo(logfileStream);
  logging = true;
  document.getElementById("beginLogging").disabled = true;
  document.getElementById("finishLogging").disabled = false;
  document.getElementById("beginLogging").hidden = true;
  document.getElementById("finishLogging").hidden = false;
}

async function logText(text) {
  if (logging) {
    logfileStream.write(text);
  }
}

async function finishLogging() {
  logging = false;
  document.getElementById("beginLogging").disabled = false;
  document.getElementById("finishLogging").disabled = true;
  document.getElementById("beginLogging").hidden = false;
  document.getElementById("finishLogging").hidden = true;

  await logfileStream.close();
  console.log("wrote logfile to disk");
}

document.getElementById("append").checked = (localStorage.append == "false" ? false : true);
