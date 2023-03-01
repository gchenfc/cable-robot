outputElement = document.getElementById("output");
outputSetpointElement = document.getElementById("setpointOut");
outputTrackerElement = document.getElementById("trackerOut");
outputWaypointElement = document.getElementById("waypointOut");

if (!localStorage.verbosity) localStorage.setItem('verbosity', 0);
document.getElementById("verbosity").selectedIndex = localStorage.verbosity;

function println(str, verbosity = 0) {
  if (verbosity <= document.getElementById("verbosity").selectedIndex) {
    outputElement.innerHTML += str + "\n";
    if (outputElement.innerHTML.length > 3000) outputElement.innerHTML = outputElement.innerHTML.slice(outputElement.innerHTML.length - 3000);

    //scroll down to bottom of div
    outputElement.scrollTop = outputElement.scrollHeight;
  }
}

function printlnSetpoint(str, verbosity = 0) {
  if (verbosity <= document.getElementById("verbosity").selectedIndex) {
    outputSetpointElement.innerHTML += str + "\n";
    if (outputSetpointElement.innerHTML.length > 500) outputSetpointElement.innerHTML = outputSetpointElement.innerHTML.slice(outputSetpointElement.innerHTML.length - 500);

    //scroll down to bottom of div
    outputSetpointElement.scrollTop = outputSetpointElement.scrollHeight;
  }
}

function printlnTracker(str, verbosity = 0) {
  if (verbosity <= document.getElementById("verbosity").selectedIndex) {
    outputTrackerElement.innerHTML += str + "\n";
    if (outputTrackerElement.innerHTML.length > 500) outputTrackerElement.innerHTML = outputTrackerElement.innerHTML.slice(outputTrackerElement.innerHTML.length - 500);

    //scroll down to bottom of div
    outputTrackerElement.scrollTop = outputTrackerElement.scrollHeight;
  }
}

function printlnWaypoint(str, verbosity = 0) {
  if (verbosity <= document.getElementById("verbosity").selectedIndex) {
    outputWaypointElement.innerHTML += str + "\n";
    if (outputWaypointElement.innerHTML.length > 500) outputWaypointElement.innerHTML = outputWaypointElement.innerHTML.slice(outputWaypointElement.innerHTML.length - 500);

    //scroll down to bottom of div
    outputWaypointElement.scrollTop = outputWaypointElement.scrollHeight;
  }
}
