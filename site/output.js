
outputElement = document.getElementById("output");

function println(str, verbosity = 0) {
  if (verbosity <= document.getElementById("verbosity").selectedIndex) {
    outputElement.innerHTML += str + "\n";
    if (outputElement.innerHTML.length > 3000) outputElement.innerHTML = outputElement.innerHTML.slice(outputElement.innerHTML.length - 3000);

    //scroll down to bottom of div
    outputElement.scrollTop = outputElement.scrollHeight;
  }
}

if (!localStorage.verbosity) localStorage.setItem('verbosity', 0);
document.getElementById("verbosity").selectedIndex = localStorage.verbosity;
