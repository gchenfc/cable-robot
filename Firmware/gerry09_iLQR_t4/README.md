To run unit tests, do:

```bash
mkdir build
cd build
cmake ..
make check
```

Running unit tests will not build the Arduino project - please compile/upload using Teensyduino.

## Annoying Quirks

### Includes

For including with relative paths, Arduino uses the path relative to the file instead of relative to the project root.

### Unit tests
Arduino isn't smart enough to not compile unrelated cpp files so you need to enclose unit tests intended to be run on the computer with
```c++
#ifndef ARDUINO
...

#endif
```
