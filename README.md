# Dynamixel AX-12A C++ Library

From: https://github.com/rosmod/lib-dynamixel

## Dependencies

Install dependencies:

```bash
sudo apt-get install libboost-system-dev build-essential cmake
```

## Generate the build files from the [CMakeLists.txt](./CMakeLists.txt)

```bash
cmake .
```

## Build the test program

```bash
make
```

## Run the executable

Note: Please look at [main.cpp](./test/main.cpp) for all the options and to
see the example code for how to use the dynamixel library.

```bash
./bin/dynamixel_test --portName <serial port> --baudRate <baud rate> --motorId <motor id>
```

e.g.

```bash
./bin/dynamixel_test --command Set --address Goal --numBytes 2 --data 512 --portName /dev/ttyAMA0 --baudRate 9600 --motorId 1
```
