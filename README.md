Dynamixel AX-12A C++ Library
============================

Build Library for ROS:
----------------------

```bash
git clone http://github.com/rosmod/lib-dynamixel
cd lib-dynamixel
catkin_make
```

Use the Library without ROS:
----------------------------

You will find an example [CMake File](./test/CMakeLists.txt) and
[source file](./test/main.cpp).  These are all found within the
[test](./test) directory, which contains a README as well. This serves
as an example of how to integrate this code into a project that does
not use ROS.