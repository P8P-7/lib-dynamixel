add_library(libdynamixel
    Dynamixel.cpp
    SerialPort.cpp
)

target_include_directories(libdynamixel
    PUBLIC
        ${CMAKE_SOURCE_DIR}/include
)
