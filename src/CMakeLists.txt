add_library(libdynamixel
    Dynamixel.cpp
    SerialPort.cpp
)

target_include_directories(libdynamixel
    PUBLIC
        ${CMAKE_SOURCE_DIR}/include
)

add_definitions(-DBOOST_LOG_DYN_LINK)