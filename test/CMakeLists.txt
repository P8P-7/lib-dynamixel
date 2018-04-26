cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_test)

include_directories(${CMAKE_SOURCE_DIR}/include)

find_package(Boost COMPONENTS system REQUIRED)

add_executable(dynamixel_test main.cpp)

target_link_libraries(dynamixel_test
    pthread
    libdynamixel
    ${Boost_SYSTEM_LIBRARY}
)
