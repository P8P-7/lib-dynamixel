cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

project(goliath-dynamixel LANGUAGES CXX)

find_package(Boost
        COMPONENTS
            log
            system
        REQUIRED
        )

add_definitions(-DBOOST_LOG_DYN_LINK)

add_library(${PROJECT_NAME}
        src/Dynamixel.cpp
        src/SerialPort.cpp
        src/Utils.cpp
        )

target_include_directories(${PROJECT_NAME}
        PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${Boost_INCLUDE_DIR}
        )

target_link_libraries(${PROJECT_NAME}
        PRIVATE
            ${CMAKE_THREAD_LIBS_INIT}
            ${Boost_LIBRARIES}
        )
