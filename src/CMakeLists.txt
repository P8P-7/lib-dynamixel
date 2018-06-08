project(libdynamixel LANGUAGES CXX)

add_library(${PROJECT_NAME}
        Dynamixel.cpp
        SerialPort.cpp
        )

add_definitions(-DBOOST_LOG_DYN_LINK)

target_include_directories(${PROJECT_NAME}
        PUBLIC
            ${CMAKE_SOURCE_DIR}/include
            ${Boost_INCLUDE_DIR}
        )

target_link_libraries(${PROJECT_NAME}
        PRIVATE
            ${CMAKE_THREAD_LIBS_INIT}
            ${Boost_LIBRARIES}
        )