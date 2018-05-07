project(dynamixel_test)

add_executable(dynamixel_test
    main.cpp
)

target_include_directories(dynamixel_test
    PUBLIC
        ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(dynamixel_test
    PUBLIC
        pthread
        libdynamixel
        ${Boost_SYSTEM_LIBRARY}
)
