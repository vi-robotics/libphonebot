cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS true)
project(phonebot)

include(cmake/FindEigen3.cmake)

option(PACKAGE_TESTS "Build the tests" OFF)
option(PACKAGE_SWIG "Build SWIG wrappers" ON)

set(Eigen3_DIR /usr/local/share/eigen3)

#find_package(Eigen3 3.3 REQUIRED)
include_directories(${eigen3_INCLUDE_DIRS})

add_library(phonebot
    SHARED

    JointTrajectoryController.cpp
    JointTrajectoryController.hpp
    PhonebotKinematics.cpp
    PhonebotKinematics.hpp
    PIDController.cpp
    PIDController.hpp

    )
#target_link_libraries(phonebot
#    Eigen3::Eigen
#    )

if(PACKAGE_TESTS)
    configure_file(CMakeLists.txt.in googletest-download/CMakeLists.txt)
    execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
    RESULT_VARIABLE result
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
    if(result)
    message(FATAL_ERROR "CMake step for googletest failed: ${result}")
    endif()
    execute_process(COMMAND ${CMAKE_COMMAND} --build .
    RESULT_VARIABLE result
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
    if(result)
    message(FATAL_ERROR "Build step for googletest failed: ${result}")
    endif()

    # Prevent overriding the parent project's compiler/linker
    # settings on Windows
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

    # Add googletest directly to our build. This defines
    # the gtest and gtest_main targets.
    add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/googletest-src
                    ${CMAKE_CURRENT_BINARY_DIR}/googletest-build
                    EXCLUDE_FROM_ALL)

    # The gtest/gtest_main targets carry header search path
    # dependencies automatically when using CMake 2.8.11 or
    # later. Otherwise we have to add them here ourselves.
    if (CMAKE_VERSION VERSION_LESS 2.8.11)
    include_directories("${gtest_SOURCE_DIR}/include")
    endif()

    enable_testing()
    add_subdirectory(tests)
endif()

if(PACKAGE_SWIG)
    add_subdirectory(swig)
endif()
