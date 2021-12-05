cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 11)
project(phonebot-tests)

#find_package(PythonLibs REQUIRED)
#include_directories(/home/jamie/Repos/matplotlib-cpp/)
#include_directories(${PYTHON_INCLUDE_DIRS})

include_directories(..)
add_executable(test_pid test_pid.cpp)
target_link_libraries(test_pid gtest phonebot ${PYTHON_LIBRARY})
add_test(test-pid test_pid)

add_executable(test_kin test_kin.cpp)
target_link_libraries(test_kin gtest phonebot ${PYTHON_LIBRARY})
add_test(test-kid test_kin)
