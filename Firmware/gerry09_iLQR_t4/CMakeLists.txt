project(cablerobot)
cmake_minimum_required(VERSION 3.19)
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -pedantic -Werror)

enable_testing()
add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND} -C $<CONFIGURATION> --output-on-failure)
add_custom_target(all.tests)

add_library(ArduinoTestUtils unit_test_simulator/arduino_test_utils.cpp)
add_subdirectory(CppUnitLite) 
add_subdirectory(src)
add_subdirectory(unit_test_simulator)
