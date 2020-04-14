cmake_minimum_required(VERSION 3.0)

project(cable_robot_simulation C CXX)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "~/gtsam/cmake")
set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH}" "~/gtsam/build_org" )
find_package(GTSAM REQUIRED)

#link_libraries(PUBLIC test_positioning )

## Build Executable

add_executable(test_positioning test_positioning.cpp)

add_executable(test_motor testMotorFactor.cpp)

add_executable(test_cable testCableFactor.cpp)

add_executable(test_wrench testWrenchFactor.cpp)

target_link_libraries(test_positioning PUBLIC gtsam)

target_include_directories(test_positioning PUBLIC
    ${GTSAM_INCLUDE_DIR}
    ${CMAKE_PREFIX_PATH}/include)

target_link_libraries(test_motor PUBLIC gtsam)
target_link_libraries(test_motor PUBLIC CppUnitLite)

target_include_directories(test_motor PUBLIC
    ${GTSAM_INCLUDE_DIR}
    ${CMAKE_PREFIX_PATH}/include
    )

target_link_libraries(test_cable PUBLIC gtsam)
target_link_libraries(test_cable PUBLIC CppUnitLite)

target_include_directories(test_cable PUBLIC
    ${GTSAM_INCLUDE_DIR}
    ${CMAKE_PREFIX_PATH}/include)

target_link_libraries(test_wrench PUBLIC gtsam)
target_link_libraries(test_wrench PUBLIC CppUnitLite)

target_include_directories(test_wrench PUBLIC
    ${GTSAM_INCLUDE_DIR}
    ${CMAKE_PREFIX_PATH}/include)


# gtsamAddExamplesGlob(“*.cpp” “${excluded_examples}” “gtsam;${Boost_PROGRAM_OPTIONS_LIBRARY}”)