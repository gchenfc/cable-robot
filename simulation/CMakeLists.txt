cmake_minimum_required(VERSION 3.0)

project(cable_robot_simulation C CXX)

find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBS gtsam)

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)

# For unit tests and scripts.
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")

## Cable Robot Library
file(GLOB sources "*.cpp")
file(GLOB headers "*.h")

foreach(SOURCE_DIR "factors" "robot" "util")
    file(GLOB sources_subdir ${SOURCE_DIR}/*.cpp)
    list(APPEND sources ${sources_subdir})
    file(GLOB headers_subdir ${SOURCE_DIR}/*.h)
    list(APPEND headers ${headers_subdir})
    add_subdirectory(${SOURCE_DIR})
endforeach()

add_library(cable_robot SHARED ${sources} ${headers})

target_link_libraries(cable_robot ${GTSAM_LIBS})
target_include_directories(cable_robot PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Install library and headers, if desired.
install(TARGETS cable_robot LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
set_target_properties(cable_robot PROPERTIES LINKER_LANGUAGE CXX)

# Process subdirectories.
add_subdirectory(tests)
add_subdirectory(scripts)
