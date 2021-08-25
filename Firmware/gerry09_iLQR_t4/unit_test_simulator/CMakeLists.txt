
find_package(gtwrap)

set(interface_files ${PROJECT_SOURCE_DIR}/cpp/${PROJECT_NAME}.h)

# pybind_wrap(${PROJECT_NAME}_py # target
#             "${interface_files}" # list of interface header files
#             "${PROJECT_NAME}.cpp" # the generated cpp
#             "${PROJECT_NAME}" # module_name
#             "${PROJECT_MODULE_NAME}" # top namespace in the cpp file e.g. gtsam
#             "${ignore}" # ignore classes
#             ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.tpl # the wrapping template file
#             ${PROJECT_NAME} # libs
#             "${PROJECT_NAME}" # dependencies
#             ON # use boost
#             )

add_library(Metro Metro/Metro.cpp)
target_link_libraries(Metro PUBLIC ArduinoTestUtils)
target_include_directories(Metro PUBLIC "${PROJECT_SOURCE_DIR}/unit_test_simulator/")

add_executable(test_runner_test test_runner_test.cpp)
target_link_libraries(test_runner_test PUBLIC Metro)
target_include_directories(test_runner_test PUBLIC
                          "${PROJECT_SOURCE_DIR}/"
                          "${PROJECT_SOURCE_DIR}/unit_test_simulator/Metro"
                          )
