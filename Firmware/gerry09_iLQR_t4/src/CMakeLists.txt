set(linkLibraries Metro)
set(includeDirectories ${CMAKE_SOURCE_DIR}/unit_test_simulator ${CMAKE_SOURCE_DIR}/unit_test_simulator/Metro)
add_subdirectory(communication)

FILE(GLOB script_srcs RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*_test.cpp")
FOREACH(script_src IN ITEMS ${script_srcs})
	# Get test base name (strip .cpp from end)
	GET_FILENAME_COMPONENT(script_name ${script_src} NAME_WE)

	# Add executable
	ADD_EXECUTABLE(${script_name} ${script_src} ${script_headers})
	TARGET_LINK_LIBRARIES(${script_name} CppUnitLite ArduinoTestUtils)
	TARGET_INCLUDE_DIRECTORIES(${script_name} PRIVATE ${CMAKE_SOURCE_DIR})

	# Add target dependencies
	ADD_TEST(NAME ${script_name} COMMAND ${script_name})
	ADD_DEPENDENCIES(check ${script_name})
	ADD_DEPENDENCIES(all.tests ${script_name})

	# Regular test run:
	ADD_CUSTOM_TARGET(${script_name}.run
		COMMAND ${EXECUTABLE_OUTPUT_PATH}${script_name}
		DEPENDS ${script_name}
	)

	# Exclude from 'make all' and 'make install'
	SET_TARGET_PROPERTIES(${script_name} PROPERTIES EXCLUDE_FROM_ALL ON)
ENDFOREACH()
