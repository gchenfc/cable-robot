project(cablerobot)
cmake_minimum_required(VERSION 3.19)
set(CMAKE_CXX_STANDARD 11)

enable_testing()
add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND} -C $<CONFIGURATION> --output-on-failure)
add_custom_target(all.tests)

add_subdirectory(CppUnitLite) 

add_compile_options(-Wall -Wextra -pedantic -Werror)
FILE(GLOB script_srcs RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*_test.cpp")
FOREACH(script_src IN ITEMS ${script_srcs})
	# Get test base name (strip .cpp from end)
	GET_FILENAME_COMPONENT(script_name ${script_src} NAME_WE)

	# Add executable
	ADD_EXECUTABLE(${script_name} ${script_src} ${script_headers})
	TARGET_LINK_LIBRARIES(${script_name} CppUnitLite ${linkLibraries})
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
