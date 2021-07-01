# Build/install CppUnitLite

file(GLOB cppunitlite_headers "*.h")
file(GLOB cppunitlite_src "*.cpp")

add_library(CppUnitLite STATIC ${cppunitlite_src} ${cppunitlite_headers})

install(FILES ${cppunitlite_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/CppUnitLite)
