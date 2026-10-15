cmake_minimum_required(VERSION 3.15)

project(driver_openhmd)

set(CMAKE_CXX_STANDARD 11)

# Build-time metadata the SteamVR plugin needs next to the DLL.
file(COPY driver.vrdrivermanifest DESTINATION ${CMAKE_BINARY_DIR})
file(COPY resources DESTINATION ${CMAKE_BINARY_DIR})

# Vendored, frozen OpenHMD subset (see openhmd/CMakeLists.txt).
include_directories("${CMAKE_SOURCE_DIR}/openhmd/include")
add_subdirectory(./openhmd)

# openvr_driver.h (Valve's OpenVR SDK header).
include_directories("${CMAKE_SOURCE_DIR}/openvr/")

add_library(driver_openhmd SHARED
  driverlog.cpp
  driverlog.h
  driver_openhmd.cpp
)

if(MSVC)
  target_compile_options(driver_openhmd PRIVATE /W4 /D_CRT_SECURE_NO_WARNINGS)
else(MSVC)
  target_compile_options(driver_openhmd PRIVATE -Wall)
endif(MSVC)

target_link_libraries(driver_openhmd
  ${CMAKE_DL_LIBS}
  openhmd
)

# Output goes to bin/win64 like the original plugin.
if (WIN32)
  if ("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
    set(PLUGINDIR ${CMAKE_BINARY_DIR}/bin/win64 CACHE TYPE INTERNAL)
  else()
    set(PLUGINDIR ${CMAKE_BINARY_DIR}/bin/win32 CACHE TYPE INTERNAL)
  endif()
elseif(APPLE)
  set(PLUGINDIR ${CMAKE_BINARY_DIR}/bin/osx64 CACHE STRING INTERNAL)
elseif(NOT ANDROID)
  set(PLUGINDIR ${CMAKE_BINARY_DIR}/bin/linux64 CACHE STRING INTERNAL)
endif()
set_target_properties(driver_openhmd PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${PLUGINDIR}")

SET_TARGET_PROPERTIES(driver_openhmd PROPERTIES PREFIX "")

add_dependencies(driver_openhmd openhmd)
