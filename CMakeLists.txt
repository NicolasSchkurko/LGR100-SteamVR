cmake_minimum_required(VERSION 3.15)

project(driver_openhmd)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_STANDARD 99)

# Build-time metadata the SteamVR plugin needs next to the DLL.
file(COPY driver.vrdrivermanifest DESTINATION ${CMAKE_BINARY_DIR})
file(COPY resources DESTINATION ${CMAKE_BINARY_DIR})

# hidapi: vendored under third_party/, so the driver builds with nothing
# else installed.
set(HIDAPI_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/hidapi")
if(NOT EXISTS "${HIDAPI_ROOT}/include/hidapi.h")
    set(HIDAPI_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/hidapi")
endif()
find_path(HIDAPI_INCLUDE_DIR hidapi.h PATHS "${HIDAPI_ROOT}/include" NO_DEFAULT_PATH)
find_library(HIDAPI_LIBRARY hidapi PATHS "${HIDAPI_ROOT}/x64" NO_DEFAULT_PATH)
if(NOT HIDAPI_INCLUDE_DIR OR NOT HIDAPI_LIBRARY)
    message(FATAL_ERROR "hidapi not found in ${HIDAPI_ROOT}")
endif()
set(HIDAPI_INCLUDE_DIRS "${HIDAPI_INCLUDE_DIR}")
set(HIDAPI_LIBRARIES "${HIDAPI_LIBRARY}")
include_directories(${HIDAPI_INCLUDE_DIRS})

# The hidapi DLL ships next to the driver DLL.
file(COPY "${HIDAPI_ROOT}/x64/hidapi.dll" DESTINATION "${CMAKE_BINARY_DIR}/bin/win64")

# The standalone device layer: HID + IMU decoding + sensor fusion. This is
# what replaces the vendored OpenHMD stack, so the whole driver is one block.
add_library(lgr100_standalone STATIC
  standalone/lgr100_device.cpp
  standalone/lgr100_omath.c
  standalone/lgr100_fusion.c
  standalone/lgr100_packet.c
)
target_include_directories(lgr100_standalone PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/standalone)
target_link_libraries(lgr100_standalone ${HIDAPI_LIBRARIES})

# openvr_driver.h (Valve's OpenVR SDK header, the server side). It lives in
# third_party/, next to the client half of the SDK (openvr.h and
# openvr_api.lib) that the tools use.
set(OPENVR_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/openvr")
if(NOT EXISTS "${OPENVR_ROOT}/openvr_driver.h")
    set(OPENVR_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/openvr")
endif()
include_directories("${OPENVR_ROOT}")

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
  lgr100_standalone
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



