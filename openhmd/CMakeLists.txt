cmake_minimum_required(VERSION 3.10)

# Vendored subset of OpenHMD (https://github.com/OpenHMD/OpenHMD, Boost Software
# License 1.0) covering only what this driver uses: the core context/device API,
# math and sensor fusion, the Win32 platform layer, the LG R100 driver and the
# dummy driver (the plugin's null controllers). Upstream declares itself
# UNMAINTAINED; this subset is frozen here so the SteamVR plugin builds
# self-contained. Phase 1 of the roadmap replaces it with direct hidapi code.

project(openhmd C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")

set(openhmd_source_files
	${CMAKE_CURRENT_LIST_DIR}/src/openhmd.c
	${CMAKE_CURRENT_LIST_DIR}/src/platform-win32.c
	${CMAKE_CURRENT_LIST_DIR}/src/platform-posix.c
	${CMAKE_CURRENT_LIST_DIR}/src/drv_dummy/dummy.c
	${CMAKE_CURRENT_LIST_DIR}/src/omath.c
	${CMAKE_CURRENT_LIST_DIR}/src/fusion.c
	${CMAKE_CURRENT_LIST_DIR}/src/shaders.c
	${CMAKE_CURRENT_LIST_DIR}/src/drv_lgr100/lgr100.c
	${CMAKE_CURRENT_LIST_DIR}/src/drv_lgr100/packet.c
)

add_definitions(-DDRIVER_LGR)

find_package(HIDAPI REQUIRED)
include_directories(${HIDAPI_INCLUDE_DIRS})
set(LIBS ${LIBS} ${HIDAPI_LIBRARIES})

add_library(openhmd STATIC ${openhmd_source_files})

target_include_directories(openhmd PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(openhmd ${LIBS})
target_compile_definitions(openhmd PUBLIC -DOHMD_STATIC)
