cmake_minimum_required(VERSION 3.1)

project (driver_openhmd)

set (CMAKE_CXX_STANDARD 11)

# meta data that the steamvr plugin needs in the base directory of the steamvr plugin
file(COPY driver.vrdrivermanifest DESTINATION ${CMAKE_BINARY_DIR})
file(COPY resources DESTINATION ${CMAKE_BINARY_DIR})

# Optionally link a prebuilt static OpenHMD (and its hidapi import lib) instead of building the submodule
set(OPENHMD_PREBUILT_LIB "" CACHE FILEPATH "Prebuilt static openhmd library")
set(OPENHMD_PREBUILT_INCLUDE "" CACHE PATH "Include dir for the prebuilt openhmd")
set(OPENHMD_PREBUILT_DEPS "" CACHE STRING "Extra libraries the prebuilt openhmd needs")
if(OPENHMD_PREBUILT_LIB)
  add_library(openhmd STATIC IMPORTED)
  set_target_properties(openhmd PROPERTIES
    IMPORTED_LOCATION "${OPENHMD_PREBUILT_LIB}"
    INTERFACE_INCLUDE_DIRECTORIES "${OPENHMD_PREBUILT_INCLUDE}"
    INTERFACE_COMPILE_DEFINITIONS OHMD_STATIC
    INTERFACE_LINK_LIBRARIES "${OPENHMD_PREBUILT_DEPS}")
else()
  include_directories("${CMAKE_SOURCE_DIR}/subprojects/openhmd/include")
  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/subprojects/openhmd/cmake/")
  add_subdirectory(./subprojects/openhmd)
endif()

include_directories("${CMAKE_SOURCE_DIR}/subprojects/openvr/")

add_library(driver_openhmd SHARED
  driverlog.cpp
  driverlog.h
  driver_openhmd.cpp
)

if(MSVC)
  string(REGEX REPLACE "/W[0-4]" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  target_compile_options(driver_openhmd PRIVATE /W4 /WX /external:anglebrackets /external:W0)
  target_compile_definitions(driver_openhmd PRIVATE _USE_MATH_DEFINES _CRT_SECURE_NO_WARNINGS)
else(MSVC)
  target_compile_options(driver_openhmd PRIVATE -Wall) #-Wextra -Wno-unused-parameter) #-pedantic)
endif(MSVC)

target_link_libraries(driver_openhmd
  ${CMAKE_DL_LIBS}
  openhmd
)

#determine the output directory for the steamvr plugin
if (WIN32)
  # FIXME need to account for different architectures
  if ("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
    set(PLUGINDIR ${CMAKE_BINARY_DIR}/bin/win64 CACHE TYPE INTERNAL)
  else()
    set(PLUGINDIR ${CMAKE_BINARY_DIR}/bin/win32 CACHE TYPE INTERNAL)
  endif()
elseif(APPLE)
  if ("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
    set(PLUGINDIR ${CMAKE_BINARY_DIR}/bin/osx64 CACHE STRING INTERNAL)
  else()
    set(PLUGINDIR ${CMAKE_BINARY_DIR}/bin/osx32 CACHE STRING INTERNAL)
  endif()
elseif(NOT ANDROID)
  if ("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
    set(PLUGINDIR ${CMAKE_BINARY_DIR}/bin/linux64 CACHE STRING INTERNAL)
  else()
    set(PLUGINDIR ${CMAKE_BINARY_DIR}/bin/linux32 CACHE STRING INTERNAL)
  endif()
endif()
MESSAGE("SteamVR plugin path: ${PLUGINDIR}")
set_target_properties(driver_openhmd PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${PLUGINDIR}" RUNTIME_OUTPUT_DIRECTORY "${PLUGINDIR}")

SET_TARGET_PROPERTIES(driver_openhmd PROPERTIES PREFIX "") # don't add lib prefix to driver_openhmd.so

if(NOT OPENHMD_PREBUILT_LIB)
  add_dependencies(driver_openhmd openhmd)
endif()
