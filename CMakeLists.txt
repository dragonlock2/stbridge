cmake_minimum_required(VERSION 3.20.0)
project(stbridge)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_subdirectory(pybind11)

pybind11_add_module(stbridge
    stbridge.cpp
    libs/src/bridge/bridge.cpp
    libs/src/common/criticalsectionlock.cpp
    libs/src/common/stlink_device.cpp
    libs/src/common/stlink_interface.cpp
    libs/src/error/ErrLog.cpp
)

find_package(libusb CONFIG QUIET)
if(NOT ${libusb_FOUND})
    find_library(LIBUSB_LIBRARIES NAMES usb-1.0)
    find_path(LIBUSB_INCLUDE_DIRS NAMES libusb.h PATH_SUFFIXES libusb-1.0)
endif()
target_link_libraries(stbridge PRIVATE ${LIBUSB_LIBRARIES})

target_include_directories(stbridge PRIVATE
    ${LIBUSB_INCLUDE_DIRS}
    libs/src/bridge
    libs/src/common
    libs/src/error
)
