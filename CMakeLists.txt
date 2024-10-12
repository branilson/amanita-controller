# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(amanita-controller)

# Include the USB common utilities (if needed)
include(${ZEPHYR_BASE}/samples/subsys/usb/common/common.cmake)

# Add the include path for Pid-C headers
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/Pid-C/include)

# Add Pid-C source files to the app build
file(GLOB pid_c_sources ${CMAKE_CURRENT_SOURCE_DIR}/Pid-C/src/*.c)

# Collect all your application sources
file(GLOB app_sources src/*.c)

# Add all sources (app + Pid-C) to the target
target_sources(app PRIVATE ${app_sources} ${pid_c_sources})