# ###############################################################################
# MIT License
#
# Copyright (c) 2025 Nico Trost
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# ###############################################################################

cmake_minimum_required(VERSION 3.22)
project(WandererRotatorSDK VERSION 1.1.0 DESCRIPTION "Wanderer Rotator SDK for telescope control")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find libudev package
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUDEV libudev)

# SDK Library
add_library(WandererRotatorSDK SHARED 
	WandererRotatorSDK.cpp 
	WandererRotatorLogging.cpp 
	WandererRotatorSerialPort.cpp
	WandererRotatorDevice.cpp
	WandererRotatorProtocol.cpp)

# Public headers
target_include_directories(WandererRotatorSDK PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Link libudev
if(LIBUDEV_FOUND)
	target_include_directories(WandererRotatorSDK PRIVATE ${LIBUDEV_INCLUDE_DIRS})
	target_link_libraries(WandererRotatorSDK PRIVATE ${LIBUDEV_LIBRARIES})
else()
	message(FATAL_ERROR "libudev not found. Install it with: sudo apt-get install libudev-dev")
endif()

# Test executable
add_executable(test_wanderer_rotator test_wanderer_rotator.cpp)
target_link_libraries(test_wanderer_rotator WandererRotatorSDK)

# Installation
install(TARGETS WandererRotatorSDK
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
)
install(FILES WandererRotatorSDK.h WandererRotatorLogging.h WandererRotatorSerialPort.h WandererRotatorDevice.h WandererRotatorProtocol.h DESTINATION include)
# CPack configuration for Debian package generation
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_CONTACT "Nico")
set(CPACK_DEBIAN_PACKAGE_HOMEPAGE "https://github.com/nitr57/WandererRotatorSDK")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Nico")
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION_SUMMARY "SDK for controlling Wanderer Rotator devices via serial communication")

# Runtime dependencies
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libudev1")

# License
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")

include(CPack)