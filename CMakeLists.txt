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
project(WandererRotatorSDK VERSION 1.6.0 DESCRIPTION "Wanderer Rotator SDK for telescope control")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(WANDERER_ROTATOR_BUILD_TESTS "Build the interactive and automated test programs" ON)

# std::thread needs pthreads linked explicitly on glibc < 2.34
find_package(Threads REQUIRED)

# Find libudev package
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUDEV libudev)

if(NOT LIBUDEV_FOUND)
	message(FATAL_ERROR "libudev not found. Install it with: sudo apt-get install libudev-dev")
endif()

set(WANDERER_ROTATOR_SOURCES
	WandererRotatorSDK.cpp
	WandererRotatorLogging.cpp
	WandererRotatorSerialPort.cpp
	WandererRotatorDevice.cpp
	WandererRotatorProtocol.cpp)

# SDK Library - only the C API in WandererRotatorSDK.h is exported
add_library(WandererRotatorSDK SHARED ${WANDERER_ROTATOR_SOURCES})
target_include_directories(WandererRotatorSDK PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(WandererRotatorSDK PRIVATE ${LIBUDEV_INCLUDE_DIRS})
target_link_libraries(WandererRotatorSDK PRIVATE Threads::Threads ${LIBUDEV_LIBRARIES})
target_compile_definitions(WandererRotatorSDK PRIVATE WANDERER_ROTATOR_SDK_BUILD)
target_compile_options(WandererRotatorSDK PRIVATE -Wall -Wextra)
set_target_properties(WandererRotatorSDK PROPERTIES
	CXX_VISIBILITY_PRESET hidden
	VISIBILITY_INLINES_HIDDEN ON
	VERSION ${PROJECT_VERSION}
	SOVERSION ${PROJECT_VERSION_MAJOR})

if(WANDERER_ROTATOR_BUILD_TESTS)
	# Interactive test against real hardware
	add_executable(test_wanderer_rotator test_wanderer_rotator.cpp)
	target_link_libraries(test_wanderer_rotator WandererRotatorSDK)

	# Automated tests against a simulated rotator on a pseudo-terminal. Built
	# from the sources directly since it needs the (hidden) internal API.
	enable_testing()
	add_executable(test_fake_rotator tests/test_fake_rotator.cpp ${WANDERER_ROTATOR_SOURCES})
	target_include_directories(test_fake_rotator PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${LIBUDEV_INCLUDE_DIRS})
	target_link_libraries(test_fake_rotator PRIVATE Threads::Threads ${LIBUDEV_LIBRARIES})
	add_test(NAME fake_rotator COMMAND test_fake_rotator)
	set_tests_properties(fake_rotator PROPERTIES TIMEOUT 120)

	# Automated tests against a real rotator - moves it, so not part of ctest
	add_executable(test_hardware tests/test_hardware.c)
	target_link_libraries(test_hardware WandererRotatorSDK m)
endif()

# Installation
install(TARGETS WandererRotatorSDK
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
)
install(FILES WandererRotatorSDK.h DESTINATION include)
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
