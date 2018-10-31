# Copyright 2011,2012,2016 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.

########################################################################
# Setup library
########################################################################
include(GrPlatform) #define LIB_SUFFIX

include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

list(APPEND sm200a_sources
    iq_impl.cc
)

set(sm200a_sources "${sm200a_sources}" PARENT_SCOPE)
if(NOT sm200a_sources)
	MESSAGE(STATUS "No C++ sources... skipping lib/")
	return()
endif(NOT sm200a_sources)

add_library(gnuradio-sm200a SHARED ${sm200a_sources})
target_link_libraries(gnuradio-sm200a ${Boost_LIBRARIES} ${GNURADIO_ALL_LIBRARIES} "/usr/local/lib/libsm_api.so")
set_target_properties(gnuradio-sm200a PROPERTIES DEFINE_SYMBOL "gnuradio_sm200a_EXPORTS")

if(APPLE)
    set_target_properties(gnuradio-sm200a PROPERTIES
        INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
    )
endif(APPLE)

########################################################################
# Install built library files
########################################################################
include(GrMiscUtils)
GR_LIBRARY_FOO(gnuradio-sm200a RUNTIME_COMPONENT "sm200a_runtime" DEVEL_COMPONENT "sm200a_devel")

########################################################################
# Build and register unit test
########################################################################
include(GrTest)

include_directories(${CPPUNIT_INCLUDE_DIRS})

list(APPEND test_sm200a_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/test_sm200a.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/qa_sm200a.cc
)

add_executable(test-sm200a ${test_sm200a_sources})

target_link_libraries(
  test-sm200a
  ${GNURADIO_RUNTIME_LIBRARIES}
  ${Boost_LIBRARIES}
  ${CPPUNIT_LIBRARIES}
  gnuradio-sm200a
)

GR_ADD_TEST(test_sm200a test-sm200a)

########################################################################
# Print summary
########################################################################
message(STATUS "Using install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Building for version: ${VERSION} / ${LIBVER}")

