#########################################################################
# CMakeLists.txt file for building  cpp4ec with CMake                      #
#                                                                       #
# Copyright (c) 2013 Leopold Palomo-Avellaneda <leo@alaxarxa.net>       #
#                                                                       #
# This file is free software: you can redistribute it and/or modify     #
# it under the terms of the GNU General Public License as published by  #
# the Free Software Foundation, either version 3 of the License, or     #
# (at your option) any later version.                                   #
#                                                                       #
# This program is distributed in the hope that it will be useful,       #
# but WITHOUT ANY WARRANTY; without even the implied warranty of        #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
# GNU General Public License for more details.                          #
#                                                                       #
# You should have received a copy of the GNU General Public License     #
# along with this program.  If not, see <http://www.gnu.org/licenses/>. #
#########################################################################

cmake_minimum_required (VERSION 2.6)

project(CPP4EC)


# BMM version. Defined here but not exported to header files, because it's already defined there
set( CPP4EC_VERSION 0.0.1 )
string( REGEX MATCHALL "[0-9]+" CPP4EC_VERSIONS ${CPP4EC_VERSION} )
LIST( GET CPP4EC_VERSIONS 0 CPP4EC_VERSION_MAJOR)
LIST( GET CPP4EC_VERSIONS 1 CPP4EC_VERSION_MINOR)
LIST( GET CPP4EC_VERSIONS 2 CPP4EC_VERSION_PATCH)

# Some example programs use "libpthread"
# http://stackoverflow.com/questions/1620918/cmake-and-libpthread
find_package(Threads REQUIRED)
include(FindPkgConfig)

#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake-modules)

include_directories(${PROJECT_SOURCE_DIR})

# Setting some flags
set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-undefined")
# Optional
#set (CMAKE_C_FLAGS "-fdump-rtl-expand")

#Check if we need to link against librt
include(CheckLibraryExists)
CHECK_LIBRARY_EXISTS(rt clock_gettime "time.h" HAVE_CLOCK_GETTIME)
if(HAVE_CLOCK_GETTIME)
  set(CMAKE_SHARED_EXTRA_LIBS "-lrt")
endif()

option(ENABLE_STATIC_LIB "Enable libbmm static." ON)
option(ENABLE_TESTS "Enable test programs and utilities linked dynamically." ON)
option(ENABLE_TESTS_STATIC "Enable test programs and utilities linked statically." OFF)

option(ENABLE_RT "Enable Realtime with Xenomai." ON)

if(ENABLE_RT)
  #common commands for building c++ executables and libraries
  # try Xenomai
  # find Xenomai
   pkg_check_modules(XENO_NATIVE REQUIRED libxenomai_native)
   message("Found xenomai native includes : ${XENO_NATIVE_INCLUDE_DIRS}" )
   message("--> with CFLAGS : ${XENO_NATIVE_CFLAGS}" )
   message("Found xenomai native libs : ${XENO_NATIVE_LIBRARIES}" )
   message("--> with LDFLAGS : ${XENO_NATIVE_LDFLAGS}" )

   pkg_check_modules(XENO_RTDM REQUIRED libxenomai_rtdm)
   message("Found xenomai native includes : ${XENO_RTDM_INCLUDE_DIRS}" )
   message("--> with CFLAGS : ${XENO_RTDM_CFLAGS}" )
   message("Found xenomai native libs : ${XENO_RTDM_LIBRARIES}" )
   message("--> with LDFLAGS : ${XENO_RTDM_LDFLAGS}" )

   pkg_check_modules(SOEMRT REQUIRED soemrt)
   message("Found soemrt (soem with realtime patch) and its includes : ${SOEMRT_INCLUDE_DIRS}" )
   message("--> with CFLAGS : ${SOEMRT_CFLAGS}" )
   message("Found soemrt (soem with realtime patch) and its libs : ${SOEMRT_LIBRARIES}" )
   message("--> with LDFLAGS : ${SOEMRT_LDFLAGS}" )

   list(APPEND CPP4EC_CFLAGS ${SOEMRT_CFLAGS})
   list(APPEND CPP4EC_CFLAGS ${XENO_NATIVE_CFLAGS})
   list(APPEND CPP4EC_CFLAGS ${XENO_RTDM_CFLAGS})
   message("The complete cpp4ec CFLAGS : ${CPP4EC_CFLAGS}" )

   list(APPEND CPP4EC_LDFLAGS ${SOEMRT_LDFLAGS})
   list(APPEND CPP4EC_LDFLAGS ${XENO_NATIVE_LDFLAGS})
   list(APPEND CPP4EC_CFLAGS ${XENO_RTDM_CFLAGS})
   message("The complete cpp4ec LFLAGS : ${CPP4EC_LDFLAGS}" )

else(ENABLE_RT)
   message (SEND_ERROR "This package by now needs Xenomai RTOS")
endif(ENABLE_RT)

#PUGI_XML
find_path(PUGI_XML_INCLUDE_DIRS pugixml.hpp
    PATHS /usr/include
	  /user/local/include)

if(PUGI_XML_INCLUDE_DIRS)
  message("Found PUGI_XML_INCLUDE_DIRS  on: ${PUGI_XML_INCLUDE_DIRS}")
  include_directories(${PUGI_XML_INCLUDE_DIRS})
else(PUGI_XML_INCLUDE_DIRS)
  message(SEND_ERROR "Not found PUGI_XML_INCLUDE_DIRS")
endif(PUGI_XML_INCLUDE_DIRS)

find_library (PUGI_XML_LIBRARIES
                NAMES pugixml libpugixml
                PATHS /usr/lib
		      /usr/local/lib
                      /user/local/lib)
if(PUGI_XML_LIBRARIES)
  message("Found PUGI_XML_LIBRARIES on:" ${PUGI_XML_LIBRARIES})
else(PUGI_XML_LIBRARIES)
  message(SEND_ERROR "Not found PUGI_XML_LIBRARIES")
endif(PUGI_XML_LIBRARIES)


add_subdirectory(src)
add_subdirectory(tests)