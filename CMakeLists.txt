#########################################################################
# CMakeLists.txt file for building  ec4cpp with CMake                      #
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

project(EC4CPP)

# BMM version. Defined here but not exported to header files, because it's already defined there
set( EC4CPP_VERSION 0.0.1 )
string( REGEX MATCHALL "[0-9]+" EC4CPP_VERSIONS ${EC4CPP_VERSION} )
LIST( GET EC4CPP_VERSIONS 0 EC4CPP_VERSION_MAJOR)
LIST( GET EC4CPP_VERSIONS 1 EC4CPP_VERSION_MINOR)
LIST( GET EC4CPP_VERSIONS 2 EC4CPP_VERSION_PATCH)

# Some example programs use "libpthread"
# http://stackoverflow.com/questions/1620918/cmake-and-libpthread
find_package(Threads REQUIRED)
include(FindPkgConfig)

#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules)

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

   pkg_check_modules(SOEMRT REQUIRED soemrt)
   message("Found soemrt (soem with realtime patch) and its includes : ${SOEMRT_INCLUDE_DIRS}" )
   message("--> with CFLAGS : ${SOEMRT_CFLAGS}" )
   message("Found soemrt (soem with realtime patch) and its libs : ${SOEMRT_LIBRARIES}" )
   message("--> with LDFLAGS : ${SOEMRT_LDFLAGS}" )

   list(APPEND EC4CPP_CFLAGS ${SOEMRT_CFLAGS})
   list(APPEND EC4CPP_CFLAGS ${XENO_NATIVE_CFLAGS})
   message("The complete ec4cpp CFLAGS : ${EC4CPP_CFLAGS}" )

   list(APPEND EC4CPP_LDFLAGS ${SOEMRT_LDFLAGS})
   list(APPEND EC4CPP_LDFLAGS ${XENO_NATIVE_LDFLAGS})
   message("The complete ec4cpp LFLAGS : ${EC4CPP_LDFLAGS}" )

else(ENABLE_RT)
   message (FATAL_ERROR, "This package by now needs Xenomai RTOS")
endif(ENABLE_RT)




add_subdirectory(src)
add_subdirectory(tests)