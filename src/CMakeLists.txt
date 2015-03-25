
# CPP4EC header files
set(CPP4EC_HDRS
      EcError.h
      EcErrorSGDV.h
      EcMaster.h
      EcSlaveFactory.h
      EcSlave.h
      EcSlaveSGDV.h
      EcSlaveTrack.h
      EcUtil.h
)

# CPP4EC source files
set(CPP4EC_SRC
      EcError.cpp
      EcErrorSGDV.cpp
      EcSlave.cpp
      EcSlaveFactory.cpp
      EcSlaveSGDV.cpp
      EcSlaveTrack.cpp   
)

# CPP4EC RT versions source files
set(CPP4ECRT_SRC ${CPP4EC_SRC} EcRTMaster.cpp)

# CPP4EC NRT versions source files
set(CPP4ECRT_SRC ${CPP4EC_SRC} EcNRTMaster.cpp)

if(ENABLE_RT)
    include_directories(${SOEMRT_INCLUDE_DIRS})
    include_directories(${XENO_NATIVE_INCLUDE_DIRS})
    add_subdirectory(EcRTThread)
endif(ENABLE_RT)

include_directories(${SOEM_INCLUDE_DIRS})

ADD_DEFINITIONS(-std=c++11 )

set(BOOST_LIBS thread date_time system)
find_package(Boost COMPONENTS ${BOOST_LIBS} REQUIRED)
message(STATUS "Find Boost and using ${Boost_LIBRARIES}")
include_directories(${Boost_INCLUDE_DIRS})

if(ENABLE_RT)
  ##### Building cpp4ec_hrt #######
  # cpp4ec half realtime time. Internal loop RT and external NRT

  add_library(cpp4ec_hrt_static STATIC ${CPP4ECRT_SRC})
  target_link_libraries(cpp4ec_hrt_static rtectask_hrt_static ${SOEMRT_LIBRARIES} 
 		${XENO_NATIVE_LIBRARIES} ${PUGI_XML_LIBRARIES} 
		${XENO_RTDM_LIBRARIES} ${Boost_LIBRARIES})
  set_target_properties(cpp4ec_hrt_static PROPERTIES
    OUTPUT_NAME cpp4ec_hrt
    COMPILE_FLAGS ${CPP4EC_CFLAGS}
    LINK_FLAGS ${CPP4EC_LDFLAGS}
    INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
    COMPILE_DEFINITIONS "HRT"
    CLEAN_DIRECT_OUTPUT 1)

  install(TARGETS cpp4ec_hrt_static DESTINATION lib)

  # Create dynamic library (.so) with all basic BMM functions
  add_library(cpp4ec_hrt_dynamic SHARED ${CPP4ECRT_SRC})
  target_link_libraries(cpp4ec_hrt_dynamic rtectask_hrt_static ${SOEMRT_LIBRARIES} 
		${XENO_NATIVE_LIBRARIES} ${PUGI_XML_LIBRARIES} 
		${XENO_RTDM_LIBRARIES} ${Boost_LIBRARIES})
  set_target_properties( cpp4ec_hrt_dynamic PROPERTIES
    DEFINE_SYMBOL "CPP4EC_DLL_EXPORT"
    SOVERSION "${CPP4EC_VERSION_MAJOR}"
    VERSION "${CPP4EC_VERSION}"
    OUTPUT_NAME cpp4ec_hrt
    COMPILE_FLAGS ${CPP4EC_CFLAGS}
    LINK_FLAGS ${CPP4EC_LDFLAGS}
    INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
    COMPILE_DEFINITIONS "HRT"
    CLEAN_DIRECT_OUTPUT 1)

  # Defaul install
  install(TARGETS cpp4ec_hrt_dynamic DESTINATION lib)

  ##### Building cpp4ec_rt #######
  # cpp4ec full realtime time. Internal loop RT and user loop RT

  add_library(cpp4ec_rt_static STATIC ${CPP4ECRT_SRC})
  target_link_libraries(cpp4ec_rt_static rtectask_rt_static ${SOEMRT_LIBRARIES} 
 		${XENO_NATIVE_LIBRARIES} ${PUGI_XML_LIBRARIES} 
		${XENO_RTDM_LIBRARIES} ${Boost_LIBRARIES})
  set_target_properties(cpp4ec_rt_static PROPERTIES
    OUTPUT_NAME cpp4ec_rt
    COMPILE_FLAGS ${CPP4EC_CFLAGS}
    LINK_FLAGS ${CPP4EC_LDFLAGS}
    INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
    COMPILE_DEFINITIONS "RT"
    CLEAN_DIRECT_OUTPUT 1)

  install(TARGETS cpp4ec_rt_static DESTINATION lib)

  # Create dynamic library (.so) with all basic BMM functions
  add_library(cpp4ec_rt_dynamic SHARED ${CPP4ECRT_SRC})
  target_link_libraries(cpp4ec_rt_dynamic rtectask_rt_static ${SOEMRT_LIBRARIES} 
		${XENO_NATIVE_LIBRARIES} ${PUGI_XML_LIBRARIES} 
		${XENO_RTDM_LIBRARIES} ${Boost_LIBRARIES})
  set_target_properties( cpp4ec_rt_dynamic PROPERTIES
    DEFINE_SYMBOL "CPP4EC_DLL_EXPORT"
    SOVERSION "${CPP4EC_VERSION_MAJOR}"
    VERSION "${CPP4EC_VERSION}"
    OUTPUT_NAME cpp4ec_rt
    COMPILE_FLAGS ${CPP4EC_CFLAGS}
    LINK_FLAGS ${CPP4EC_LDFLAGS}
    INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
    COMPILE_DEFINITIONS "RT"
    CLEAN_DIRECT_OUTPUT 1)

  # Headers install
  install(FILES ${CPP4EC_HDRS} DESTINATION include/cpp4ec)

endif(ENABLE_RT)

##### Building cpp4ec #######
# cpp4ec non realtime time. 

add_library(cpp4ec_static STATIC ${CPP4EC_SRC})
target_link_libraries(cpp4ec_static ${SOEM_LIBRARIES}  ${PUGI_XML_LIBRARIES} ${Boost_LIBRARIES})

set_target_properties(cpp4ec_static PROPERTIES
      OUTPUT_NAME cpp4ec
#      COMPILE_FLAGS ${CPP4EC_CFLAGS}
#      LINK_FLAGS ${CPP4EC_LDFLAGS}
     INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
      CLEAN_DIRECT_OUTPUT 1)

install(TARGETS cpp4ec_static DESTINATION lib)

# Create dynamic library (.so) with all basic BMM functions
add_library(cpp4ec_dynamic SHARED ${CPP4EC_SRC})
target_link_libraries(cpp4ec_dynamic ${SOEM_LIBRARIES}  ${PUGI_XML_LIBRARIES} ${Boost_LIBRARIES} )

set_target_properties( cpp4ec_dynamic PROPERTIES
    DEFINE_SYMBOL "CPP4EC_DLL_EXPORT"
    SOVERSION "${CPP4EC_VERSION_MAJOR}"
    VERSION "${CPP4EC_VERSION}"
    OUTPUT_NAME cpp4ec
#    COMPILE_FLAGS ${CPP4EC_CFLAGS}
#    LINK_FLAGS ${CPP4EC_LDFLAGS}
    INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
    CLEAN_DIRECT_OUTPUT 1)

# Defaul install
install(TARGETS cpp4ec_dynamic DESTINATION lib)

# Headers install
install(FILES ${CPP4EC_HDRS}  DESTINATION include/cpp4ec)


