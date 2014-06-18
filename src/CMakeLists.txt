
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

if(ENABLE_RT)
    include_directories(${SOEMRT_INCLUDE_DIRS})
    include_directories(${XENO_NATIVE_INCLUDE_DIRS})
    set(CPP4EC_SRC ${CPP4EC_SRC} EcRTMaster.cpp)
    add_subdirectory(EcRTThread)
else(ENABLE_RT)
    include_directories(${SOEM_INCLUDE_DIRS})
    set(CPP4EC_SRC ${CPP4EC_SRC} EcNRTMaster.cpp)
#    add_subdirectory(EcNRTThread)
endif(ENABLE_RT)

ADD_DEFINITIONS(-std=c++11 )

if(ENABLE_RT)

  add_library(cpp4ec_static STATIC ${CPP4EC_SRC})
  target_link_libraries(cpp4ec_static cpp4ec_rt_static ${SOEMRT_LIBRARIES} ${XENO_NATIVE_LIBRARIES} ${PUGI_XML_LIBRARIES} ${XENO_RTDM_LIBRARIES})
  set_target_properties(cpp4ec_static PROPERTIES
    OUTPUT_NAME cpp4ec
    COMPILE_FLAGS ${CPP4EC_CFLAGS}
    LINK_FLAGS ${CPP4EC_LDFLAGS}
    INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
    CLEAN_DIRECT_OUTPUT 1)

  install(TARGETS cpp4ec_static DESTINATION lib)

  # Create dynamic library (.so) with all basic BMM functions
  add_library(cpp4ec_dynamic SHARED ${CPP4EC_SRC})
  target_link_libraries(cpp4ec_dynamic cpp4ec_rt_static ${SOEMRT_LIBRARIES} ${XENO_NATIVE_LIBRARIES} ${PUGI_XML_LIBRARIES} ${XENO_RTDM_LIBRARIES})
  set_target_properties( cpp4ec_dynamic PROPERTIES
    DEFINE_SYMBOL "CPP4EC_DLL_EXPORT"
    SOVERSION "${CPP4EC_VERSION_MAJOR}"
    VERSION "${CPP4EC_VERSION}"
    OUTPUT_NAME cpp4ec
    COMPILE_FLAGS ${CPP4EC_CFLAGS}
    LINK_FLAGS ${CPP4EC_LDFLAGS}
    INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
    CLEAN_DIRECT_OUTPUT 1)

  # Defaul install
  install(TARGETS cpp4ec_dynamic DESTINATION lib)

  # Headers install
  install(FILES ${CPP4EC_HDRS} DESTINATION include/cpp4ec)

else(ENABLE_RT)

  set(BOOST_LIBS thread date_time system)
  find_package(Boost COMPONENTS ${BOOST_LIBS} REQUIRED)
  include_directories(${Boost_INCLUDE_DIRS})

  add_library(cpp4ec_static STATIC ${CPP4EC_SRC})
  target_link_libraries(cpp4ec_static ${SOEM_LIBRARIES}  ${PUGI_XML_LIBRARIES} ${Boost_LIBRARIES})

#  target_link_libraries(cpp4ec_static cpp4ec_nrt_static ${SOEM_LIBRARIES}  ${PUGI_XML_LIBRARIES} ${Boost_LIBRARIES})
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

#  target_link_libraries(cpp4ec_dynamic cpp4ec_nrt_static ${SOEM_LIBRARIES}  ${PUGI_XML_LIBRARIES} ${Boost_LIBRARIES} )
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


endif(ENABLE_RT)

