
# CPP4EC header files
set(CPP4EC_HDRS
      EcError.h
      EcErrorSGDV.h
      EcMaster.h
      EcSlaveFactory.h
      EcSlave.h
      EcSlaveSGDV.h
      servos_rt.h
      EcUtil.h
)

# CPP4EC source files
set(CPP4EC_SRC
      EcError.cpp
      EcErrorSGDV.cpp
      EcMaster.cpp
      EcSlave.cpp
      EcSlaveFactory.cpp
      EcSlaveSGDV.cpp
      EcUtil.cpp
)

if(ENABLE_RT)
    include_directories(${XENO_NATIVE_INCLUDE_DIRS})
    include_directories(${SOEMRT_INCLUDE_DIRS})

    add_library(cpp4ec_static STATIC ${CPP4EC_SRC})
    target_link_libraries(cpp4ec_static ${SOEMRT_LIBRARIES} ${XENO_NATIVE_LIBRARIES} ${PUGI_XML_LIBRARIES})
    set_target_properties(cpp4ec_static PROPERTIES
      OUTPUT_NAME cpp4ec
      COMPILE_FLAGS ${CPP4EC_CFLAGS}
      LINK_FLAGS ${CPP4EC_LDFLAGS}
      INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
      CLEAN_DIRECT_OUTPUT 1)

    install(TARGETS cpp4ec_static DESTINATION lib)

  # Create dynamic library (.so) with all basic BMM functions
  add_library(cpp4ec_dynamic SHARED ${CPP4EC_SRC})
  target_link_libraries(cpp4ec_dynamic ${SOEMRT_LIBRARIES} ${XENO_NATIVE_LIBRARIES} ${PUGI_XML_LIBRARIES})
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


endif()

