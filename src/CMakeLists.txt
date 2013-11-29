
# EC4CPP header files
set(EC4CPP_HDRS
      EcError.h
      EcErrorSGDV.h
      EcMaster.h
      EcSlaveFactory.h
      EcSlave.h
      EcSlaveSGDV.h
      servos_rt.h
)

# EC4CPP source files
set(EC4CPP_SRC
      EcError.cpp
      EcMaster.cpp
      EcSlave.cpp
      EcSlaveFactory.cpp
      EcSlaveSGDV.cpp
)


if(ENABLE_RT)
    include_directories(${XENO_NATIVE_INCLUDE_DIRS})
    include_directories(${SOEMRT_INCLUDE_DIRS})

    add_library(ec4cpp_static STATIC ${EC4CPP_SRC})
    target_link_libraries(ec4cpp_static ${SOEMRT_LIBRARIES} ${XENO_NATIVE_LIBRARIES})
    set_target_properties(ec4cpp_static PROPERTIES
      OUTPUT_NAME ec4cpp
      COMPILE_FLAGS ${EC4CPP_CFLAGS}
      LINK_FLAGS ${EC4CPP_LDFLAGS}
      INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
      CLEAN_DIRECT_OUTPUT 1)

    install(TARGETS ec4cpp_static DESTINATION lib)

  # Create dynamic library (.so) with all basic BMM functions
  add_library(ec4cpp_dynamic SHARED ${EC4CPP_SRC})
  target_link_libraries(ec4cpp_dynamic ${SOEMRT_LIBRARIES} ${XENO_NATIVE_LIBRARIES})
  set_target_properties( ec4cpp_dynamic PROPERTIES
    DEFINE_SYMBOL "EC4CPP_DLL_EXPORT"
    SOVERSION "${EC4CPP_VERSION_MAJOR}"
    VERSION "${EC4CPP_VERSION}"
    OUTPUT_NAME ec4cpp
    COMPILE_FLAGS ${EC4CPP_CFLAGS}
    LINK_FLAGS ${EC4CPP_LDFLAGS}
    INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
    CLEAN_DIRECT_OUTPUT 1)

  # Defaul install
  install(TARGETS ec4cpp_dynamic DESTINATION lib)


endif()

