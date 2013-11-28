
# EC4C++ header files
set(EC4C++_HDRS
      EcError.h
      EcErrorSGDV.h
      EcMaster.h
      EcSlaveFactory.h
      EcSlave.h
      EcSlaveSGDV.h
      servos_rt.h
)

# EC4C++ source files
set(EC4C++_SRC
      EcError.cpp
      EcMaster.cpp
      EcSlave.cpp
      EcSlaveFactory.cpp
      EcSlaveSGDV.cpp
)


if(ENABLE_RT)
    include_directories(${XENO_NATIVE_INCLUDE_DIRS})
    include_directories(${SOEMRT_INCLUDE_DIRS})

    add_library(ec4c++_static STATIC ${EC4C++_SRC})
    target_link_libraries(ec4c++_static ${SOEMRT_LIBRARIES} ${XENO_NATIVE_LIBRARIES})
    set_target_properties(ec4c++_static PROPERTIES
      OUTPUT_NAME ec4c++
      COMPILE_FLAGS ${EC4C++_CFLAGS}
      LINK_FLAGS ${EC4C++_LDFLAGS}
      INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
      CLEAN_DIRECT_OUTPUT 1)

    install(TARGETS ec4c++_static DESTINATION lib)

  # Create dynamic library (.so) with all basic BMM functions
  add_library(ec4c++_dynamic SHARED ${EC4C++_SRC})
  target_link_libraries(ec4c++_dynamic ${SOEMRT_LIBRARIES} ${XENO_NATIVE_LIBRARIES})
  set_target_properties( ec4c++_dynamic PROPERTIES
    DEFINE_SYMBOL "EC4CPP_DLL_EXPORT"
    SOVERSION "${EC4C++_VERSION_MAJOR}"
    VERSION "${EC4C++_VERSION}"
    OUTPUT_NAME ec4c++
    COMPILE_FLAGS ${EC4C++_CFLAGS}
    LINK_FLAGS ${EC4C++_LDFLAGS}
    INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
    CLEAN_DIRECT_OUTPUT 1)

  # Defaul install
  install(TARGETS ec4c++_dynamic DESTINATION lib)


endif()

