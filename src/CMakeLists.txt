
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

#MINI_XML
find_path(MINI_XML_INCLUDE_DIRS mxml.h
    PATHS /user/local/include
	  /usr/include)

if(MINI_XML_INCLUDE_DIRS)
  message("Found MINI_XML_INCLUDE_DIRS  on: ${MINI_XML_INCLUDE_DIRS}")
else(MINI_XML_INCLUDE_DIRS)
  message("Not found MINI_XML_INCLUDE_DIRS")
endif(MINI_XML_INCLUDE_DIRS)

find_library (MINI_XML_LIBRARIES
		NAMES mxml libmxml
		PATHS /user/local/lib
		      /usr/local/lib
		      /usr/lib)
if(MINI_XML_LIBRARIES)
  message("Found MINI_XML_LIBRARIES on:" ${MINI_XML_LIBRARIES})
else(MINI_XML_LIBRARIES)
  message("Not found MINI_XML_LIBRARIES")
endif(MINI_XML_LIBRARIES)


include_directories(${MINI_XML_INCLUDE_DIRS})

if(ENABLE_RT)
    include_directories(${XENO_NATIVE_INCLUDE_DIRS})
    include_directories(${SOEMRT_INCLUDE_DIRS})

    add_library(ec4cpp_static STATIC ${EC4CPP_SRC})
    target_link_libraries(ec4cpp_static ${SOEMRT_LIBRARIES} ${XENO_NATIVE_LIBRARIES} ${MINI_XML_LIBRARIES})
    set_target_properties(ec4cpp_static PROPERTIES
      OUTPUT_NAME ec4cpp
      COMPILE_FLAGS ${EC4CPP_CFLAGS}
      LINK_FLAGS ${EC4CPP_LDFLAGS}
      INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
      CLEAN_DIRECT_OUTPUT 1)

    install(TARGETS ec4cpp_static DESTINATION lib)

  # Create dynamic library (.so) with all basic BMM functions
  add_library(ec4cpp_dynamic SHARED ${EC4CPP_SRC})
  target_link_libraries(ec4cpp_dynamic ${SOEMRT_LIBRARIES} ${XENO_NATIVE_LIBRARIES} ${MINI_XML_LIBRARIES})
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

