
# EM4CPP header files
set(EM4CPP_HDRS
   soem_master.h
   servos_rt.h
   soem_slave.hpp
   soem_driver_factory.h
   soem_SGDV.hpp
   errors/master_error.hpp
   errors/SGDV_error.hpp
)

# EM4CPP source files 
set(EM4CPP_SRC
   soem_master.cpp
   soem_driver_factory.cpp
   soem_slave.cpp
   soem_SGDV.cpp
   errors/master_error.cpp
)


if(ENABLE_RT)
    include_directories(${XENO_NATIVE_INCLUDE_DIRS})
    include_directories(${SOEMRT_INCLUDE_DIRS})

    add_library(em4cpp_static STATIC ${EM4CPP_SRC})
    target_link_libraries(em4cpp_static ${SOEMRT_LIBRARIES} ${XENO_NATIVE_LIBRARIES})
    set_target_properties(em4cpp_static PROPERTIES
      OUTPUT_NAME em4cpp 
      COMPILE_FLAGS ${EM4CPP_CFLAGS}
      LINK_FLAGS ${EM4CPP_LDFLAGS}
      INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
      CLEAN_DIRECT_OUTPUT 1)

    install(TARGETS em4cpp_static DESTINATION lib)
  
  # Create dynamic library (.so) with all basic BMM functions
  add_library(em4cpp_dynamic SHARED ${EM4CPP_SRC})
  target_link_libraries(em4cpp_dynamic ${SOEMRT_LIBRARIES} ${XENO_NATIVE_LIBRARIES}) 
  set_target_properties( em4cpp_dynamic PROPERTIES
    DEFINE_SYMBOL "EM4CPP_DLL_EXPORT"
    SOVERSION "${EM4CPP_VERSION_MAJOR}"
    VERSION "${EM4CPP_VERSION}"
    OUTPUT_NAME em4cpp
    COMPILE_FLAGS ${EM4CPP_CFLAGS}
    LINK_FLAGS ${EM4CPP_LDFLAGS}
    INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
    CLEAN_DIRECT_OUTPUT 1)

  # Defaul install
  install(TARGETS em4cpp_dynamic DESTINATION lib)


endif()

