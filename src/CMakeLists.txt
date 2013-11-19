
# BMM header files
set(SERVOS_HDRS
  soem_master.h
  servos_rt.h
  soem_slave.hpp
  soem_driver_factory.h
  soem_SGDV.hpp

)

# BMM source files (without examples)
set(SERVOS_SRC
  soem_master.cpp
  soem_driver_factory.cpp
  soem_slave.cpp
  soem_SGDV.cpp
)

include_directories(${RTNET_INCLUDE_DIR})

# add_library(servos_static STATIC ${SERVOS_SRC})
# set_target_properties(servos_static PROPERTIES 
#     OUTPUT_NAME servos
# # Set accumulated compilation flags: (.so and .a)
#   COMPILE_FLAGS "${XENO_CFLAGS_RTDM}"
#   LINK_FLAGS "${CMAKE_LD_FLAGS}"
#    INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
#    CLEAN_DIRECT_OUTPUT 1)
# 
# # Create dynamic library (.so) with all basic SERVOS functions
# add_library(servos_dynamic SHARED ${SERVOS_SRC})
# target_link_libraries (servos_dynamic ${CMAKE_SHARED_EXTRA_LIBS})
# set_target_properties( servos_dynamic PROPERTIES
#   DEFINE_SYMBOL "SERVOS_DLL_EXPORT"
#   SOVERSION "${BMM_VERSION_MAJOR}.${BMM_VERSION_MINOR}.${BMM_VERSION_PATCH}"
#   VERSION "${BMM_VERSION}"
#   OUTPUT_NAME servos
# # Set accumulated compilation flags: (.so and .a)
#   COMPILE_FLAGS "${XENO_CFLAGS_RTDM}"
#   LINK_FLAGS "${CMAKE_SHARED_LINKER_FLAGS}"
#   INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
#   CLEAN_DIRECT_OUTPUT 1)

if(ENABLE_RT)
    add_library(servos_static_rt STATIC ${SERVOS_SRC})
    target_link_libraries(servos_static_rt ${CMAKE_LINKER_FLAGS_RTNET} soemrt)
    set_target_properties(servos_static_rt PROPERTIES
      OUTPUT_NAME servosrt 
      COMPILE_FLAGS "${XENO_CFLAGS_RTDM}"
      #LINK_FLAGS "${CMAKE_LINKER_FLAGS_RTNET}"
      INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
      CLEAN_DIRECT_OUTPUT 1)

    install(TARGETS servos_static_rt DESTINATION lib)
  
  # Create dynamic library (.so) with all basic BMM functions
  add_library(servos_dynamic_rt SHARED ${SERVOS_SRC})
  target_link_libraries(servos_dynamic_rt ${CMAKE_LINKER_FLAGS_RTNET} soemrt)
  set_target_properties( servos_dynamic_rt PROPERTIES
    DEFINE_SYMBOL "BMM_DLL_EXPORT"
    SOVERSION "${BMM_VERSION_MAJOR}.${BMM_VERSION_MINOR}.${BMM_VERSION_PATCH}"
    VERSION "${BMM_VERSION}"
    OUTPUT_NAME servosrt
    COMPILE_FLAGS "${XENO_CFLAGS_RTDM}"
    #LINK_FLAGS "${CMAKE_LINKER_FLAGS_RTNET}"
    INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
    CLEAN_DIRECT_OUTPUT 1)

  # Defaul install
  install(TARGETS servos_dynamic_rt DESTINATION lib)


endif()

