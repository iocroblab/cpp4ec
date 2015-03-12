
# CPP4EC header files
set(CPP4EC_RT_HDR
      EcRTThread.h
)

# CPP4EC source files
set(CPP4EC_RT_SRC
      EcFRTThread.c
   

)

add_definitions(-fPIC)
#if(ENABLE_RT)
    add_library(cpp4ec_rt_static STATIC ${CPP4EC_RT_SRC})
#endif()

