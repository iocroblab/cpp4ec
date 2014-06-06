
# CPP4EC header files
set(CPP4EC_NRT_HDR
      EcNRTThread.h
)

# CPP4EC source files
set(CPP4EC_NRT_SRC
      EcNRTThread.cpp
   

)

add_definitions(-fPIC)
add_library(cpp4ec_nrt_static STATIC ${CPP4EC_NRT_SRC})

