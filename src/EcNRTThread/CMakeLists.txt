
# CPP4EC header files
set(CPP4EC_NRT_HDR
      EcNRTThread.h
)

# CPP4EC source files
set(CPP4EC_NRT_SRC
      EcNRTThread.cpp
   

)
#set(BOOST_LIBS thread date_time system)
#find_package(Boost COMPONENTS ${BOOST_LIBS} REQUIRED)
#include_directories(${Boost_INCLUDE_DIRS})
add_definitions(-fPIC)
if(ENABLE_RT)
    add_library(cpp4ec_nrt_static STATIC ${CPP4EC_NRT_SRC})
endif()

