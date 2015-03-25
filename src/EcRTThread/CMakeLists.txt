
# CPP4EC header files
set(RTECTASK_HDR
      EcRTThread.h
)

# CPP4EC source files
set(RTECTASK_SRC
      EcRTThread.c
)

add_definitions(-fPIC)

add_library(rtectask_hrt_static STATIC ${RTECTASK_SRC})
set_target_properties(rtectask_hrt_static PROPERTIES
    COMPILE_DEFINITIONS "HRT"
    CLEAN_DIRECT_OUTPUT 1)

add_library(rtectask_rt_static STATIC ${RTECTASK_SRC})
set_target_properties(rtectask_rt_static PROPERTIES
    COMPILE_DEFINITIONS "RT"
    CLEAN_DIRECT_OUTPUT 1)
