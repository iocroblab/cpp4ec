
include_directories(${PROJECT_SOURCE_DIR}/src)
ADD_DEFINITIONS(-std=c++11 )

add_executable(demo demo.cpp)
target_link_libraries(demo cpp4ec_dynamic)

add_executable(demo2 demo2.cpp)
target_link_libraries(demo2 cpp4ec_dynamic)

#  add_dependencies(demo)
#   add_definitions(${EC4C++_CFLAGS})
#  add_definitions(${XENO_NATIVE_CFLAGS})

