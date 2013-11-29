
include_directories(${PROJECT_SOURCE_DIR}/src)

add_executable(demo demo.cpp)
target_link_libraries(demo ec4cpp_dynamic)
#  add_dependencies(demo)
#   add_definitions(${EC4C++_CFLAGS})
#  add_definitions(${XENO_NATIVE_CFLAGS})


#


