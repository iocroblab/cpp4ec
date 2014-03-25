
include_directories(${PROJECT_SOURCE_DIR}/src)
ADD_DEFINITIONS(-std=c++11 )

add_executable(demo1 demo1.cpp)
target_link_libraries(demo1 cpp4ec_dynamic)

add_executable(demo2 demo2.cpp)
target_link_libraries(demo2 cpp4ec_dynamic)



