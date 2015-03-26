
include_directories(${PROJECT_SOURCE_DIR}/src)
ADD_DEFINITIONS(-std=c++11 )

add_executable(demo1 demo1.cpp)
target_link_libraries(demo1 cpp4ec_dynamic)

add_executable(demo2 demo2.cpp)
target_link_libraries(demo2 cpp4ec_hrt_dynamic)

add_executable(demo3 demo3.cpp)
target_link_libraries(demo3 cpp4ec_dynamic)

add_executable(demo4 demo4.cpp)
target_link_libraries(demo4 cpp4ec_dynamic)





