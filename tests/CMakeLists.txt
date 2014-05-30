
include_directories(${PROJECT_SOURCE_DIR}/src)
ADD_DEFINITIONS(-std=c++11 )

add_executable(demo1 demo1.cpp)
target_link_libraries(demo1 cpp4ec_dynamic)

add_executable(demo2 demo2.cpp)
target_link_libraries(demo2 cpp4ec_dynamic)

add_executable(demo3 demo3.cpp)
target_link_libraries(demo3 cpp4ec_dynamic)

add_executable(demo4 demo4.cpp)
target_link_libraries(demo4 cpp4ec_dynamic)

add_executable(demoCarril demoCarril.cpp )
target_link_libraries(demoCarril cpp4ec_dynamic )

add_executable(demoCarrilVelocity demoCarrilVelocity.cpp)
target_link_libraries(demoCarrilVelocity cpp4ec_dynamic)

add_executable(demoCarrilPosition demoCarrilPosition.cpp)
target_link_libraries(demoCarrilPosition cpp4ec_dynamic  ${Boost_LIBRARIES})

#add_executable(demoCarril2 demoCarril2.cpp )
#target_link_libraries(demoCarril2 cpp4ec_dynamic )

#set(BOOST_LIBS thread date_time system)
#find_package(Boost COMPONENTS ${BOOST_LIBS} REQUIRED)
#add_executable(demoDialbox demoDialbox.cpp /srv/robotica/carril/dialbox/src/dialbox.cpp)
#target_link_libraries(demoDialbox cpp4ec_dynamic ${Boost_LIBRARIES})

#add_executable(demoDialboxPos demoDialboxPos.cpp /srv/robotica/carril/dialbox/src/dialbox.cpp)
#target_link_libraries(demoDialboxPos cpp4ec_dynamic ${Boost_LIBRARIES})

#add_executable(demoDialboxPos2 demoDialboxPos2.cpp /srv/robotica/carril/dialbox/src/dialbox.cpp)
#target_link_libraries(demoDialboxPos2 cpp4ec_dynamic ${Boost_LIBRARIES})

#add_executable(demoInteractive demoInteractive.cpp /srv/robotica/carril/dialbox/src/dialbox.cpp)
#target_link_libraries(demoInteractive cpp4ec_dynamic ${Boost_LIBRARIES})




