cmake_minimum_required(VERSION 2.8.9)

set(PROJECTNAME object-surface-exploration)
project(${PROJECTNAME})

find_package(YARP REQUIRED)

include_directories(${YARP_INCLUDE_DIRS} ./include/)

set(SOURCES src/main.cpp src/robotControlServer.cpp src/robotControl.cpp)
set(HEADERS include/robotControlServer.h include/robotControl.h)



###################################################################
# Generate source from Apache Thrift Interface Definition Language
###################################################################
set(generated_libs_dir "${CMAKE_CURRENT_SOURCE_DIR}")
yarp_idl_to_dir(robotControl.thrift ${generated_libs_dir} ${SOURCES} ${HEADERS} ${INCLUDE_DIRS})

include(robotControl_thrift.cmake)

add_executable(${PROJECTNAME} ${SOURCES})

target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES})

install(TARGETS ${PROJECTNAME} DESTINATION ./bin)














	
