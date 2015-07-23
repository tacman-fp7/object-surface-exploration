cmake_minimum_required(VERSION 2.8)

set(PROJECTNAME object-surface-exploration)
project(${PROJECTNAME})

find_package(YARP)

include_directories(${YARP_INCLUDE_DIRS})

add_executable(${PROJECTNAME} src/main.cpp)

target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES})

install(TARGETS ${PROJECTNAME} DESTINATION ./bin)














	
