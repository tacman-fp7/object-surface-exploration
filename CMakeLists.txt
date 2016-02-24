cmake_minimum_required(VERSION 2.8.9)

set(PROJECTNAME object-surface-exploration)
project(${PROJECTNAME})

find_package(YARP REQUIRED)
find_package(iCub REQUIRED)
find_package(Gurls REQUIRED)


include_directories(${YARP_INCLUDE_DIRS} ${Gurls_INCLUDE_DIRS} ./include/)


set(HEADERS  include/robotControl.h include/exploreObject.h include/explorationStrategyThread.h include/planarExplorationThread.h
include/maintainContactThread.h include/objectClassifierThread.h include/objectFeaturesThread.h 
include/pidController.h include/tappingExplorationthread.h include/surfacemodelgp.h)

set(SOURCES src/main.cpp src/robotControl.cpp src/exploreObject.cpp
src/maintainContactThread.cpp src/objectFeaturesThread.cpp
src/planarExplorationThread.cpp src/pidController.cpp
src/tappingExplorationthread.cpp src/surfacemodelgp.cpp ${HEADERS})

set(CONTEXT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/bin/context" CACHE PATH "Path to the context files")
set( CMAKE_BUILD_TYPE Debug CACHE STRING
       "Choose the type of build, options are: None Debug Release RelWithDebInfo
MinSizeRel." FORCE )

###################################################################
# Generate source from Apache Thrift Interface Definition Language
###################################################################
set(generated_libs_dir "${CMAKE_CURRENT_SOURCE_DIR}")
yarp_idl_to_dir(robotControl.thrift ${generated_libs_dir} ${SOURCES} ${HEADERS} ${INCLUDE_DIRS})

include(robotControl_thrift.cmake)

add_executable(${PROJECTNAME} ${SOURCES})

target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${Gurls_LIBRARIES})

install(TARGETS ${PROJECTNAME} DESTINATION ./bin)

install(DIRECTORY ./contexts/
  DESTINATION ./contexts/${PROJECTNAME}/)













	
