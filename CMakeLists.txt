cmake_minimum_required(VERSION 2.8.9)

set(PROJECTNAME object-surface-exploration)
project(${PROJECTNAME})

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})
include(YarpInstallationHelpers)
include(YarpIDL)

#find_package(YARP REQUIRED)
#find_package(iCub REQUIRED)
find_package(Gurls REQUIRED)

find_package(ICUBcontrib)
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()

add_definitions(${YARP_DEFINES}) # add yarp definitions - this contains also -D_REENTRANT

include_directories(${YARP_INCLUDE_DIRS} ${Gurls_INCLUDE_DIRS} ./include/)


set(HEADERS  include/robotControl.h include/exploreObject.h include/explorationStrategyThread.h
include/objectFeaturesThread.h
include/tappingExplorationThread.h include/surfaceModelGP.h include/gpExplorationThread.h
include/exploreGPSurfaceThread.h include/contactSafetyThread.h)

set(SOURCES src/main.cpp src/robotControl.cpp src/exploreObject.cpp
src/objectFeaturesThread.cpp
src/tappingExplorationThread.cpp src/surfaceModelGP.cpp src/exploreGPSurfaceThread.cpp
src/gpExplorationThread.cpp src/contactSafetyThread.cpp ${HEADERS})

#set(CONTEXT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/bin/context" CACHE PATH "Path to the context files")
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

install(TARGETS ${PROJECTNAME} DESTINATION bin)

#install(DIRECTORY ./contexts/
#  DESTINATION ./contexts/${PROJECTNAME}/)


# Search for application files
file(GLOB APP_SCRIPTS ${CMAKE_CURRENT_SOURCE_DIR}/applications/*.xml)
# Search for ini files
file(GLOB APP_CONF
                ${CMAKE_CURRENT_SOURCE_DIR}/contexts/${PROJECTNAME}/*.ini
                ${CMAKE_CURRENT_SOURCE_DIR}/contexts/${PROJECTNAME}/*.xml)


yarp_install(FILES ${APP_SCRIPTS} DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})
yarp_install(FILES ${APP_CONF} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECTNAME})


icubcontrib_add_uninstall_target()










	
