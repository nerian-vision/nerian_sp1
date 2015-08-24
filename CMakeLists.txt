cmake_minimum_required(VERSION 2.8.3)
project(nerian_sp1)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs cv_bridge)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  LIBRARIES visiontransfer
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

#######################
## libvisiontransfer ##
#######################

# Extract sources while configuring
execute_process(COMMAND tar --keep-newer-files -xvJf ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/sp1-software-1_0-src.tar.xz -C ${CMAKE_CURRENT_BINARY_DIR})

# Build sources at runtime
add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/sp1-software-1_0-src/lib/libvisiontransfer.so
    COMMAND ${CMAKE_COMMAND} .
    COMMAND make
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/sp1-software-1_0-src
)

add_custom_target(visiontransfer_stub ALL
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/sp1-software-1_0-src/lib/libvisiontransfer.so
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/sp1-software-1_0-src/bin/spcom
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    PERMISSIONS OWNER_WRITE OWNER_READ OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/sp1-software-1_0-src/lib/libvisiontransfer.so
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${CMAKE_CURRENT_BINARY_DIR}/sp1-software-1_0-src/libvisiontransfer
    ${CMAKE_CURRENT_BINARY_DIR}/sp1-software-1_0-src/spcom/helpers)

# Declare a C++ executable
add_executable(nerian_sp1_node
    src/nerian_sp1_node.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/sp1-software-1_0-src/spcom/helpers/colorcoder.cpp
)

# Add cmake target dependencies of the executable
# same as for the library above
add_dependencies(nerian_sp1_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
    visiontransfer_stub)

# Specify libraries to link a library or executable target against
target_link_libraries(nerian_sp1_node
  ${catkin_LIBRARIES}
  visiontransfer
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# Mark executables and/or libraries for installation
install(TARGETS nerian_sp1_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

