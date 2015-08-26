cmake_minimum_required(VERSION 2.8.3)
project(nerian_sp1)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs cv_bridge
    message_generation)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(SDL REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
add_message_files(FILES StereoCameraInfo.msg)

# Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES sensor_msgs)

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
#  INCLUDE_DIRS ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  LIBRARIES visiontransfer
  CATKIN_DEPENDS message_runtime sensor_msgs
#  DEPENDS system_lib
)

#######################
## libvisiontransfer ##
#######################

set(VT_VERSION 1_0_1)

# Extract sources while configuring
execute_process(COMMAND tar --keep-newer-files --warning none -xJf ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/sp1-software-${VT_VERSION}-src.tar.xz -C ${CMAKE_CURRENT_BINARY_DIR})

# Re-extract source files if updated
add_custom_target(untar_src ALL
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/sp1-software-${VT_VERSION}-src.tar.xz
    COMMAND tar --keep-newer-files --warning none -xJf ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/sp1-software-${VT_VERSION}-src.tar.xz -C ${CMAKE_CURRENT_BINARY_DIR}
)

# Build sources at runtime
add_custom_target(visiontransfer_stub ALL
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/sp1-software-${VT_VERSION}-src.tar.xz
    COMMAND ${CMAKE_COMMAND} .
    COMMAND make
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/sp1-software-${VT_VERSION}-src
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/sp1-software-${VT_VERSION}-src/bin/spcom
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    PERMISSIONS OWNER_WRITE OWNER_READ OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/sp1-software-${VT_VERSION}-src/lib/libvisiontransfer.so
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

link_directories(${CMAKE_CURRENT_BINARY_DIR}/sp1-software-${VT_VERSION}-src/lib)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${CMAKE_CURRENT_BINARY_DIR}/sp1-software-${VT_VERSION}-src/libvisiontransfer
    ${CMAKE_CURRENT_BINARY_DIR}/sp1-software-${VT_VERSION}-src/spcom/helpers
    ${Boost_INCLUDE_DIRS} ${SDL_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})

# Declare a C++ executable
add_executable(nerian_sp1_node
    src/nerian_sp1_node.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/sp1-software-${VT_VERSION}-src/spcom/helpers/colorcoder.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/sp1-software-${VT_VERSION}-src/spcom/helpers/sdlwindow.cpp
)

# Add cmake target dependencies of the executable
# same as for the library above
add_dependencies(nerian_sp1_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
    visiontransfer_stub)

# Specify libraries to link a library or executable target against
target_link_libraries(nerian_sp1_node ${catkin_LIBRARIES} ${Boost_LIBRARIES}
  ${SDL_LIBRARY} ${OpenCV_LIBS} visiontransfer)

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

