cmake_minimum_required(VERSION 3.0.2)
project(fr_joint_velocity_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  pluginlib
  roscpp
  franka_hw
  eigen_conversions
  sensor_msgs
)

## System dependencies are found with CMake's conventions

find_package(Eigen3 REQUIRED)
find_package(Franka 0.9.0 QUIET)
if(NOT Franka_FOUND)
  find_package(Franka 0.8.0 REQUIRED)
endif()

# merge Franka + INCLUDE_DIRS in topological order
list_insert_in_workspace_order(catkin_INCLUDE_DIRS ${Franka_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

# # Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/ForceSensitiveCBF.cfg
# )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fr_joint_velocity_controller
  CATKIN_DEPENDS
    controller_interface
    franka_hw
    eigen_conversions
    sensor_msgs
    hardware_interface
    pluginlib
    roscpp
  DEPENDS Franka
)

###########
## Build ##
###########

## Declare a C++ library
add_library(fr_joint_velocity_controller
  src/fr_joint_velocity_controller.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(fr_joint_velocity_controller PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)
target_include_directories(fr_joint_velocity_controller SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(fr_joint_velocity_controller PUBLIC
  include
)

#############
## Install ##
#############


## Mark libraries for installation
install(TARGETS fr_joint_velocity_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Install other files
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES franka_example_controllers_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
