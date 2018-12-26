cmake_minimum_required(VERSION 2.8.3)
project(parking_path_planning)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cmake_modules
  message_generation
  pcl_ros
)
find_package(Eigen REQUIRED)
#####################################PCL###########################################

find_package(PCL REQUIRED QUIET COMPONENTS common)
##################################### QT ##########################################
find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
SET(CMAKE_CXX_FLAGS "-std=c++0x")

##################################### OMPL  ##########################################
#find_package(OMPL)
set(OMPL_LIBRARIES "/home/insightque/01_Git/ompl-1.4.1-Source/build/Release/lib/libompl.so")
set(OMPL_INCLUDE_DIRS "/home/insightque/01_Git/ompl-1.4.1-Source/src/")
set(OMPL_INCLUDE_BASE_DIRS "/home/insightque/01_Git/ompl-1.4.1-Source/src/ompl/base/")

MESSAGE( STATUS "OMPL_LIBRARY_DIRS: " ${OMPL_LIBRARIES} )
MESSAGE( STATUS "OMPL_INCLUDE_DIRS: " ${OMPL_INCLUDE_DIRS} )

include(${QT_USE_FILE})

###################################################################################
find_package(Boost REQUIRED COMPONENTS signals)

option(WITH_OPENMP      "Build with OpenMP support for multithreading"                  ON)
# add OpenMP support
    if (WITH_OPENMP)
find_package(OpenMP QUIET)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")
    if (NOT OPENMP_FOUND)
    set(WITH_OPENMP OFF)
    endif()
endif()

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
#add_message_files(
#   DIRECTORY msg
#   FILES
#   UserMessage.msg
#)


## Generate services in the 'srv' folder
# add_service_files(
#   DIRECTORY srv
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# ) 

#commented by shinsh -- should be added for user msg or srv

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
#  INCLUDE_DIRS inc
#  LIBRARIES lib
  CATKIN_DEPENDS 
    roscpp 
    std_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIR}
  ${OMPL_INCLUDE_DIRS}
)


######################## test_ui_node #################################
set( SRC_FILES_QT_UI_NODE src/parking_path_planning.cpp)

## Declare a cpp executable
add_executable(parking_path_planning ${SRC_FILES_QT_UI_NODE} )

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(ompl_qt_pkg_node ompl_qt_pkg_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(parking_path_planning
   ${catkin_LIBRARIES}
   ${Eigen_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OMPL_LIBRARIES}
   ${boost_LIBRARIES}
)
#######################################################################


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ompl_qt_pkg ompl_qt_pkg_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ompl_qt_pkg.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
