cmake_minimum_required(VERSION 2.8.3)
project(mrb_visualization_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  matlab_ros_bridge
  roscpp
  visualization_msgs
  cmake_module_matlab
)

find_package(Matlab REQUIRED)
if(MATLAB_FOUND)
    include_directories(${MATLAB_INCLUDE_DIRS})
    include_directories(${SIMULINK_INCLUDE_DIRS})
endif()

add_definitions(-DMATLAB_MEX_FILE)

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
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
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
#   visualization_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
#  INCLUDE_DIRS include
#  LIBRARIES mrb_visualization_msgs
   CATKIN_DEPENDS roscpp visualization_msgs matlab_ros_bridge
   DEPENDS boost Matlab
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(SOURCES
  rmsg_pub_MarkerArray.cpp
)

string(FIND ${CATKIN_DEVEL_PREFIX} ${PROJECT_NAME} FoundPosition REVERSE)
if(FoundPosition EQUAL -1)
	set(${PROJECT_NAME}_MEX_DESTINATION ${CATKIN_DEVEL_PREFIX}/${PROJECT_NAME}/${CATKIN_PACKAGE_LIB_DESTINATION})
else()
	set(${PROJECT_NAME}_MEX_DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION})
endif()

foreach(SOURCE ${SOURCES})
    	string(REGEX REPLACE ".cpp" "" TARGET_NAME "${SOURCE}")
    	message(STATUS "Adding target ${TARGET_NAME} for ${SOURCE}")
    	add_library(${TARGET_NAME} SHARED src/${PROJECT_NAME}/${SOURCE})
    	set_target_properties(${TARGET_NAME} PROPERTIES PREFIX "" SUFFIX ".mexa64")
	set_target_properties(${TARGET_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${${PROJECT_NAME}_MEX_DESTINATION})
    	target_link_libraries(${TARGET_NAME} ${catkin_LIBRARIES})
    	install(TARGETS ${TARGET_NAME}
			ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
			LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
			RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
		)
endforeach(SOURCE)

## Declare a C++ executable
# add_executable(mrb_visualization_msgs_node src/mrb_visualization_msgs_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(mrb_visualization_msgs_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(mrb_visualization_msgs_node
#   ${catkin_LIBRARIES}
# )

#get_property(MAKE_INFO_SOURCE_PATH GLOBAL PROPERTY MAKE_INFO_SOURCE_PATH)
set(MAKE_INFO_SOURCE_PATH "'${PROJECT_SOURCE_DIR}/src/${PROJECT_NAME}'")

#get_property(MAKE_INFO_INCLUDE_PATH GLOBAL PROPERTY MAKE_INFO_INCLUDE_PATH)
#set(MAKE_INFO_INCLUDE_PATH "'${Boost_INCLUDE_DIRS}' '${catkin_INCLUDE_DIRS}'")
set(MAKE_INFO_INCLUDE_PATH "'${catkin_INCLUDE_DIRS}'")
string(REGEX REPLACE ";" "' '" MAKE_INFO_INCLUDE_PATH "${MAKE_INFO_INCLUDE_PATH}")

#get_property(MAKE_INFO_LINK_LIBS_OBJS GLOBAL PROPERTY MAKE_INFO_LINK_LIBS_OBJS)

foreach (catkin_LIBERARY ${catkin_LIBRARIES})
	if(IS_ABSOLUTE ${catkin_LIBERARY})
		set(${catkin_LIBERARY}_path ${catkin_LIBERARY})
	else(IS_ABSOLUTE ${catkin_LIBERARY})
		find_library(${catkin_LIBERARY}_path ${catkin_LIBERARY})
	endif(IS_ABSOLUTE ${catkin_LIBERARY})
	list(APPEND MAKE_INFO_LINK_LIBS_OBJS ${${catkin_LIBERARY}_path})
endforeach(catkin_LIBERARY)
string(REGEX REPLACE ";" "' '" MAKE_INFO_LINK_LIBS_OBJS "'${MAKE_INFO_LINK_LIBS_OBJS}'")

configure_file(${RTWMAKECFG_TEMPLATE_PATH} ${${PROJECT_NAME}_MEX_DESTINATION}/rtwmakecfg.m)

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

## Mark cpp header files for installation
install(DIRECTORY src/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
   FILES_MATCHING PATTERN "*.cpp"
   PATTERN ".svn" EXCLUDE
   PATTERN "CMakeFiles" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
   ${CATKIN_DEVEL_PREFIX}/${PROJECT_NAME}/${CATKIN_PACKAGE_LIB_DESTINATION}/rtwmakecfg.m
   DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mrb_active_estimation_msgs.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
