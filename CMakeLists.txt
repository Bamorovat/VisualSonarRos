cmake_minimum_required(VERSION 3.0.2)
project(visual_sonar_ros)

## Project Information
set(PROJECT_VERSION "2.0.0")
set(PROJECT_DESCRIPTION "Visual Sonar ROS - Mobile robot navigation using omnidirectional vision and virtual sonar algorithms")
set(PROJECT_AUTHOR "Mohammad Hossein Bamorovat Abadi")
set(PROJECT_EMAIL "m.bamorovvat@gmail.com")
set(PROJECT_WEBSITE "https://bamorovat.com")

## Compile as C++14 for modern features and better performance
add_compile_options(-std=c++14 -Wall -Wextra -O3)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  dynamic_reconfigure
)

## Find OpenCV with minimum required version
find_package(OpenCV 3.0 REQUIRED)

## Find required system dependencies
find_package(Threads REQUIRED)

## Check OpenCV version and components
if(OpenCV_VERSION VERSION_LESS "3.0")
  message(FATAL_ERROR "OpenCV version ${OpenCV_VERSION} is not supported. Please use OpenCV 3.0 or higher.")
endif()

## Display build information
message(STATUS "Visual Sonar ROS v${PROJECT_VERSION}")
message(STATUS "Author: ${PROJECT_AUTHOR} <${PROJECT_EMAIL}>")
message(STATUS "Website: ${PROJECT_WEBSITE}")
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

################################################
## Optional Dynamic Reconfigure Configuration ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/VisualSonar.cfg
# )
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
#   nav_msgs#   sensor_msgs#   std_msgs
# )

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
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS 
    cv_bridge
    geometry_msgs
    image_transport
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf
    dynamic_reconfigure
  DEPENDS 
    OpenCV
    Threads
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Define source files for the library
set(LIBRARY_SOURCES
  src/Visual_Sonar.cpp
  src/Navigation.cpp
  src/Light_Reflection_Elimination.cpp
  src/edge_detection.cpp
)

## Define header files for installation
set(HEADER_FILES
  include/visual_sonar_ros/Visual_Sonar.h
  include/visual_sonar_ros/Navigation.h
  include/visual_sonar_ros/light_reflection_elimination.h
  include/visual_sonar_ros/edge_detection.h
  include/visual_sonar_ros/visual_sonar_ros_node.h
)

## Create shared library with all Visual Sonar algorithms
add_library(${PROJECT_NAME}_lib SHARED
  ${LIBRARY_SOURCES}
)

## Add cmake target dependencies for the library
add_dependencies(${PROJECT_NAME}_lib 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

## Link libraries to the shared library
target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  Threads::Threads
)

## Set library properties
set_target_properties(${PROJECT_NAME}_lib PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION 2
  OUTPUT_NAME "visual_sonar_algorithms"
)

## Declare the main executable
add_executable(${PROJECT_NAME}_node
  src/Visual_Sonar_Ros.cpp
)

## Set executable output name for easier usage
set_target_properties(${PROJECT_NAME}_node PROPERTIES 
  OUTPUT_NAME "visual_sonar_node"
)

## Add cmake target dependencies for the executable
add_dependencies(${PROJECT_NAME}_node 
  ${PROJECT_NAME}_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

## Link libraries to the executable
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  Threads::Threads
)

## Optional: Create additional utility executables
# Uncomment if you want to create test/debug executables
# add_executable(${PROJECT_NAME}_test_edge_detection test/test_edge_detection.cpp)
# target_link_libraries(${PROJECT_NAME}_test_edge_detection ${PROJECT_NAME}_lib ${catkin_LIBRARIES})


#############
## Install ##
#############

## Install executables and libraries
install(TARGETS ${PROJECT_NAME}_lib ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

## Install configuration files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE
)

## Install documentation
install(FILES
  README.md
  LICENSE
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Install example parameter files
install(DIRECTORY params/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

## Enable testing if requested
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  
  ## Add gtest based cpp unit tests
  catkin_add_gtest(${PROJECT_NAME}_unit_tests 
    test/test_visual_sonar.cpp
    test/test_edge_detection.cpp  
    test/test_navigation.cpp
    test/test_light_reflection.cpp
  )
  
  if(TARGET ${PROJECT_NAME}_unit_tests)
    target_link_libraries(${PROJECT_NAME}_unit_tests 
      ${PROJECT_NAME}_lib
      ${catkin_LIBRARIES}
    )
  endif()
  
  ## Add rostest integration tests
  add_rostest_gtest(${PROJECT_NAME}_integration_tests
    test/integration_test.launch
    test/test_integration.cpp
  )
  
  if(TARGET ${PROJECT_NAME}_integration_tests)
    target_link_libraries(${PROJECT_NAME}_integration_tests 
      ${PROJECT_NAME}_lib
      ${catkin_LIBRARIES}
    )
  endif()
endif()

#######################
## Code Quality & CI ##
#######################

## Optional: Add code formatting and linting
# find_program(CLANG_FORMAT "clang-format")
# if(CLANG_FORMAT)
#   add_custom_target(format
#     COMMAND ${CLANG_FORMAT} -i ${LIBRARY_SOURCES} ${HEADER_FILES}
#     WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
#     COMMENT "Running clang-format"
#   )
# endif()

## Display final build summary
message(STATUS "=== Visual Sonar ROS Build Configuration ===")
message(STATUS "Version: ${PROJECT_VERSION}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: C++14")
message(STATUS "OpenCV: ${OpenCV_VERSION}")
message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "=============================================")
