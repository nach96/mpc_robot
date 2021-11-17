cmake_minimum_required(VERSION 3.0.2)
project(mpc_robot)

set(CMAKE_BUILD_TYPE Debug)


find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
nav_msgs
geometry_msgs
tf
#message_generation
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES mpc_robot
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

set(IPOPT_LIBRARY_DIRS /usr/local/lib)
set(IPOPT_INCLUDE_DIRS /usr/local/include/coin-or)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${IPOPT_INCLUDE_DIRS}
 ${CMAKE_SOURCE_DIR}/include
)
link_directories(
    ${IPOPT_LIBRARY_DIRS} 
)

add_library(myNLPlib_old src/old_mynlp.cpp)
target_link_libraries(myNLPlib_old ipopt)

add_library(myNLPlib_ss src/mynlp.cpp src/mynlp_ss_rk4.cpp)
target_link_libraries(myNLPlib_ss ipopt)

add_library(myNLPlib_ms src/mynlp.cpp src/mynlp_ms.cpp)
target_link_libraries(myNLPlib_ms ipopt)

add_library(myNLPlib_fd src/mynlp.cpp src/mynlp_fd_rk4.cpp)
target_link_libraries(myNLPlib_fd ipopt)

add_executable(oc_ex src/oc_main.cpp)
target_link_libraries(oc_ex myNLPlib_ss myNLPlib_ms myNLPlib_fd myNLPlib_old)
target_link_libraries(oc_ex ${catkin_LIBRARIES})

#add_library(myMPClib_fd src/mpc_node.cpp)
#target_link_libraries(myMPClib_fd myNLPlib_ss myNLPlib_ms myNLPlib_fd myNLPlib_old)




#############
## Install ##
#############



#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mpc_robot.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
