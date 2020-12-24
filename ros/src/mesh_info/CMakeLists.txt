cmake_minimum_required(VERSION 3.10.0)
project(mesh_info)

# set this to path to AirSim root folder if you want your catkin workspace in a custom directory
set(AIRSIM_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../../AirSim/)

LIST(APPEND CMAKE_MODULE_PATH "${AIRSIM_ROOT}/cmake/cmake-modules") 
INCLUDE("${AIRSIM_ROOT}/cmake/cmake-modules/CommonSetup.cmake")

CommonSetup()

#add_subdirectory(${AIRSIM_ROOT}/external/rpclib/rpclib-2.2.1 ${CMAKE_CURRENT_BINARY_DIR}/rpclib)
add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirLib)
add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" MavLinkCom)

CommonSetup()

IncludeEigen()

SetupConsoleBuild()

set(CMAKE_CXX_STANDARD 11)
set(CXX_EXP_LIB "-nostdinc++ -I/usr/include/c++/8 -I/usr/include/x86_64-linux-gnu/c++/8 -nodefaultlibs 
-l/usr/lib/x86_64-linux-gnu/libc++.so -l/usr/lib/x86_64-linux-gnu/libc++abi.so
-lm -lc -lgcc_s -lgcc  
-lstdc++fs -fmax-errors=10 -Wnoexcept -Wstrict-null-sentinel") 

set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/rpclib-2.2.1/include")
set(RPC_LIB rpc) # name of .a file with lib prefix
message(STATUS "found RPC_LIB_INCLUDES=${RPC_LIB_INCLUDES}")

# todo eigen3 in AirLib already
# find_package(Eigen3 REQUIRED) 
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  mavros_msgs
  message_generation
  nav_msgs
  # nodelet
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
)

include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
  ${AIRSIM_ROOT}/AirLib/deps/eigen3
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS cmake_modules sensor_msgs geometry_msgs roslib
)

add_executable(mesh_info_node src/mesh_info_node.cc)
set_target_properties(mesh_info_node PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
target_link_libraries(mesh_info_node ${catkin_LIBRARIES})
target_link_libraries(mesh_info_node AirLib)
target_link_libraries(mesh_info_node ${RPC_LIB})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
  )