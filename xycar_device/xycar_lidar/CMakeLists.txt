cmake_minimum_required(VERSION 2.8.3)
project(xycar_lidar)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(RPLIDAR_SDK_PATH "./sdk/rplidar/")
set(YDLIDAR_SDK_PATH "./sdk/ydlidar/")
#add_definitions(-std=c++11) # Use C++11

find_package(catkin REQUIRED COMPONENTS
  rosconsole
  roscpp
  sensor_msgs
)

#add_subdirectory(sdk)

FILE(GLOB RPLIDAR_SDK_SRC 
  "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/*.cpp"
)

FILE(GLOB YDLIDAR_SDK_SRC 
  "${YDLIDAR_SDK_PATH}/src/impl/unix/*.cpp"
  "${YDLIDAR_SDK_PATH}/src/*.cpp"
  "${YDLIDAR_SDK_PATH}/src/*.c"
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/src
  ${YDLIDAR_SDK_PATH}/include
  ${YDLIDAR_SDK_PATH}/src
  ${RPLIDAR_SDK_PATH}/include
  ${RPLIDAR_SDK_PATH}/src
)

add_executable(xycar_rplidar src/rplidar_node.cpp ${RPLIDAR_SDK_SRC})
target_link_libraries(xycar_rplidar ${catkin_LIBRARIES})

add_executable(xycar_ydlidar src/ydlidar_node.cpp ${YDLIDAR_SDK_SRC})
target_link_libraries(xycar_ydlidar ${catkin_LIBRARIES})

install(TARGETS xycar_rplidar xycar_ydlidar
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch sdk launch rviz sdk
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)


