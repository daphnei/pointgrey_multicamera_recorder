cmake_minimum_required(VERSION 2.8.3)
project(poll_cameras)

add_definitions(-std=c++11)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  dynamic_reconfigure
  flea3 REQUIRED        # needed for the test module
)

generate_dynamic_reconfigure_options(
  cfg/ExposureControlDyn.cfg
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )


#add_executable(${PROJECT_NAME} src/CamTest.cpp)

add_library(${PROJECT_NAME} src/CamTest.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

add_executable(${PROJECT_NAME}_node src/broadcast_cameras_node.cpp src/CamTest.cpp)

# make sure configure headers are built before any node using them
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )

#
# build the test node as well
#
#add_executable(cam_test_node src/cam_test_node.cpp src/CamTest.cpp)

#target_link_libraries(cam_test_node
#  ${PROJECT_NAME}
#  ${catkin_LIBRARIES}
# )

