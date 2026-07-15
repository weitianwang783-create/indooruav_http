cmake_minimum_required(VERSION 3.0.2)
project(indooruav_http)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  indooruav_msgs
  roscpp
  roslib
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  message_generation
)

find_package(yaml-cpp REQUIRED)

add_service_files(
  FILES
  SendAirline.srv
  SendPic.srv
  TakeoffState.srv
  SendErrorData.srv
  SendFlyOver.srv
  AirlineSync.srv
  CacheAirlineMeta.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS indooruav_msgs roscpp roslib std_msgs std_srvs sensor_msgs geometry_msgs nav_msgs tf message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

add_executable(indooruav_http_server
  node/node_http_server.cpp
  src/http_server.cpp
)

add_executable(indooruav_http_client
  node/node_http_client.cpp
  src/http_client.cpp
)

add_dependencies(indooruav_http_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(indooruav_http_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(indooruav_http_server ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
target_link_libraries(indooruav_http_client ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

install(TARGETS indooruav_http_server indooruav_http_client
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
