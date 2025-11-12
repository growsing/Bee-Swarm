cmake_minimum_required(VERSION 2.8.3)
project(iris_ir_sitl)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  CATKIN_DEPENDS gazebo_ros roscpp std_msgs geometry_msgs 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
)

# 编译红外发射器插件
add_library(infrared_emitter_plugin src/plugins/infrared_emitter_plugin.cpp)
target_link_libraries(infrared_emitter_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
)

# 编译红外接收器插件
add_library(infrared_receiver_plugin src/plugins/infrared_receiver_plugin.cpp)
target_link_libraries(infrared_receiver_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(DIRECTORY launch worlds models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS infrared_emitter_plugin infrared_receiver_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/uav_formation_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)