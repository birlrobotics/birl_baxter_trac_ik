cmake_minimum_required(VERSION 2.8.3)
project(trac_ik_examples)

find_package(catkin REQUIRED
  COMPONENTS
    trac_ik_lib
    catkin
    cmake_modules
    geometry_msgs
    kdl_conversions
    orocos_kdl
    tf
    message_generation
    std_msgs
)

find_package(Boost REQUIRED COMPONENTS date_time)
find_package(orocos_kdl REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    trac_ik_lib
  DEPENDS
    Boost
    orocos_kdl
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)


add_executable(baxter_trac_ik  src/baxter_trac_ik.cpp)
add_executable(joint_pub  src/baxter_joint_pub.cpp)
add_executable(broadcast  src/broadcast.cpp)
add_executable(listener_test src/listener.cpp)


target_link_libraries(joint_pub
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)
target_link_libraries(baxter_trac_ik
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)
target_link_libraries(broadcast
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)
target_link_libraries(listener_test
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)
install(TARGETS baxter_trac_ik
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
