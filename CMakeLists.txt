cmake_minimum_required(VERSION 2.8.3)
project(ultimate_msckf_vio)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
)


#include("cmake/FindGlog.cmake")
#find_package(lib_glog REQUIRED)
#include_directories(SYSTEM ${GLOG_INCLUDE_DIRS})
#list(APPEND LINKER_LIBS ${GLOG_LIBRARIES})


find_package(OpenCV REQUIRED)
set(GLOG_LIBRARY glog)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  feature_tracker_lib
  src/ultimate_msckf_vio/feature_tracker.cpp
)

add_library(
  msckf_estimator_lib
  src/ultimate_msckf_vio/data_manager.cpp
  src/ultimate_msckf_vio/msckf_estimator.cpp
  src/ultimate_msckf_vio/ekf_state.cpp
  src/ultimate_msckf_vio/visual_observation_manager.cpp
  src/ultimate_msckf_vio/feature_bundle.cpp
)

catkin_package()

#add_executable(talker_test src/${PROJECT_NAME}/talker_test.cpp)
#target_link_libraries(talker_test ${catkin_LIBRARIES} )

#add_executable(listenner_test src/${PROJECT_NAME}/listenner_test.cpp)
#target_link_libraries(listenner_test ${catkin_LIBRARIES} )

#add_executable(feature_tracker_node src/${PROJECT_NAME}/feature_tracker_node.cpp)
#target_link_libraries(feature_tracker_node ${catkin_LIBRARIES} ${OpenCV_LIBS} featrue_tracker_lib)
#target_link_libraries(feature_tracker_node PUBLIC ${GLOG_LIBRARY})

add_executable(msckf_estimator_node src/${PROJECT_NAME}/msckf_estimator_node.cpp)
target_link_libraries(msckf_estimator_node ${catkin_LIBRARIES} msckf_estimator_lib)


