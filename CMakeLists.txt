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

set(GLOG_LIBRARY glog)
include_directories(${GLOG_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)

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
  src/ultimate_msckf_vio/visual_ekf_update.cpp
  src/ultimate_msckf_vio/parameter_reader.cpp
  src/ultimate_msckf_vio/feature_tracker.cpp
  src/ultimate_msckf_vio/vio_initializer.cpp
  src/ultimate_msckf_vio/frame.cpp

)

catkin_package()

#add_executable(talker_test src/${PROJECT_NAME}/talker_test.cpp)
#target_link_libraries(talker_test ${catkin_LIBRARIES} )

#add_executable(listenner_test src/${PROJECT_NAME}/listenner_test.cpp)
#target_link_libraries(listenner_test ${catkin_LIBRARIES} )

#add_executable(feature_tracker_node src/${PROJECT_NAME}/feature_tracker_node.cpp)
#target_link_libraries(feature_tracker_node ${catkin_LIBRARIES} ${OpenCV_LIBS} featrue_tracker_lib)
#target_link_libraries(feature_tracker_node PUBLIC ${GLOG_LIBRARY})

target_link_libraries(msckf_estimator_lib PUBLIC ${catkin_LIBRARIES} ${OpenCV_LIBS} ${GLOG_LIBRARY} gflags)


add_executable(msckf_estimator_node app/msckf_estimator_node.cpp)
target_link_libraries(msckf_estimator_node ${catkin_LIBRARIES} msckf_estimator_lib)

#########
# test
#########
#add_executable(test_state_agument test/test_state_agument.cpp)
#target_link_libraries(test_state_agument ${catkin_LIBRARIES} msckf_estimator_lib)

#add_executable(test_geometry test/test_geometry.cpp)
#target_link_libraries(test_geometry ${catkin_LIBRARIES} msckf_estimator_lib)

add_executable(test_jaccobian test/test_jaccobian.cpp)
target_link_libraries(test_jaccobian msckf_estimator_lib)


