cmake_minimum_required(VERSION 2.8.3)
project(mission7_state_estimator)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
)


find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(sophus REQUIRED)


include_directories(
	${OpenCV_INCLUDE_DIRS}
	include
	include/invio
	/usr/include/eigen3
	${catkin_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS}
	${sophus_INCLUDE_DIRS}
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES mission7_state_estimator
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########


add_library(mission7_state_estimator_types include/mission7_state_estimator/Types.h)
set_target_properties(mission7_state_estimator_types PROPERTIES LINKER_LANGUAGE CXX ${Eigen_LIBRARIES} ${sophus_LIBRARIES})

add_library(mission7_state_estimator include/mission7_state_estimator/Mission7StateEstimator.cpp include/mission7_state_estimator/Processor.cpp include/mission7_state_estimator/Callback.cpp)
target_link_libraries(mission7_state_estimator ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES} ${sophus_LIBRARIES} mission7_state_estimator_types)

add_executable(state_estimation_node src/state_estimation_node.cpp)
target_link_libraries(state_estimation_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES} ${sophus_LIBRARIES} mission7_state_estimator)
