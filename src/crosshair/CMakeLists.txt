cmake_minimum_required(VERSION 3.5)
project(crosshair)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
add_compile_options(-g)

## is used, also find other catkin packages
if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()


set(_opencv_version 4)
find_package(OpenCV 4 QUIET COMPONENTS imgproc highgui)
if(NOT OpenCV_FOUND)
  set(_opencv_version 3)
  find_package(OpenCV 3 REQUIRED COMPONENTS imgproc highgui)
endif()

find_package(catkin REQUIRED COMPONENTS
	camera_info_manager
	depthai_ros_msgs
	message_filters
	depthai_bridge
	sensor_msgs
	vision_msgs
	cv_bridge
	std_msgs
	nodelet
	roscpp
)

find_package(depthai CONFIG REQUIRED)

catkin_package(
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS 
		camera_info_manager
		depthai_ros_msgs
		message_filters
		depthai_bridge
		sensor_msgs
		vision_msgs
		cv_bridge
		std_msgs
		nodelet
		roscpp
	DEPENDS
		OpenCV
)


include_directories(${catkin_INCLUDE_DIRS})


add_executable(depthai-demo src/depthai-depth.cpp)
add_dependencies(depthai-demo
        ${catkin_EXPORTED_TARGETS} 
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
      )




#############
## Install ##
#############


catkin_install_python(PROGRAMS
  scripts/luxnet.py 
  scripts/trainer.py 
  scripts/xml2txt.py 
  scripts/image_generator.py 
  scripts/interactive_thresholder.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
