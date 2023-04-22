cmake_minimum_required(VERSION 3.4.1)
project(${TARGET_NAME})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wl,--allow-shlib-undefined")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wl,--allow-shlib-undefined")

# install target and libraries
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install/${TARGET_NAME})

set(CMAKE_SKIP_INSTALL_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# rknn api
set(RKNN_API_PATH ${CMAKE_SOURCE_DIR}/runtime/librknn_api)

if (CMAKE_C_COMPILER MATCHES "aarch64")
  set(LIB_ARCH aarch64)
else()
  set(LIB_ARCH armhf)
endif()
set(RKNN_RT_LIB ${RKNN_API_PATH}/${LIB_ARCH}/librknnrt.so)
include_directories(${RKNN_API_PATH}/include)

# Catkin
find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  std_msgs
  cv_bridge
  sensor_msgs
  geometry_msgs
  image_transport
  message_generation
)

include_directories(${catkin_INCLUDE_DIRS})

# Find librealsense2 installed package
find_package(realsense2 REQUIRED)

# opencv
find_package(OpenCV REQUIRED)

# demo
include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(${TARGET_NAME}
        src/main.cpp
        src/buffnet.cpp
        src/postprocess.cc
        src/buff_realsense.cpp
)

target_link_libraries(${TARGET_NAME}
  ${RKNN_RT_LIB}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${realsense2_LIBRARY}
)

# install target and libraries
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install/${TARGET_NAME})
install(TARGETS ${TARGET_NAME} DESTINATION ./)
install(PROGRAMS ${RKNN_RT_LIB} DESTINATION lib)
