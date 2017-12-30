cmake_minimum_required(VERSION 2.8.3)
project(human_mapping)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_geometry
  people_msgs
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV 2.4.8 REQUIRED)

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   people_msgs   sensor_msgs#   std_msgs
 )

catkin_package(

)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

include_directories(${OpenCV_INCLUDE_DIRS})

#file(GLOB_RECURSE human_mapping_SRC "src/human_mapping_node.cpp")
add_executable(human_mapping_node "src/human_mapping_node.cpp" )
target_link_libraries(human_mapping_node
    ${OpenCV_LIBS}
   ${catkin_LIBRARIES}
)
