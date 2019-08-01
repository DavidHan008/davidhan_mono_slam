cmake_minimum_required(VERSION 2.8.3)
project(davidhan_mono_slam)
set(CMAKE_CXX_FLAGS "-O2 -msse4 -DEIGEN_NO_DEBUG -DNDEBUG")
find_package(catkin REQUIRED COMPONENTS cv_bridge tf sensor_msgs cv_bridge roscpp std_msgs image_transport)
find_package(OpenCV REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS})
catkin_package(
)
add_executable(davidhan-mono-slam
        src/david_RosVSLAMRansac.cpp
        src/vslamRansac.cpp
        )


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(mono_slam_node mono_slam_generate_messages_cpp)

## Specify libraries to link a library or executable target against

target_link_libraries(davidhan-mono-slam  ${catkin_LIBRARIES} ${OpenCV_LIBS}  config++)