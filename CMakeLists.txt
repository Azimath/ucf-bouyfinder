cmake_minimum_required(VERSION 2.8.3)
project(bouyfinder)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED sensor_msgs cv_bridge roscpp std_msgs image_transport)
find_package(OpenCV 2.4.8 EXACT REQUIRED)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
add_executable(image_converter src/image_converter.cpp)
add_executable(bouyfinder src/bouyfinder.cpp)
target_link_libraries(image_converter ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(bouyfinder ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
