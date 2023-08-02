cmake_minimum_required(VERSION 3.0.2)
project(voodoo)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)

set(CMAKE_CXX_STANDARD 17)
set(COMPILE_FLAGS ${COMPILE_FLAGS} -Wall -Wextra -Wpedantic)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include/
)

add_executable(voodoo_node src/voodoo_node.cpp src/voodoo.cpp src/IO.cpp)
target_link_libraries(voodoo_node ${catkin_LIBRARIES})


