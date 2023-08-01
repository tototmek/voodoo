cmake_minimum_required(VERSION 3.0.2)
project(voodoo)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(voodoo_node src/voodoo_node.cpp)
target_link_libraries(voodoo_node ${catkin_LIBRARIES})


