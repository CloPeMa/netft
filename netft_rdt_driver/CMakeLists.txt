# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(netft_rdt_driver)
find_package(catkin REQUIRED COMPONENTS roscpp diagnostic_updater diagnostic_msgs geometry_msgs roscpp_serialization)
find_package(Boost REQUIRED COMPONENTS system thread program_options)

include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
include_directories(include)

link_directories(${catkin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})

add_library(${PROJECT_NAME} SHARED src/netft_rdt_driver.cpp)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


add_executable(netft_node src/netft_node.cpp)
target_link_libraries(netft_node ${PROJECT_NAME})
target_link_libraries(netft_node ${Boost_LIBRARIES})
target_link_libraries(netft_node ${catkin_LIBRARIES})

catkin_package(
    DEPENDS roscpp diagnostic_updater diagnostic_msgs geometry_msgs
    CATKIN_DEPENDS roscpp diagnostic_updater diagnostic_msgs geometry_msgs
    LIBRARIES ${PROJECT_NAME}
)
