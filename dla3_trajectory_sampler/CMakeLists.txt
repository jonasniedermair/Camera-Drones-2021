cmake_minimum_required(VERSION 2.8.3)
project(dla3_trajectory_sampler)

find_package(catkin REQUIRED COMPONENTS
        eigen_catkin REQUIRED
        mav_msgs REQUIRED
        mav_trajectory_generation REQUIRED
        mav_trajectory_generation_ros REQUIRED
        mav_visualization REQUIRED
        mav_planning_msgs REQUIRED
        eigen_checks REQUIRED
        roslib REQUIRED
        roscpp REQUIRED
        tf REQUIRED
        )

include_directories(
        ${catkin_INCLUDE_DIRS}
)

include_directories(include)

add_definitions(-std=c++11)

############
# BINARIES #
############
add_executable(dla3_trajectory_sampler
        src/dla3_trajectory_sampler.cc
        src/dla3_trajectory_sampler_node.cc
        )

target_link_libraries(dla3_trajectory_sampler ${catkin_LIBRARIES})
