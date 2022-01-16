cmake_minimum_required(VERSION 2.8.3)
project(dla3_trajectory_generator)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_MAC	OSX_RPATH 0)
add_definitions(-std=c++11)



############
# BINARIES #
############
cs_add_executable(dla3_trajectory_generator
        src/dla3_trajectory_generator.cc
        src/dla3_trajectory_generator_node.cc
)

##########
# EXPORT #
##########
cs_install()
cs_export()
