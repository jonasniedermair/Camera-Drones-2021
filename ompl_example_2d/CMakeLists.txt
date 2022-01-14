cmake_minimum_required(VERSION 3.0.2)
project(planner_node)

set(CMAKE_VERBOSE_MAKEFILE ON)

# python2.7 for some reason uses the register keyword. Since C++17 removed this keyword, we have to se this flag
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-register -fopenmp")
set(CMAKE_CXX_STANDARD 11)
find_package(catkin REQUIRED COMPONENTS
		roscpp
		message_filters
		sensor_msgs
		std_msgs
		tf
		tf_conversions
		eigen_conversions
		std_srvs
		message_generation
		)

link_directories(${OMPL_LIBRARY_DIRS})

find_package(ompl REQUIRED
		     Boost 1.67.0 REQUIRED COMPONENTS program_options)

catkin_package(
		#  INCLUDE_DIRS include
		#  LIBRARIES planner_lib
		#  CATKIN_DEPENDS roscpp std_msgs
		#  DEPENDS system_lib
)

include_directories(
		include
		${catkin_INCLUDE_DIRS}
		${OMPL_INCLUDE_DIRS}
		${Boost_INCLUDE_DIR}
)

add_executable(planner_node src/ompl_example_2d_node.cpp)

target_link_libraries(planner_node ${catkin_LIBRARIES} ${OMPL_LIBRARIES} Boost::program_options)
