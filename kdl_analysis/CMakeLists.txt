cmake_minimum_required(VERSION 2.6)

find_package(kdl_codyco)
find_package(kdl_format_io)
find_package(YARP)

include_directories(${orocos_kdl_INCLUDE_DIRS} ${kdl_codyco_INCLUDE_DIRS} ${kdl_format_io_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})

add_executable(3axis_kdl_analysis 3axis_kdl_analysis.cpp)

target_link_libraries(3axis_kdl_analysis ${orocos_kdl_LIBRARIES} ${kdl_codyco_LIBRARIES} ${kdl_format_io_LIBRARIES} ${YARP_LIBRARIES})