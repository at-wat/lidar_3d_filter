cmake_minimum_required(VERSION 2.8.3)
project(hokuyo3d_compensate)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)
find_package(Boost 1.53 REQUIRED system serialization)

## Declare a catkin package
catkin_package(
    DEPENDS roscpp sensor_msgs
)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
	message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(hokuyo3d_compensate src/hokuyo3d_compensate.cpp)
target_link_libraries(hokuyo3d_compensate ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(reduce_veil src/reduce_veil.cpp)
target_link_libraries(reduce_veil ${catkin_LIBRARIES} ${Boost_LIBRARIES})
