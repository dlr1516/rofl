# https://stackoverflow.com/questions/54702582/how-to-configure-project-with-components-in-cmake
cmake_minimum_required(VERSION 3.1)
project(rofl)
set(VERSION_NUM 1.0.0)
#add_definitions(-std=c++0x)

set(CMAKE_BUILD_TYPE Release)


find_package(Eigen3 REQUIRED)
find_package(PCL)

# Adds the subdirectories of the components of the library
add_subdirectory(common)
#add_subdirectory(geometry)

if(PCL_FOUND)
  include_directories(${PCL_INCLUDE_DIRS})

  add_subdirectory(pcl)
endif()

# Installs instruction for CMake configuration scripts 

