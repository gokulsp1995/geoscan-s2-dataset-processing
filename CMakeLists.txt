cmake_minimum_required(VERSION 3.10)
project(pcd2octomap_converter)

set(CMAKE_CXX_STANDARD 14)

# Find Required Packages
find_package(PCL REQUIRED COMPONENTS io common)
find_package(octomap REQUIRED)

# Include Directories
include_directories(${PCL_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${OCTOMAP_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Add Executable
add_executable(pcd2octomap pcd2octomap.cpp)
target_link_libraries(pcd2octomap ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})
