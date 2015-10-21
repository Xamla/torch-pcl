cmake_minimum_required(VERSION 2.8.11)

project(torch-pcl)
find_package(PCL 1.3 REQUIRED COMPONENTS common io)
find_package(Torch REQUIRED)

include_directories(${Torch_INSTALL_INCLUDE} ${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
# Linker options
set(CMAKE_LDFLAGS "${CMAKE_LDFLAGS}  ")

# Library path
set(CMAKE_LDFLAGS "${CMAKE_LDFLAGS} -L\".\" ")

add_library(torch-pcl SHARED "${SOURCE_DIR}/io.cpp")
target_link_libraries(torch-pcl TH ${PCL_LIBRARIES})
