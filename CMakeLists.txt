set (CMAKE_CXX_STANDARD 11)
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pose)

find_package(OpenCV REQUIRED)
include_directories($(OpenCV_DIR))

find_package (VTK 5.10.1 REQUIRED)
find_package (PCL 1.7.0 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Boost REQUIRED)

find_package( CUDA REQUIRED )
include_directories(/usr/local/cuda/include)
set(
    CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS};
    -O3 -lineinfo
    -gencode=arch=compute_61,code=sm_61
    )

add_executable(pose pose.cpp)
target_link_libraries(pose ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES})
