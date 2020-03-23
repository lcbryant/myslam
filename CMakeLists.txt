cmake_minimum_required(VERSION 3.0)

project(myslam VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c++11 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native")

find_package(mynteye REQUIRED)
message(STATUS "Found mynteye: ${mynteye_VERSION}")

# When SDK build with OpenCV, we can add WITH_OPENCV macro to enable some
# features depending on OpenCV, such as ToMat().
if(mynteye_WITH_OPENCV)
  add_definitions(-DWITH_OPENCV)
endif()

find_package(OpenCV REQUIRED)
message(STATUS "Found OpenCV: ${OpenCV_VERSION}")

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

## mynteye_demo

add_executable(collect_data collect_data.cpp)
target_link_libraries(collect_data mynteye ${OpenCV_LIBS})