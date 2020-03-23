# 声明要求的 cmake 最低版本
cmake_minimum_required( VERSION 2.8 )

# 声明一个 cmake 工程
project( result )

# 设置编译模式
SET(CMAKE_CXX_COMPILER "g++")
set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS "-std=c++11" )

# 语法：add_executable( 程序名 源代码文件 ）
add_executable( main main.cpp )

# 添加一个库
add_library( slam_shared SHARED slamBase.cpp )
include_directories( "/usr/include/eigen3" )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
find_package( PCL REQUIRED )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

# 将库文件链接到可执行程序上
target_link_libraries( main 
			slam_shared 
			${OpenCV_LIBS} 
			${PCL_LIBRARIES} 
			-lpthread
			)
