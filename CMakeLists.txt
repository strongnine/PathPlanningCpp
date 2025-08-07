# requirement of cmake version
# 设置 cmake 要求的最低版本号：
cmake_minimum_required(VERSION 3.5)

# project name
# 这个指令隐式的定义了两个 cmake 变量: 
#     1. <projectname>_BINARY_DIR - 构建路径
#     2. <projectname>_SOURCE_DIR - 工程路径 (CMakeLists.txt 所在路径)
# PROJECT_BINARY_DIR 和 PROJECT_SOURCE_DIR变量
PROJECT(PathPlanningCpp
    LANGUAGES CXX
    VERSION 1.0.0
    DESCRIPTION "Set of path planning algorithms implemented in C++. "
)

find_package(OpenCV REQUIRED)

# set the directory of executable files
# SET 指令的语法: 
#     SET(VAR [VALUE] [CACHE TYPE DOCSTRING [FORCE]])
# 显式的定义变量 VAR 为 [VALUE]
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PathPlanningCpp_SOURCE_DIR}/bin)
# set(CMAKE_CXX_COMPILER "/usr/bin/g++")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# SET(CMAKE_C_COMPILER "/usr/bin/gcc")

add_subdirectory(src)