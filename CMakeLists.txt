cmake_minimum_required(VERSION 3.0.0)
project(Renderer VERSION 0.1.0)

# DEBUG
# set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
# set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

add_subdirectory(core)

add_executable(Renderer main.cpp)

# 添加链接库
target_link_libraries(Renderer CORE_SRCS)
