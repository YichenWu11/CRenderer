cmake_minimum_required(VERSION 3.0.0)
project(TinyRenderer VERSION 0.1.0)

add_executable(TinyRenderer main.cpp geometry.h geometry.cpp 
               model.cpp model.h tgaimage.cpp tgaimage.h
               our_gl.cpp our_gl.h             
)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
