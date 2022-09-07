cmake_minimum_required(VERSION 3.0.0)
project(TinyRenderer VERSION 0.1.0)

add_executable(TinyRenderer 
               main.cpp 
               core/geometry.h 
               core/geometry.cpp 
               core/model.cpp 
               core/model.h 
               core/tgaimage.cpp 
               core/tgaimage.h
               core/our_gl.cpp 
               core/our_gl.h 
               core/render.h 
               core/shaders.cpp              
               core/shaders.h        
)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
