cmake_minimum_required(VERSION 3.0.0)
project(Renderer VERSION 0.1.0)

set(HEADERS
        core/geometry.h 
        core/model.h 
        core/tgaimage.h
        core/our_gl.h 
        core/render.h
        core/shaders.h  
        core/sample.h
        )

set(SOURCES
        main.cpp 
        core/geometry.cpp 
        core/model.cpp 
        core/tgaimage.cpp 
        core/our_gl.cpp 
        core/shaders.cpp              
        core/sample.cpp
        )

add_executable(Renderer ${HEADERS} ${SOURCES})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
