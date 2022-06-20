cmake_minimum_required(VERSION 3.10)

project(camera_models)


set(CMAKE_CONFIGURATION_TYPES Debug Release CACHE TYPE INTERNAL FORCE -Who-dev)
project(camera_models LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 14)

add_compile_options(-Wall)

set(LIBRARY_DIR /home/ibrahimov/dev/CPPLibs CACHE PATH "Path to libs folder")

find_package(Eigen3 REQUIRED)

include_directories(models)
include_directories(util)

set(HEADER_FILES
    models/camera_models.h
    util/Eigen.h
)

set(SOURCE_FILES 
    models/camera_models.cpp
)

add_executable(main main.cpp ${HEADER_FILES} ${SOURCE_FILES})

include_directories(${EIGEN3_INCLUDE_DIR})
target_link_libraries(main Eigen3::Eigen)
