cmake_minimum_required(VERSION 3.16)

project(SlamKernel)

    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

    set(CXX_STANDARD 20)
    set(CXX_STANDARD_REQUIRED ON)
    set(CXX_EXTENSIONS OFF)

    add_subdirectory(./slam)

    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

    add_library(${PROJECT_NAME} SHARED
        ./interface/SlamKernel.h
        ./interface/SlamKernel.cpp
    )
    target_include_directories(${PROJECT_NAME}
        PRIVATE ./slam/include
        PRIVATE ./slam/external/DBoW2
        PRIVATE ./slam/external/g2o
        PRIVATE ./slam/external/Sophus
    )
    target_link_libraries(${PROJECT_NAME}
        PRIVATE orbslam3
    )