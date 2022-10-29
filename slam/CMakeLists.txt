cmake_minimum_required(VERSION 3.16)

project(orbslam3)

    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

    set(CXX_STANDARD 20)
    set(CXX_STANDARD_REQUIRED ON)
    set(CXX_EXTENSIONS OFF)

    add_subdirectory(./external/DBoW2)
    add_subdirectory(./external/g2o)
    add_subdirectory(./external/Sophus)

    find_package(Eigen3 3 REQUIRED)
    find_package(OpenCV 4 REQUIRED)

    # orbslam3
    file(GLOB slam_camera_files  CONFIGURE_DEPENDS ./include/camera_models/*.h ./src/camera_models/*.cpp)
    file(GLOB slam_core_files    CONFIGURE_DEPENDS ./include/core/*.h ./src/core/*.cpp)
    file(GLOB slam_feature_files CONFIGURE_DEPENDS ./include/feature/*.h ./src/feature/*.cpp)
    file(GLOB slam_frame_files   CONFIGURE_DEPENDS ./include/frame/*.h ./src/frame/*.cpp)
    file(GLOB slam_map_files     CONFIGURE_DEPENDS ./include/map/*.h ./src/map/*.cpp)
    file(GLOB slam_solver_files  CONFIGURE_DEPENDS ./include/solver/*.h ./src/solver/*.cpp)
    file(GLOB slam_threads_files CONFIGURE_DEPENDS ./include/threads/*.h ./src/threads/*.cpp)
    file(GLOB slam_utils_files   CONFIGURE_DEPENDS ./include/utils/*.h ./src/utils/*.cpp)

    add_library(${PROJECT_NAME} SHARED
        ${slam_camera_files}
        ${slam_core_files}
        ${slam_feature_files}
        ${slam_frame_files}
        ${slam_map_files}
        ${slam_solver_files}
        ${slam_threads_files}
        ${slam_utils_files}
    )
    target_include_directories(${PROJECT_NAME}
        PRIVATE ./include
        PUBLIC  ./external/DBoW2
        PUBLIC  ./external/g2o
        PUBLIC  ./external/Sophus
        PUBLIC  ${OpenCV_INCLUDE_DIR}
    )
    target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        g2o
        DBoW2
        Eigen3::Eigen
    )