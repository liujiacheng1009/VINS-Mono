cmake_minimum_required(VERSION 3.10)
project(vins_mono_standalone)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

add_executable(vins_simulation_standalone
    simulation_standalone.cpp
    ../data_generator/src/data_generator.cpp
    ../vins_estimator/src/parameters.cpp
    ../vins_estimator/src/estimator.cpp
    ../vins_estimator/src/feature_manager.cpp
    ../vins_estimator/src/factor/pose_local_parameterization.cpp
    ../vins_estimator/src/factor/projection_factor.cpp
    ../vins_estimator/src/factor/projection_td_factor.cpp
    ../vins_estimator/src/factor/marginalization_factor.cpp
    ../vins_estimator/src/utility/utility.cpp
    ../vins_estimator/src/initial/solve_5pts.cpp
    ../vins_estimator/src/initial/initial_aligment.cpp
    ../vins_estimator/src/initial/initial_sfm.cpp
    ../vins_estimator/src/initial/initial_ex_rotation.cpp
)

target_include_directories(vins_simulation_standalone PRIVATE
    ros_compat
    ../vins_estimator/src
    ../data_generator/src
    ${EIGEN3_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(vins_simulation_standalone
    ${OpenCV_LIBS}
    ${CERES_LIBRARIES}
    pthread
)
