cmake_minimum_required(VERSION 3.10)
project(vins_mono_standalone)

enable_testing()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

add_subdirectory(../third_party/log_value ${CMAKE_BINARY_DIR}/third_party/log_value)

add_executable(vins_simulation_standalone
    simulation_standalone.cpp
    ../data_generator/src/data_generator.cpp
    ../vins_estimator/src/parameters.cpp
    ../vins_estimator/src/estimator.cpp
    ../vins_estimator/src/state_manager.cpp
    ../vins_estimator/src/feature_manager.cpp
    ../vins_estimator/src/factor/imu_factor.cpp
    ../vins_estimator/src/factor/pose_local_parameterization.cpp
    ../vins_estimator/src/factor/projection_factor.cpp
    ../vins_estimator/src/factor/marginalization_factor.cpp
    ../vins_estimator/src/utility/utility.cpp
)

target_include_directories(vins_simulation_standalone PRIVATE
    ../vins_estimator/src
    ../data_generator/src
    ${EIGEN3_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(vins_simulation_standalone
    ${OpenCV_LIBS}
    ${CERES_LIBRARIES}
    log_value::log_value
    pthread
)

add_executable(state_manager_smoke_test
    state_manager_smoke_test.cpp
    ../vins_estimator/src/state_manager.cpp
    ../vins_estimator/src/parameters.cpp
    ../vins_estimator/src/utility/utility.cpp
    ../vins_estimator/src/factor/imu_factor.cpp
    ../vins_estimator/src/factor/marginalization_factor.cpp
)

target_include_directories(state_manager_smoke_test PRIVATE
    ../vins_estimator/src
    ${EIGEN3_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(state_manager_smoke_test pthread ${OpenCV_LIBS} ${CERES_LIBRARIES})

add_test(NAME state_manager_smoke COMMAND state_manager_smoke_test)
add_test(
    NAME simulation_metrics_regression
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/check_simulation_metrics.sh
            $<TARGET_FILE:vins_simulation_standalone>
)
set_tests_properties(simulation_metrics_regression PROPERTIES TIMEOUT 120)
