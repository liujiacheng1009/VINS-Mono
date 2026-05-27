cmake_minimum_required(VERSION 3.14)
project(vins_multi_tests LANGUAGES CXX)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GTest REQUIRED)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../third_party/log_value
                 ${CMAKE_BINARY_DIR}/third_party/log_value)

include(GoogleTest)
enable_testing()

set(VINS_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../vins_estimator/src)

add_executable(imu_factor_test
    imu_factor_test.cpp
    ${VINS_SRC_DIR}/factor/imu_factor.cpp
    ${VINS_SRC_DIR}/parameters.cpp
    ${VINS_SRC_DIR}/cam_chain.cpp
    ${VINS_SRC_DIR}/utility/utility.cpp
)

target_include_directories(imu_factor_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${VINS_SRC_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(imu_factor_test PRIVATE
    GTest::gtest
    GTest::gtest_main
    ${CERES_LIBRARIES}
    ${OpenCV_LIBS}
    log_value::log_value
    pthread
)

gtest_discover_tests(imu_factor_test)

add_executable(projection_factor_test
    projection_factor_test.cpp
    ${VINS_SRC_DIR}/factor/projection_factor.cpp
    ${VINS_SRC_DIR}/parameters.cpp
    ${VINS_SRC_DIR}/cam_chain.cpp
    ${VINS_SRC_DIR}/utility/utility.cpp
)

target_include_directories(projection_factor_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${VINS_SRC_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(projection_factor_test PRIVATE
    GTest::gtest
    GTest::gtest_main
    ${CERES_LIBRARIES}
    ${OpenCV_LIBS}
    log_value::log_value
    pthread
)

gtest_discover_tests(projection_factor_test)

add_executable(marginalization_factor_test
    marginalization_factor_test.cpp
    ${VINS_SRC_DIR}/factor/marginalization_factor.cpp
    ${VINS_SRC_DIR}/utility/utility.cpp
)

target_include_directories(marginalization_factor_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${VINS_SRC_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(marginalization_factor_test PRIVATE
    GTest::gtest
    GTest::gtest_main
    ${CERES_LIBRARIES}
    ${OpenCV_LIBS}
    log_value::log_value
    pthread
)

gtest_discover_tests(marginalization_factor_test)

add_executable(cam_chain_extrinsic_test
    cam_chain_extrinsic_test.cpp
    ${VINS_SRC_DIR}/cam_chain.cpp
)

target_include_directories(cam_chain_extrinsic_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${VINS_SRC_DIR}
    ${EIGEN3_INCLUDE_DIRS}
)

target_compile_definitions(cam_chain_extrinsic_test PRIVATE
    VINS_TEST_SIM_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../config/simulation"
)

target_link_libraries(cam_chain_extrinsic_test PRIVATE
    GTest::gtest
    GTest::gtest_main
    log_value::log_value
    pthread
)

gtest_discover_tests(cam_chain_extrinsic_test)
