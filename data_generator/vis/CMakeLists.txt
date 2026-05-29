# DataGenerator visualization: JSON dump + optional pybind11 module.

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

set(DG_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../src)
set(DG_CORE
    ${DG_SRC_DIR}/data_generator.cpp
    ${DG_SRC_DIR}/data_generator_options.cpp
    ${DG_SRC_DIR}/data_generator_config.cpp
)
set(VINS_PARAM_SRC
    ../../vins_estimator/src/parameters.cpp
    ../../vins_estimator/src/cam_chain.cpp
    ../../vins_estimator/src/utility/utility.cpp
)

add_executable(sim_generator_dump sim_generator_dump.cpp ${DG_CORE} ${VINS_PARAM_SRC})
set_target_properties(sim_generator_dump PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
target_include_directories(sim_generator_dump PRIVATE
    ${DG_SRC_DIR}
    ../../vins_estimator/src
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(sim_generator_dump pthread ${OpenCV_LIBS} log_value::log_value)

if(BUILD_SIM_PYTHON)
    find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

    find_package(pybind11 CONFIG QUIET)
    if(NOT pybind11_FOUND)
        execute_process(
            COMMAND "${Python3_EXECUTABLE}" -m pybind11 --cmakedir
            OUTPUT_VARIABLE _pybind11_cmakedir
            OUTPUT_STRIP_TRAILING_WHITESPACE
            RESULT_VARIABLE _pybind11_rc
        )
        if(_pybind11_rc EQUAL 0 AND _pybind11_cmakedir)
            list(APPEND CMAKE_PREFIX_PATH "${_pybind11_cmakedir}")
            find_package(pybind11 CONFIG REQUIRED)
        else()
            message(FATAL_ERROR
                "pybind11 not found. Install: sudo apt install pybind11-dev OR pip3 install pybind11")
        endif()
    endif()

    pybind11_add_module(vins_sim_data bindings.cpp ${DG_CORE} ${VINS_PARAM_SRC})
    target_include_directories(vins_sim_data PRIVATE
        ${DG_SRC_DIR}
        ../../vins_estimator/src
        ${EIGEN3_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(vins_sim_data PRIVATE Eigen3::Eigen ${OpenCV_LIBS} log_value::log_value)
endif()
