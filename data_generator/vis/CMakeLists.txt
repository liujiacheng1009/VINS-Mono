# DataGenerator visualization: JSON dump + optional pybind11 module.
# Included from standalone/CMakeLists.txt (BUILD_SIM_PYTHON for pybind).

find_package(Eigen3 REQUIRED)

set(DG_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../src)
set(DG_CORE ${DG_SRC_DIR}/data_generator.cpp)

add_executable(sim_generator_dump sim_generator_dump.cpp ${DG_CORE})
target_include_directories(sim_generator_dump PRIVATE ${DG_SRC_DIR} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(sim_generator_dump pthread)

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
                "未找到 pybind11。请安装其一：\n"
                "  sudo apt install pybind11-dev\n"
                "  pip3 install pybind11")
        endif()
    endif()

    pybind11_add_module(vins_sim_data bindings.cpp ${DG_CORE})
    target_include_directories(vins_sim_data PRIVATE ${DG_SRC_DIR} ${EIGEN3_INCLUDE_DIRS})
    target_link_libraries(vins_sim_data PRIVATE Eigen3::Eigen)
endif()
