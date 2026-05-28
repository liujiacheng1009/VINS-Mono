#include "../src/data_generator.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(vins_sim_data, m)
{
    m.doc() = "DataGenerator bindings for trajectory / IMU / observation visualization";

    py::class_<DataGenerator>(m, "DataGenerator")
        .def(py::init<bool>(), py::arg("verbose") = false)
        .def("update", &DataGenerator::update)
        .def("get_time", &DataGenerator::getTime)
        .def("get_position", &DataGenerator::getPosition)
        .def("get_rotation", &DataGenerator::getRotation)
        .def("get_velocity", &DataGenerator::getVelocity)
        .def("get_angular_velocity", &DataGenerator::getAngularVelocity)
        .def("get_linear_acceleration", &DataGenerator::getLinearAcceleration)
        .def("get_accelerometer_bias", &DataGenerator::getAccelerometerBias)
        .def("get_gyroscope_bias", &DataGenerator::getGyroscopeBias)
        .def("get_cloud", &DataGenerator::getCloud)
        .def("get_image", &DataGenerator::getImage)
        .def("num_cameras", &DataGenerator::numCameras)
        .def("get_ric", &DataGenerator::getRic)
        .def("get_tic", &DataGenerator::getTic)
        .def("set_quiet", &DataGenerator::setQuiet);

    m.attr("FREQ") = DataGenerator::FREQ;
    m.attr("MAX_TIME") = DataGenerator::MAX_TIME;
    m.attr("IMU_PER_IMG") = DataGenerator::IMU_PER_IMG;
    m.attr("NUM_POINTS") = DataGenerator::NUM_POINTS;
}
