#include "../../data_generator/src/data_generator.h"
#include "../../data_generator/src/data_generator_config.h"
#include "../../data_generator/src/data_generator_options.h"
#include "../../vins_estimator/src/parameters.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

static DataGeneratorOptions loadOptionsWithConfig(const std::string &sim_config_file,
                                                  const std::string &dg_config_file)
{
    readParameters(sim_config_file);
    DataGeneratorOptions o = dataGeneratorOptionsFromVinsParameters();
    if (dg_config_file.empty())
        dataGeneratorLoadDefaultConfig(o);
    else
        dataGeneratorLoadConfig(dg_config_file, o);
    return o;
}

PYBIND11_MODULE(vins_sim_data, m)
{
    m.doc() = "DataGenerator bindings for trajectory / IMU / observation visualization";

    py::class_<DataGeneratorOptions>(m, "DataGeneratorOptions")
        .def(py::init<>())
        .def_readwrite("num_cam", &DataGeneratorOptions::num_cam)
        .def_readwrite("camera_ids", &DataGeneratorOptions::camera_ids)
        .def_readwrite("ric", &DataGeneratorOptions::ric)
        .def_readwrite("tic", &DataGeneratorOptions::tic)
        .def_readwrite("fov_deg", &DataGeneratorOptions::fov_deg)
        .def_readwrite("num_points", &DataGeneratorOptions::num_points)
        .def_readwrite("imu_per_img", &DataGeneratorOptions::imu_per_img);

    m.def("options_from_vins", &dataGeneratorOptionsFromVinsParameters,
          "Build options from vinsParameters(); readParameters() must already have been called.");
    m.def("load_config", &dataGeneratorLoadConfig, py::arg("config_file"), py::arg("options"));
    m.def("read_parameters", &readParameters, py::arg("config_file"),
          "Load simulation_config + cam_chain (call once before options_from_vins).");
    m.def("load_options", &loadOptionsWithConfig, py::arg("sim_config_file"),
          py::arg("dg_config_file") = std::string(),
          "read_parameters + options_from_vins + load default data_generator config.");
    m.def("legacy_options", &DataGeneratorOptions::legacyDefaults);
    m.attr("DEFAULT_DG_CONFIG") = defaultDataGeneratorConfigPath();

    py::class_<DataGenerator>(m, "DataGenerator")
        .def(py::init<bool>(), py::arg("verbose") = false)
        .def(py::init<const DataGeneratorOptions &, bool>(), py::arg("options"), py::arg("verbose") = false)
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
        .def("get_observed_points", [](const DataGenerator &self) { return self.output_gr_pts; })
        .def("num_cameras", &DataGenerator::numCameras)
        .def("camera_id", &DataGenerator::cameraId)
        .def("camera_ids", &DataGenerator::cameraIds)
        .def("get_ric", &DataGenerator::getRic, py::arg("cam_id"))
        .def("get_tic", &DataGenerator::getTic, py::arg("cam_id"))
        .def("imu_per_image", &DataGenerator::imuPerImage)
        .def("fov_deg", &DataGenerator::fovDeg)
        .def("set_quiet", &DataGenerator::setQuiet);

    m.attr("FREQ") = DataGenerator::FREQ;
    m.attr("MAX_TIME") = DataGenerator::MAX_TIME;
}
