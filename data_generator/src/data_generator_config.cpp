#include "data_generator_config.h"

#include "../../vins_estimator/src/parameters.h"

#include <opencv2/core.hpp>

#include <stdexcept>

DataGeneratorOptions dataGeneratorOptionsFromVinsParameters()
{
    const VinsParameters &vp = vinsParameters();
    const int n = vp.numOfCam();
    if (n < 1)
        throw std::runtime_error("num_of_cam must be >= 1 (call readParameters first)");

    if (static_cast<int>(vp.ric().size()) != n || static_cast<int>(vp.tic().size()) != n)
        throw std::runtime_error("VinsParameters extrinsic count does not match num_of_cam");

    DataGeneratorOptions o;
    o.num_cam = n;
    o.camera_ids = vp.cameraIds();
    o.ric.clear();
    o.tic.clear();

    for (int slot = 0; slot < n; ++slot)
    {
        const int cam_id = vp.cameraId(slot);
        o.ric[cam_id] = vp.ricForCamera(cam_id);
        o.tic[cam_id] = vp.ticForCamera(cam_id);
    }

    return o;
}

void dataGeneratorLoadConfig(const std::string &config_file, DataGeneratorOptions &options)
{
    cv::FileStorage fs(config_file, cv::FileStorage::READ);
    if (!fs.isOpened())
        return;
    const cv::FileNode root = fs.root();
    if (!root["fov_deg"].empty())
        options.fov_deg = static_cast<int>(root["fov_deg"]);
    if (!root["num_points"].empty())
        options.num_points = static_cast<int>(root["num_points"]);
    if (!root["imu_per_img"].empty())
        options.imu_per_img = static_cast<int>(root["imu_per_img"]);
}

void dataGeneratorLoadDefaultConfig(DataGeneratorOptions &options)
{
    static const char *const candidates[] = {
        defaultDataGeneratorConfigPath(),
        "../data_generator/config/data_generator.yaml",
    };
    for (const char *path : candidates)
    {
        cv::FileStorage fs(path, cv::FileStorage::READ);
        if (fs.isOpened())
        {
            dataGeneratorLoadConfig(path, options);
            return;
        }
    }
}
