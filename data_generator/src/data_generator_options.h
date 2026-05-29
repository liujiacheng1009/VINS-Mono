#pragma once

#include <unordered_map>
#include <vector>
#include <eigen3/Eigen/Dense>

/** Runtime configuration for DataGenerator (multi-camera). */
struct DataGeneratorOptions
{
    int num_cam = 1;
    /** Active camera slots; camera_ids[slot] = physical camera index. */
    std::vector<int> camera_ids{0};
    /** IMU–camera extrinsics keyed by physical camera id. */
    std::unordered_map<int, Eigen::Matrix3d> ric;
    std::unordered_map<int, Eigen::Vector3d> tic;

    int fov_deg = 90;
    int num_points = 500;
    int imu_per_img = 50;

    /** Hard-coded single-camera extrinsics (legacy, cam0 only). */
    static DataGeneratorOptions legacyDefaults();
};
