#pragma once

#include <cstdlib>
#include <cmath>
#include <vector>
#include <tuple>
#include <map>
#include <unordered_map>
#include <random>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "data_generator_options.h"

using namespace std;
using namespace Eigen;

class DataGenerator
{
  public:
    explicit DataGenerator(bool verbose = true);
    explicit DataGenerator(const DataGeneratorOptions &options, bool verbose = false);

    void update();

    double getTime();

    Vector3d getPoint(int i);
    Vector3d getAP(int i);
    vector<Vector3d> getCloud();
    Vector3d getPosition();
    Matrix3d getRotation();
    Vector3d getVelocity();

    Vector3d getAngularVelocity();
    Vector3d getLinearAcceleration();
    Vector3d getAccelerometerBias();
    Vector3d getGyroscopeBias();

    /** packed_id = track_id * num_cam + slot */
    vector<pair<int, Vector3d>> getImage();

    int numCameras() const { return num_cam_; }
    const std::vector<int> &cameraIds() const { return camera_ids_; }
    int cameraId(int slot) const { return camera_ids_.at(static_cast<size_t>(slot)); }
    /** Extrinsic for physical camera \a cam_id (key in ric_/tic_ map). */
    Matrix3d getRic(int cam_id) const { return ric_.at(cam_id); }
    Vector3d getTic(int cam_id) const { return tic_.at(cam_id); }
    void setQuiet(bool quiet) { quiet_ = quiet; }

    int imuPerImage() const { return imu_per_img_; }
    int fovDeg() const { return fov_deg_; }
    int numPoints() const { return num_points_; }

    static int const FREQ = 500;
    static int const MAX_TIME = 40;
    static int const NUMBER_OF_AP = 1;
    static int const MAX_BOX = 10;
    static int const IMU_PER_WIFI = 5;

  /** Union of landmark pool indices visible in the last getImage() (all cameras). */
    vector<Vector3d> output_gr_pts;
    /** Per-slot landmark pool indices visible in the last getImage(). */
    vector<vector<int>> output_gr_ids;
    /** Per-slot landmark indices that received a new track_id this frame. */
    vector<vector<int>> output_new_gr_ids;
    vector<Vector3d> output_Axis[6];

  private:
    void initLandmarks(bool verbose);
    void initAxis();

    int num_cam_;
    std::vector<int> camera_ids_;
    std::unordered_map<int, Eigen::Matrix3d> ric_;
    std::unordered_map<int, Eigen::Vector3d> tic_;
    int fov_deg_;
    int num_points_;
    int imu_per_img_;

    std::vector<int> pts_;
    double t;
    std::vector<std::map<int, int>> before_feature_id_;
    std::vector<std::map<int, int>> current_feature_id_;
    int current_id;

    Vector3d ap[NUMBER_OF_AP];
    Matrix3d acc_cov, gyr_cov;
    Matrix2d pts_cov;
    default_random_engine generator;
    normal_distribution<double> distribution;

    Vector3d Axis[6];
    bool quiet_ = false;
};
