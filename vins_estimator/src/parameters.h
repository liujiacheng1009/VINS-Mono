#pragma once

#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "utility/utility.h"
#include "utility/logging.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>

//#define UNIT_SPHERE_ERROR

/** Runtime configuration loaded from YAML; access only via getters/setters. */
class VinsParameters
{
  public:
    static VinsParameters &instance();

    void loadFromConfig(const std::string &config_file);

    int windowSize() const { return window_size_; }
    void setWindowSize(int v) { window_size_ = v; }

    int numOfCam() const { return num_of_cam_; }
    void setNumOfCam(int v) { num_of_cam_ = v; }

    int maxFeatureCount() const { return max_feature_count_; }
    void setMaxFeatureCount(int v) { max_feature_count_ = v; }

    double focalLength() const { return focal_length_; }
    void setFocalLength(double v) { focal_length_ = v; }

    double initDepth() const { return init_depth_; }
    void setInitDepth(double v) { init_depth_ = v; }

    double minParallax() const { return min_parallax_; }
    void setMinParallax(double v) { min_parallax_ = v; }

    double accNoise() const { return acc_n_; }
    void setAccNoise(double v) { acc_n_ = v; }

    double accRandomWalk() const { return acc_w_; }
    void setAccRandomWalk(double v) { acc_w_ = v; }

    double gyrNoise() const { return gyr_n_; }
    void setGyrNoise(double v) { gyr_n_ = v; }

    double gyrRandomWalk() const { return gyr_w_; }
    void setGyrRandomWalk(double v) { gyr_w_ = v; }

    const std::vector<Eigen::Matrix3d> &ric() const { return ric_; }
    std::vector<Eigen::Matrix3d> &ric() { return ric_; }
    void setRic(std::vector<Eigen::Matrix3d> v) { ric_ = std::move(v); }
    void clearExtrinsics() { ric_.clear(); tic_.clear(); }
    void addExtrinsic(const Eigen::Matrix3d &R, const Eigen::Vector3d &T)
    {
        ric_.push_back(R);
        tic_.push_back(T);
    }

    const std::vector<Eigen::Vector3d> &tic() const { return tic_; }
    std::vector<Eigen::Vector3d> &tic() { return tic_; }
    void setTic(std::vector<Eigen::Vector3d> v) { tic_ = std::move(v); }

    const Eigen::Vector3d &gravity() const { return gravity_; }
    Eigen::Vector3d &gravity() { return gravity_; }
    void setGravity(const Eigen::Vector3d &g) { gravity_ = g; }
    void setGravityNorm(double g_norm) { gravity_.z() = g_norm; }

    double solverTime() const { return solver_time_; }
    void setSolverTime(double v) { solver_time_ = v; }

    int numIterations() const { return num_iterations_; }
    void setNumIterations(int v) { num_iterations_ = v; }

    int estimateExtrinsic() const { return estimate_extrinsic_; }
    void setEstimateExtrinsic(int v) { estimate_extrinsic_ = v; }

    int estimateTd() const { return estimate_td_; }
    void setEstimateTd(int v) { estimate_td_ = v; }

    int rollingShutter() const { return rolling_shutter_; }
    void setRollingShutter(int v) { rolling_shutter_ = v; }

    const std::string &vinsResultPath() const { return vins_result_path_; }
    void setVinsResultPath(std::string v) { vins_result_path_ = std::move(v); }

    double imageRow() const { return row_; }
    void setImageRow(double v) { row_ = v; }

    double imageCol() const { return col_; }
    void setImageCol(double v) { col_ = v; }

    double td() const { return td_; }
    void setTd(double v) { td_ = v; }

    double rollingShutterTr() const { return tr_; }
    void setRollingShutterTr(double v) { tr_ = v; }

  private:
    int window_size_ = 10;
    int num_of_cam_ = 1;
    int max_feature_count_ = 1000;
    double focal_length_ = 460.0;
    double init_depth_ = 5.0;
    double min_parallax_ = 0.0;
    double acc_n_ = 0.0;
    double acc_w_ = 0.0;
    double gyr_n_ = 0.0;
    double gyr_w_ = 0.0;
    std::vector<Eigen::Matrix3d> ric_;
    std::vector<Eigen::Vector3d> tic_;
    Eigen::Vector3d gravity_{0.0, 0.0, 9.8};
    double solver_time_ = 0.0;
    int num_iterations_ = 0;
    int estimate_extrinsic_ = 0;
    int estimate_td_ = 0;
    int rolling_shutter_ = 0;
    std::string vins_result_path_;
    double row_ = 0.0;
    double col_ = 0.0;
    double td_ = 0.0;
    double tr_ = 0.0;
};

inline VinsParameters &vinsParameters() { return VinsParameters::instance(); }

/** Shorthand accessors for layout parameters (see VinsParameters). */
inline int windowSize() { return vinsParameters().windowSize(); }
inline int numOfCam() { return vinsParameters().numOfCam(); }
inline int maxFeatureCount() { return vinsParameters().maxFeatureCount(); }
inline double focalLength() { return vinsParameters().focalLength(); }

void readParameters(const std::string &config_file);

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};
