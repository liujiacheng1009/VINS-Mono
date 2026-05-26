#pragma once

#include <map>
#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>

struct CamIntrinsics
{
    double fu = 0.0;
    double fv = 0.0;
    double cu = 0.0;
    double cv = 0.0;
    int width = 0;
    int height = 0;
};

struct CamNode
{
    CamIntrinsics intrinsics;
    bool has_intrinsics = false;
    /** Kalibr T_cam_imu: p_cam = T * p_imu */
    Eigen::Matrix4d T_cam_imu = Eigen::Matrix4d::Identity();
    bool has_T_cam_imu = false;
    /** On cam n: p_cn = T_cn_cnm1 * p_{n-1} */
    Eigen::Matrix4d T_cn_cnm1 = Eigen::Matrix4d::Identity();
    bool has_T_cn_cnm1 = false;
};

/** Sparse Kalibr cam chain keyed by camera index (cam0, cam1, ...). */
struct CamChainFile
{
    std::map<int, CamNode> cameras;
};

std::string resolveCamChainPath(const std::string &config_file, const std::string &cam_chain_file_key);

/** Default imucam chain path: config_dir/cam_chain-imucam.yaml */
std::string resolveCamChainImuPath(const std::string &config_file, const std::string &cam_chain_imu_file_key);

bool loadCamChainYaml(const std::string &path, CamChainFile &out);

/** VINS ric/tic (camera to IMU): p_imu = R * p_cam + t, from Kalibr T_cam_imu. */
bool ricTicFromTCamImu(const Eigen::Matrix4d &T_cam_imu, Eigen::Matrix3d &R_ic, Eigen::Vector3d &t_ic);

/**
 * IMU extrinsic for physical camera \a cam_id.
 * \a T_cam_imu_cam0 is T_cam_imu on cam0 from imucam yaml; cam_id>0 uses stereo T_cn_cnm1 chain.
 */
bool imuExtrinsicForCamera(int cam_id,
                           const Eigen::Matrix4d &T_cam_imu_cam0,
                           const CamChainFile &stereo_chain,
                           Eigen::Matrix3d &R_ic,
                           Eigen::Vector3d &t_ic);

/**
 * Fill per-slot ric/tic and intrinsics for active cameras.
 * \a camera_ids[slot] = physical cam index (e.g. [0], [0,1], [1]).
 */
bool applyCameraSetup(int num_cam,
                      const std::vector<int> &camera_ids,
                      const CamChainFile &imu_cam_chain,
                      const CamChainFile &stereo_chain,
                      double &focal_length_out,
                      double &image_width_out,
                      double &image_height_out,
                      std::vector<Eigen::Matrix3d> &ric_out,
                      std::vector<Eigen::Vector3d> &tic_out);
