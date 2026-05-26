#pragma once

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

/** Parsed Kalibr-style cam_chain (cam0, cam1, ...). */
struct CamChainData
{
    std::vector<CamIntrinsics> cameras;
    /** T_cn_cnm1[i]: homogeneous transform from camera i to camera i+1. */
    std::vector<Eigen::Matrix4d> T_cn_cnm1;
};

/** Resolve cam_chain path: optional cam_chain_file relative to config directory, else config_dir/cam_chain.yaml. */
std::string resolveCamChainPath(const std::string &config_file, const std::string &cam_chain_file_key);

/** Parse cam_chain.yaml; returns false if file missing or unreadable. */
bool loadCamChainFile(const std::string &path, CamChainData &out);

/**
 * Apply cam0 intrinsics and (when num_cam > 1) stereo extrinsics from chain.
 * cam0 IMU extrinsic (R_ic, t_ic) always comes from simulation_config, not the chain file.
 */
bool applyCamChain(int num_cam,
                   const CamChainData &chain,
                   const Eigen::Matrix3d &R_ic0,
                   const Eigen::Vector3d &t_ic0,
                   double &focal_length_out,
                   double &image_width_out,
                   double &image_height_out,
                   std::vector<Eigen::Matrix3d> &ric_out,
                   std::vector<Eigen::Vector3d> &tic_out);
