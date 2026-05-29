#include "parameters.h"

#include "cam_chain.h"

#include <log_value/log_macros.h>
#include <opencv2/core/eigen.hpp>

#include <fstream>
#include <stdexcept>

namespace {

void setMinParallaxFromPixels(VinsParameters &p, double keyframe_parallax_px, double focal)
{
    if (focal > 0.0)
        p.setMinParallax(keyframe_parallax_px / focal);
}

bool applyCamerasToParams(VinsParameters &p,
                          const CamChainFile &imu_chain,
                          const CamChainFile &stereo_chain,
                          int estimate_extrinsic,
                          double keyframe_parallax_px)
{
    double focal = p.focalLength();
    double width = p.imageCol();
    double height = p.imageRow();
    std::vector<Eigen::Matrix3d> ric_chain;
    std::vector<Eigen::Vector3d> tic_chain;

    if (!applyCameraSetup(p.numOfCam(), p.cameraIds(), imu_chain, stereo_chain, focal, width, height, ric_chain,
                          tic_chain))
        return false;

    p.setFocalLength(focal);
    p.setImageCol(width);
    p.setImageRow(height);
    setMinParallaxFromPixels(p, keyframe_parallax_px, focal);

    if (estimate_extrinsic != 2)
    {
        p.setRic(std::move(ric_chain));
        p.setTic(std::move(tic_chain));
    }
    return true;
}

void warnExtrinsicMode(int estimate_extrinsic)
{
    if (estimate_extrinsic == 2)
        LOG_TXT_LEVEL(logging::ValueLogger::Level::WARNING,
                      "have no prior about extrinsic param, calibrate extrinsic param");
    else if (estimate_extrinsic == 1)
        LOG_TXT_LEVEL(logging::ValueLogger::Level::WARNING, " Optimize extrinsic param around initial guess!");
    else if (estimate_extrinsic == 0)
        LOG_TXT_LEVEL(logging::ValueLogger::Level::WARNING, " fix extrinsic param ");
}

std::vector<int> readCameraIds(cv::FileStorage &fs, int num_cam)
{
    std::vector<int> ids;
    const cv::FileNode node = fs["camera_ids"];
    if (!node.empty())
    {
        if (node.type() == cv::FileNode::SEQ)
        {
            for (auto it = node.begin(); it != node.end(); ++it)
                ids.push_back(static_cast<int>(*it));
        }
        else
        {
            cv::Mat mat;
            node >> mat;
            if (!mat.empty())
            {
                if (mat.rows == 1 && mat.cols >= 1)
                {
                    for (int c = 0; c < mat.cols; ++c)
                        ids.push_back(mat.at<int>(0, c));
                }
                else if (mat.cols == 1 && mat.rows >= 1)
                {
                    for (int r = 0; r < mat.rows; ++r)
                        ids.push_back(mat.at<int>(r, 0));
                }
            }
        }
    }
    if (ids.empty())
    {
        for (int i = 0; i < num_cam; ++i)
            ids.push_back(i);
    }
    return ids;
}

class ConfigLoader
{
  public:
    explicit ConfigLoader(VinsParameters &params) : params_(params) {}

    void load(const std::string &config_file)
    {
        fs_.open(config_file, cv::FileStorage::READ);
        if (!fs_.isOpened())
        {
            std::cerr << "ERROR: Wrong path to settings" << std::endl;
            return;
        }

        loadLayout();
        const double keyframe_parallax_px = fs_["keyframe_parallax"];
        loadSolver();
        loadOutput();
        loadImu();
        loadImageDefaults();

        const int estimate_extrinsic = readEstimateExtrinsic();
        params_.setEstimateExtrinsic(estimate_extrinsic);
        params_.clearExtrinsics();

        if (!loadFromCamChains(config_file, estimate_extrinsic, keyframe_parallax_px))
            loadFocalFallback(keyframe_parallax_px);

        LOG_TXT("focal_length: ", params_.focalLength(), " ROW: ", params_.imageRow(), " COL: ", params_.imageCol());
        for (int s = 0; s < params_.numOfCam(); ++s)
            LOG_TXT("camera slot ", s, " -> cam", params_.cameraId(s));

        finalizeExtrinsics(estimate_extrinsic);
        logExtrinsics();
        loadTimingAndDepth();

        fs_.release();
    }

  private:
    void loadLayout()
    {
        if (!fs_["window_size"].empty())
            params_.setWindowSize(static_cast<int>(fs_["window_size"]));
        else
            params_.setWindowSize(10);

        if (!fs_["num_of_cam"].empty())
            params_.setNumOfCam(static_cast<int>(fs_["num_of_cam"]));
        else
            params_.setNumOfCam(1);

        if (!fs_["max_feature_count"].empty())
            params_.setMaxFeatureCountPerCam(static_cast<int>(fs_["max_feature_count"]));
        else
            params_.setMaxFeatureCountPerCam(1000);

        if (params_.windowSize() < 2)
        {
            LOG_TXT_LEVEL(logging::ValueLogger::Level::WARNING, "window_size ", params_.windowSize(),
                          " too small, using 2");
            params_.setWindowSize(2);
        }
        if (params_.numOfCam() < 1)
        {
            LOG_TXT_LEVEL(logging::ValueLogger::Level::WARNING, "num_of_cam ", params_.numOfCam(), " invalid, using 1");
            params_.setNumOfCam(1);
        }
        if (params_.maxFeatureCountPerCam() < 10)
        {
            LOG_TXT_LEVEL(logging::ValueLogger::Level::WARNING, "max_feature_count (per cam) ",
                          params_.maxFeatureCountPerCam(), " too small, using 10");
            params_.setMaxFeatureCountPerCam(10);
        }

        std::vector<int> camera_ids = readCameraIds(fs_, params_.numOfCam());
        if (static_cast<int>(camera_ids.size()) != params_.numOfCam())
        {
            LOG_TXT_LEVEL(logging::ValueLogger::Level::WARNING, "camera_ids size ", camera_ids.size(),
                          " != num_of_cam ", params_.numOfCam(), ", using 0..N-1");
            camera_ids.clear();
            for (int i = 0; i < params_.numOfCam(); ++i)
                camera_ids.push_back(i);
        }
        params_.setCameraIds(std::move(camera_ids));
    }

    void loadSolver()
    {
        params_.setSolverTime(fs_["max_solver_time"]);
        params_.setNumIterations(fs_["max_num_iterations"]);
    }

    void loadOutput()
    {
        std::string output_path;
        fs_["output_path"] >> output_path;
        params_.setVinsResultPath(output_path + "/vins_result_no_loop.csv");
        std::cout << "result path " << params_.vinsResultPath() << std::endl;

        FileSystemHelper::createDirectoryIfNotExists(output_path.c_str());

        std::ofstream fout(params_.vinsResultPath(), std::ios::out);
        fout.close();
    }

    void loadImu()
    {
        params_.setAccNoise(fs_["acc_n"]);
        params_.setAccRandomWalk(fs_["acc_w"]);
        params_.setGyrNoise(fs_["gyr_n"]);
        params_.setGyrRandomWalk(fs_["gyr_w"]);
        params_.setGravityNorm(fs_["g_norm"]);
    }

    void loadImageDefaults()
    {
        params_.setImageRow(fs_["image_height"]);
        params_.setImageCol(fs_["image_width"]);
    }

    int readEstimateExtrinsic() { return static_cast<int>(fs_["estimate_extrinsic"]); }

    bool loadLegacyImuCam0(Eigen::Matrix3d &R_ic0, Eigen::Vector3d &t_ic0)
    {
        if (fs_["extrinsicRotation"].empty() || fs_["extrinsicTranslation"].empty())
            return false;
        cv::Mat cv_R, cv_T;
        fs_["extrinsicRotation"] >> cv_R;
        fs_["extrinsicTranslation"] >> cv_T;
        cv::cv2eigen(cv_R, R_ic0);
        cv::cv2eigen(cv_T, t_ic0);
        Eigen::Quaterniond Q(R_ic0);
        R_ic0 = Q.normalized().toRotationMatrix();
        return true;
    }

    bool loadFromCamChains(const std::string &config_file, int estimate_extrinsic, double keyframe_parallax_px)
    {
        std::string stereo_key;
        if (!fs_["cam_chain_file"].empty())
            fs_["cam_chain_file"] >> stereo_key;

        std::string imu_key;
        if (!fs_["cam_chain_imu_file"].empty())
            fs_["cam_chain_imu_file"] >> imu_key;

        const std::string stereo_path = resolveCamChainPath(config_file, stereo_key);
        const std::string imu_path = resolveCamChainImuPath(config_file, imu_key);

        CamChainFile stereo_chain;
        CamChainFile imu_chain;
        const bool stereo_ok = loadCamChainYaml(stereo_path, stereo_chain);
        const bool imu_ok = loadCamChainYaml(imu_path, imu_chain);

        if (!imu_ok)
        {
            if (!imu_key.empty() || !fs_["cam_chain_imu_file"].empty())
                LOG_TXT_LEVEL(logging::ValueLogger::Level::WARNING, "cam_chain_imu_file not loaded: ", imu_path);

            if (estimate_extrinsic == 2)
                return false;

            Eigen::Matrix3d R_ic0;
            Eigen::Vector3d t_ic0;
            if (!loadLegacyImuCam0(R_ic0, t_ic0))
                return false;

            if (!stereo_ok)
                return false;

            CamChainFile imu_from_legacy;
            CamNode &n0 = imu_from_legacy.cameras[0];
            Eigen::Matrix4d T_imu_cam = Eigen::Matrix4d::Identity();
            T_imu_cam.block<3, 3>(0, 0) = R_ic0;
            T_imu_cam.block<3, 1>(0, 3) = t_ic0;
            n0.T_cam_imu = T_imu_cam.inverse();
            n0.has_T_cam_imu = true;
            LOG_TXT_LEVEL(logging::ValueLogger::Level::WARNING,
                          "using simulation_config extrinsicRotation/Translation as legacy IMU–cam0");

            return applyCamerasToParams(params_, imu_from_legacy, stereo_chain, estimate_extrinsic,
                                        keyframe_parallax_px);
        }

        if (!stereo_ok)
        {
            LOG_TXT_LEVEL(logging::ValueLogger::Level::WARNING, "cam_chain_file not loaded: ", stereo_path);
            return applyCamerasToParams(params_, imu_chain, imu_chain, estimate_extrinsic, keyframe_parallax_px);
        }

        LOG_TXT("Loaded cam_chain-imucam from ", imu_path);
        LOG_TXT("Loaded cam_chain from ", stereo_path);
        return applyCamerasToParams(params_, imu_chain, stereo_chain, estimate_extrinsic, keyframe_parallax_px);
    }

    void loadFocalFallback(double keyframe_parallax_px)
    {
        if (!fs_["focal_length"].empty())
        {
            params_.setFocalLength(static_cast<double>(fs_["focal_length"]));
            LOG_TXT_LEVEL(logging::ValueLogger::Level::WARNING,
                          "focal_length from config (no cam_chain); prefer cam_chain_file");
        }
        else
        {
            LOG_TXT_LEVEL(logging::ValueLogger::Level::WARNING, "no cam_chain focal; using default focal_length=460");
            params_.setFocalLength(460.0);
        }
        setMinParallaxFromPixels(params_, keyframe_parallax_px, params_.focalLength());
    }

    void finalizeExtrinsics(int estimate_extrinsic)
    {
        if (params_.ric().empty())
        {
            warnExtrinsicMode(estimate_extrinsic);
            if (estimate_extrinsic == 2)
            {
                for (int c = 0; c < params_.numOfCam(); ++c)
                    params_.addExtrinsic(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
            }
            else
            {
                Eigen::Matrix3d R_ic0 = Eigen::Matrix3d::Identity();
                Eigen::Vector3d t_ic0 = Eigen::Vector3d::Zero();
                loadLegacyImuCam0(R_ic0, t_ic0);
                for (int c = 0; c < params_.numOfCam(); ++c)
                    params_.addExtrinsic(R_ic0, t_ic0);
            }
            return;
        }

        if (estimate_extrinsic == 2)
        {
            warnExtrinsicMode(estimate_extrinsic);
            params_.clearExtrinsics();
            for (int c = 0; c < params_.numOfCam(); ++c)
                params_.addExtrinsic(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
        }
        else
        {
            warnExtrinsicMode(estimate_extrinsic);
        }
    }

    void logExtrinsics()
    {
        if (params_.ric().empty())
            return;

        for (int s = 0; s < params_.numOfCam() && s < static_cast<int>(params_.ric().size()); ++s)
        {
            LOG_TXT("Extrinsic slot ", s, " (cam", params_.cameraId(s), ") R:\n", params_.ric(s));
            LOG_TXT("Extrinsic slot ", s, " (cam", params_.cameraId(s), ") T: ", params_.tic(s).transpose());
        }
    }

    void loadTimingAndDepth()
    {
        params_.setInitDepth(5.0);

        params_.setTd(fs_["td"]);
        params_.setEstimateTd(fs_["estimate_td"]);
        if (params_.estimateTd())
            LOG_TXT("Unsynchronized sensors, online estimate time offset, initial td: ", params_.td());
        else
            LOG_TXT("Synchronized sensors, fix time offset: ", params_.td());

        params_.setRollingShutter(fs_["rolling_shutter"]);
        if (params_.rollingShutter())
        {
            params_.setRollingShutterTr(fs_["rolling_shutter_tr"]);
            LOG_TXT("rolling shutter camera, read out time per line: ", params_.rollingShutterTr());
        }
        else
        {
            params_.setRollingShutterTr(0);
        }
    }

    VinsParameters &params_;
    cv::FileStorage fs_;
};

} // namespace

VinsParameters &VinsParameters::instance()
{
    static VinsParameters params;
    return params;
}

namespace {

int slotForCameraId(const VinsParameters &p, int camera_id)
{
    for (int s = 0; s < p.numOfCam(); ++s)
    {
        if (p.cameraId(s) == camera_id)
            return s;
    }
    throw std::runtime_error("camera_id not in active camera_ids");
}

} // namespace

const Eigen::Matrix3d &VinsParameters::ricForCamera(int camera_id) const
{
    return ric(slotForCameraId(*this, camera_id));
}

const Eigen::Vector3d &VinsParameters::ticForCamera(int camera_id) const
{
    return tic(slotForCameraId(*this, camera_id));
}

void VinsParameters::loadFromConfig(const std::string &config_file)
{
    ConfigLoader(*this).load(config_file);
}

void readParameters(const std::string &config_file)
{
    vinsParameters().loadFromConfig(config_file);
}
