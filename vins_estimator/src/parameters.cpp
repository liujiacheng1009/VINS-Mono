#include "parameters.h"

#include "cam_chain.h"

#include <log_value/log_macros.h>
#include <opencv2/core/eigen.hpp>

#include <fstream>

namespace {

void setMinParallaxFromPixels(VinsParameters &p, double keyframe_parallax_px, double focal)
{
    if (focal > 0.0)
        p.setMinParallax(keyframe_parallax_px / focal);
}

bool applyCamChainToParams(VinsParameters &p,
                           const CamChainData &chain,
                           int num_cam,
                           const Eigen::Matrix3d &R_ic0,
                           const Eigen::Vector3d &t_ic0,
                           int estimate_extrinsic,
                           double keyframe_parallax_px)
{
    double focal = p.focalLength();
    double width = p.imageCol();
    double height = p.imageRow();
    std::vector<Eigen::Matrix3d> ric_chain;
    std::vector<Eigen::Vector3d> tic_chain;

    if (!applyCamChain(num_cam, chain, R_ic0, t_ic0, focal, width, height, ric_chain, tic_chain))
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

        Eigen::Matrix3d R_ic0 = Eigen::Matrix3d::Identity();
        Eigen::Vector3d t_ic0 = Eigen::Vector3d::Zero();
        readImuCam0Extrinsic(estimate_extrinsic, R_ic0, t_ic0);

        if (!loadFromCamChain(config_file, estimate_extrinsic, R_ic0, t_ic0, keyframe_parallax_px))
            loadFocalFallback(keyframe_parallax_px);

        LOG_TXT("focal_length: ", params_.focalLength(), " ROW: ", params_.imageRow(), " COL: ", params_.imageCol());

        finalizeExtrinsics(estimate_extrinsic, R_ic0, t_ic0);
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
            params_.setMaxFeatureCount(static_cast<int>(fs_["max_feature_count"]));
        else
            params_.setMaxFeatureCount(1000);

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
        if (params_.maxFeatureCount() < 10)
        {
            LOG_TXT_LEVEL(logging::ValueLogger::Level::WARNING, "max_feature_count ", params_.maxFeatureCount(),
                          " too small, using 10");
            params_.setMaxFeatureCount(10);
        }
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

    void readImuCam0Extrinsic(int estimate_extrinsic, Eigen::Matrix3d &R_ic0, Eigen::Vector3d &t_ic0)
    {
        if (estimate_extrinsic == 2)
            return;

        cv::Mat cv_R, cv_T;
        fs_["extrinsicRotation"] >> cv_R;
        fs_["extrinsicTranslation"] >> cv_T;
        cv::cv2eigen(cv_R, R_ic0);
        cv::cv2eigen(cv_T, t_ic0);
        Eigen::Quaterniond Q(R_ic0);
        R_ic0 = Q.normalized().toRotationMatrix();
    }

    bool loadFromCamChain(const std::string &config_file,
                          int estimate_extrinsic,
                          const Eigen::Matrix3d &R_ic0,
                          const Eigen::Vector3d &t_ic0,
                          double keyframe_parallax_px)
    {
        std::string cam_chain_file_key;
        if (!fs_["cam_chain_file"].empty())
            fs_["cam_chain_file"] >> cam_chain_file_key;

        const std::string cam_chain_path = resolveCamChainPath(config_file, cam_chain_file_key);
        CamChainData cam_chain;
        if (!loadCamChainFile(cam_chain_path, cam_chain))
        {
            if (!cam_chain_file_key.empty())
                LOG_TXT_LEVEL(logging::ValueLogger::Level::WARNING, "cam_chain_file set but not loaded: ",
                              cam_chain_path);
            return false;
        }

        LOG_TXT("Loaded cam_chain from ", cam_chain_path, " (", cam_chain.cameras.size(),
                " cameras in file, using ", params_.numOfCam(), ")");

        if (!applyCamChainToParams(params_, cam_chain, params_.numOfCam(), R_ic0, t_ic0, estimate_extrinsic,
                                   keyframe_parallax_px))
        {
            LOG_TXT_LEVEL(logging::ValueLogger::Level::WARNING, "cam_chain: apply failed");
            return false;
        }
        return true;
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

    void finalizeExtrinsics(int estimate_extrinsic, const Eigen::Matrix3d &R_ic0, const Eigen::Vector3d &t_ic0)
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
                for (int c = 0; c < params_.numOfCam(); ++c)
                    params_.addExtrinsic(R_ic0, t_ic0);
                if (params_.numOfCam() > 1)
                    LOG_TXT_LEVEL(logging::ValueLogger::Level::WARNING,
                                  "num_of_cam=", params_.numOfCam(),
                                  ": using the same extrinsicRotation/Translation for every camera; "
                                  "set cam_chain_file for stereo extrinsics.");
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

        LOG_TXT("Extrinsic_R (cam0):\n", params_.ric()[0]);
        LOG_TXT("Extrinsic_T (cam0):\n", params_.tic()[0].transpose());
        if (params_.numOfCam() > 1 && params_.ric().size() > 1)
        {
            LOG_TXT("Extrinsic_R (cam1):\n", params_.ric()[1]);
            LOG_TXT("Extrinsic_T (cam1):\n", params_.tic()[1].transpose());
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

void VinsParameters::loadFromConfig(const std::string &config_file)
{
    ConfigLoader(*this).load(config_file);
}

void readParameters(const std::string &config_file)
{
    vinsParameters().loadFromConfig(config_file);
}
