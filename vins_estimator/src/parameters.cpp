#include "parameters.h"

VinsParameters &VinsParameters::instance()
{
    static VinsParameters params;
    return params;
}

void VinsParameters::loadFromConfig(const std::string &config_file)
{
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
        return;
    }

    std::string imu_topic;
    fsSettings["imu_topic"] >> imu_topic;
    setImuTopic(std::move(imu_topic));

    setSolverTime(fsSettings["max_solver_time"]);
    setNumIterations(fsSettings["max_num_iterations"]);
    double min_parallax = fsSettings["keyframe_parallax"];
    setMinParallax(min_parallax / FOCAL_LENGTH);

    std::string output_path;
    fsSettings["output_path"] >> output_path;
    setVinsResultPath(output_path + "/vins_result_no_loop.csv");
    std::cout << "result path " << vinsResultPath() << std::endl;

    FileSystemHelper::createDirectoryIfNotExists(output_path.c_str());

    std::ofstream fout(vinsResultPath(), std::ios::out);
    fout.close();

    setAccNoise(fsSettings["acc_n"]);
    setAccRandomWalk(fsSettings["acc_w"]);
    setGyrNoise(fsSettings["gyr_n"]);
    setGyrRandomWalk(fsSettings["gyr_w"]);
    setGravityNorm(fsSettings["g_norm"]);
    setImageRow(fsSettings["image_height"]);
    setImageCol(fsSettings["image_width"]);
    ROS_INFO("ROW: %f COL: %f ", imageRow(), imageCol());

    const int estimate_extrinsic = fsSettings["estimate_extrinsic"];
    setEstimateExtrinsic(estimate_extrinsic);
    clearExtrinsics();

    if (estimate_extrinsic == 2)
    {
        ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        addExtrinsic(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
        setExCalibResultPath(output_path + "/extrinsic_parameter.csv");
    }
    else
    {
        if (estimate_extrinsic == 1)
        {
            ROS_WARN(" Optimize extrinsic param around initial guess!");
            setExCalibResultPath(output_path + "/extrinsic_parameter.csv");
        }
        if (estimate_extrinsic == 0)
            ROS_WARN(" fix extrinsic param ");

        cv::Mat cv_R, cv_T;
        fsSettings["extrinsicRotation"] >> cv_R;
        fsSettings["extrinsicTranslation"] >> cv_T;
        Eigen::Matrix3d eigen_R;
        Eigen::Vector3d eigen_T;
        cv::cv2eigen(cv_R, eigen_R);
        cv::cv2eigen(cv_T, eigen_T);
        Eigen::Quaterniond Q(eigen_R);
        eigen_R = Q.normalized();
        addExtrinsic(eigen_R, eigen_T);
        ROS_INFO_STREAM("Extrinsic_R : " << std::endl << ric()[0]);
        ROS_INFO_STREAM("Extrinsic_T : " << std::endl << tic()[0].transpose());
    }

    setInitDepth(5.0);
    setBiasAccThreshold(0.1);
    setBiasGyrThreshold(0.1);

    setTd(fsSettings["td"]);
    setEstimateTd(fsSettings["estimate_td"]);
    if (estimateTd())
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << td());
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << td());

    setRollingShutter(fsSettings["rolling_shutter"]);
    if (rollingShutter())
    {
        setRollingShutterTr(fsSettings["rolling_shutter_tr"]);
        ROS_INFO_STREAM("rolling shutter camera, read out time per line: " << rollingShutterTr());
    }
    else
    {
        setRollingShutterTr(0);
    }

    fsSettings.release();
}

void readParameters(const std::string &config_file)
{
    vinsParameters().loadFromConfig(config_file);
}
