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

    if (!fsSettings["window_size"].empty())
        setWindowSize(static_cast<int>(fsSettings["window_size"]));
    else
        setWindowSize(10);

    if (!fsSettings["num_of_cam"].empty())
        setNumOfCam(static_cast<int>(fsSettings["num_of_cam"]));
    else
        setNumOfCam(1);

    if (!fsSettings["max_feature_count"].empty())
        setMaxFeatureCount(static_cast<int>(fsSettings["max_feature_count"]));
    else
        setMaxFeatureCount(1000);

    if (!fsSettings["focal_length"].empty())
        setFocalLength(static_cast<double>(fsSettings["focal_length"]));
    else
        setFocalLength(460.0);

    if (windowSize() < 2)
    {
        ROS_WARN("window_size %d too small, using 2", windowSize());
        setWindowSize(2);
    }
    if (numOfCam() < 1)
    {
        ROS_WARN("num_of_cam %d invalid, using 1", numOfCam());
        setNumOfCam(1);
    }
    if (maxFeatureCount() < 10)
    {
        ROS_WARN("max_feature_count %d too small, using 10", maxFeatureCount());
        setMaxFeatureCount(10);
    }

    setSolverTime(fsSettings["max_solver_time"]);
    setNumIterations(fsSettings["max_num_iterations"]);
    double min_parallax = fsSettings["keyframe_parallax"];
    setMinParallax(min_parallax / focalLength());

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
        for (int c = 0; c < numOfCam(); ++c)
            addExtrinsic(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
    }
    else
    {
        if (estimate_extrinsic == 1)
            ROS_WARN(" Optimize extrinsic param around initial guess!");
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
        for (int c = 0; c < numOfCam(); ++c)
            addExtrinsic(eigen_R, eigen_T);
        if (numOfCam() > 1)
            ROS_WARN("num_of_cam=%d: using the same extrinsicRotation/Translation for every camera; "
                     "provide per-camera extrinsics when stereo calibration differs.",
                     numOfCam());
        ROS_INFO_STREAM("Extrinsic_R : " << std::endl << ric()[0]);
        ROS_INFO_STREAM("Extrinsic_T : " << std::endl << tic()[0].transpose());
    }

    setInitDepth(5.0);

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
