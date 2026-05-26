#pragma once

#include "parameters.h"
#include "feature_manager.h"
#include "state_manager.h"
#include "utility/utility.h"
#include "utility/tic_toc.h"
#include "utility/simple_types.h"
#include "utility/image_frame_input.h"

#include <ceres/ceres.h>
#include "factor/imu_factor.h"
#include "factor/pose_local_parameterization.h"
#include "factor/projection_factor.h"
#include "factor/marginalization_factor.h"

#include <map>
#include <opencv2/core/eigen.hpp>

class Estimator
{
  public:
    Estimator();

    void setParameter();

    void processIMU(double t, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);
    void processImage(const ImageFrameInput &input);
    void processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const SimpleHeader &header);
    void initializeWithGroundTruth(double t, const Vector3d &P, const Matrix3d &R, const Vector3d &V,
                                   const Vector3d &acc, const Vector3d &gyr);

    void clearState();
    void slideWindow();
    void solveOdometry();
    void slideWindowNew();
    void slideWindowOld();
    void optimization();
    bool failureDetection();

    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };

    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };

    StateManager state_;

    SolverFlag solver_flag;
    MarginalizationFlag marginalization_flag;
    Vector3d g;

    FeatureManager f_manager;

    bool first_imu;
    bool failure_occur;

    Vector3d acc_0, gyr_0;

    int sum_of_back, sum_of_front;

  private:
    void syncStateToParameters();
    void syncParametersToState();
};