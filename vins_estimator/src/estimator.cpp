#include "estimator.h"

Estimator::Estimator()
    : f_manager()
{
    ROS_INFO("init begins");
    clearState();
}

void Estimator::setParameter()
{
    state_.initFromConfig();
    ProjectionFactor::sqrt_info = focalLength() / 1.5 * Matrix2d::Identity();
    g = vinsParameters().gravity();
}

void Estimator::initializeWithGroundTruth(double t, const Vector3d &P, const Matrix3d &R, const Vector3d &V,
                                          const Vector3d &acc, const Vector3d &gyr)
{
    const int idx = std::min(state_.slotCount(), state_.windowSize());
    SimpleHeader header;
    header.stamp = SimpleTime(t);
    header.frame_id = "world";
    if (header.seq == kInvalidFrameId)
        header.seq = idx;
    state_.initializeFrameAtSlot(idx, header.seq, header, P, R, V, Vector3d::Zero(), Vector3d::Zero(), acc, gyr);
    acc_0 = acc;
    gyr_0 = gyr;
    first_imu = true;
}

void Estimator::clearState()
{
    state_.clear();

    solver_flag = INITIAL;
    first_imu = false;
    sum_of_back = 0;
    sum_of_front = 0;
    g = vinsParameters().gravity();

    state_.initFromConfig();
    f_manager.clearState();

    failure_occur = 0;
}

void Estimator::processIMU(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    const int slot = state_.slotCount();
    if (!state_.preIntegrationAtSlot(slot))
    {
        state_.preIntegrationAtSlot(slot) =
            std::make_shared<Integrator>(acc_0, gyr_0, state_.accBiasAtSlot(slot), state_.gyrBiasAtSlot(slot));
    }
    if (slot != 0)
    {
        state_.preIntegrationAtSlot(slot)->process(dt, linear_acceleration, angular_velocity);
        state_.tmpPreIntegration()->process(dt, linear_acceleration, angular_velocity);

        state_.pushImuSampleAtSlot(slot, dt, linear_acceleration, angular_velocity);

        state_.propagateImuAtSlot(slot, dt, linear_acceleration, angular_velocity, acc_0, gyr_0, g);
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void Estimator::processImage(const ImageFrameInput &input)
{
    const int slot = state_.slotCount();
    const FrameId frame_id =
        input.id != kInvalidFrameId ? input.id
                                    : (input.header.seq != kInvalidFrameId ? input.header.seq
                                                                           : static_cast<FrameId>(slot));
    SimpleHeader header = input.header;
    header.seq = frame_id;
    state_.bindFrame(slot, frame_id, header);

    const auto &image = input.features;
    ROS_DEBUG("new image coming ------------------------------------------");
    ROS_DEBUG("Adding feature points %lu", image.size());
    if (f_manager.addFeatureCheckParallax(slot, image, state_.timeDelay()))
        marginalization_flag = MARGIN_OLD;
    else
        marginalization_flag = MARGIN_SECOND_NEW;

    ROS_DEBUG("this frame is--------------------%s", marginalization_flag ? "reject" : "accept");
    ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    ROS_DEBUG("Solving %d", slot);
    ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());

    state_.tmpPreIntegration() =
        std::make_shared<Integrator>(acc_0, gyr_0, state_.accBiasAtSlot(slot), state_.gyrBiasAtSlot(slot));

    if (solver_flag == INITIAL)
    {
        if (slot == state_.windowSize())
        {
            solver_flag = NON_LINEAR;
            solveOdometry();
            slideWindow();
            f_manager.removeFailures();
            ROS_INFO("GT Initialization finish!");
            state_.updateKeyframeSnapshot();
        }
        else
            state_.setSlotCount(slot + 1);
    }
    else
    {
        TicToc t_solve;
        solveOdometry();
        ROS_DEBUG("solver costs: %fms", t_solve.toc());

        if (failureDetection())
        {
            ROS_WARN("failure detection!");
            failure_occur = 1;
            clearState();
            setParameter();
            ROS_WARN("system reboot!");
            return;
        }

        TicToc t_margin;
        slideWindow();
        f_manager.removeFailures();
        ROS_DEBUG("marginalization costs: %fms", t_margin.toc());

        state_.updateKeyframeSnapshot();
    }
}

void Estimator::processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image,
                            const SimpleHeader &header)
{
    ImageFrameInput input;
    input.header = header;
    input.id = header.seq;
    input.features = image;
    processImage(input);
}

void Estimator::solveOdometry()
{
    if (state_.slotCount() < state_.windowSize())
        return;
    if (solver_flag == NON_LINEAR)
    {
        TicToc t_tri;
        f_manager.triangulate(state_);
        ROS_DEBUG("triangulation costs %f", t_tri.toc());
        optimization();
    }
}

void Estimator::syncStateToParameters()
{
    state_.syncToParameters();
    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        state_.featureParameter(i)[0] = dep(i);
}

void Estimator::syncParametersToState()
{
    SyncFromOptions opts;
    if (failure_occur)
    {
        opts.align_origin_P0 = state_.lastPosition0();
        opts.align_origin_R0 = state_.lastRotation0();
        failure_occur = 0;
    }
    state_.syncFromParameters(opts);

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        dep(i) = state_.featureParameter(i)[0];
    f_manager.setDepth(dep);
}

bool Estimator::failureDetection()
{
    if (f_manager.last_track_num < 2)
    {
        ROS_INFO(" little feature %d", f_manager.last_track_num);
    }
    if (state_.accBiasAtSlot(state_.windowSize()).norm() > 2.5)
    {
        ROS_INFO(" big IMU acc bias estimation %f", state_.accBiasAtSlot(state_.windowSize()).norm());
        return true;
    }
    if (state_.gyrBiasAtSlot(state_.windowSize()).norm() > 1.0)
    {
        ROS_INFO(" big IMU gyr bias estimation %f", state_.gyrBiasAtSlot(state_.windowSize()).norm());
        return true;
    }

    const Vector3d tmp_P = state_.positionAtSlot(state_.windowSize());
    if ((tmp_P - state_.lastPosition()).norm() > 5)
    {
        ROS_INFO(" big translation");
        return true;
    }
    if (abs(tmp_P.z() - state_.lastPosition().z()) > 1)
    {
        ROS_INFO(" big z translation");
        return true;
    }
    const Matrix3d tmp_R = state_.rotationAtSlot(state_.windowSize());
    Matrix3d delta_R = tmp_R.transpose() * state_.lastRotation();
    Quaterniond delta_Q(delta_R);
    double delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
    if (delta_angle > 50)
    {
        ROS_INFO(" big delta_angle ");
    }
    return false;
}

void Estimator::optimization()
{
    ceres::Problem problem;
    ceres::LossFunction *loss_function = new ceres::CauchyLoss(1.0);
    for (int i = 0; i < state_.windowSize() + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(state_.poseParameter(i), SIZE_POSE, local_parameterization);
        problem.AddParameterBlock(state_.speedBiasParameter(i), SIZE_SPEEDBIAS);
    }
    for (int i = 0; i < numOfCam(); i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(state_.extrinsicParameter(i), SIZE_POSE, local_parameterization);
        if (!vinsParameters().estimateExtrinsic())
            problem.SetParameterBlockConstant(state_.extrinsicParameter(i));
    }
    if (vinsParameters().estimateTd())
        problem.AddParameterBlock(state_.timeDelayParameter(), 1);

    TicToc t_whole, t_prepare;
    syncStateToParameters();

    auto &last_marginalization_info = state_.lastMarginalizationInfo();
    auto &last_marginalization_parameter_blocks = state_.lastMarginalizationParameterBlocks();

    if (last_marginalization_info)
    {
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL, last_marginalization_parameter_blocks);
    }

    for (int i = 0; i < state_.windowSize(); i++)
    {
        int j = i + 1;
        auto &pre_int = state_.preIntegrationAtSlot(j);
        if (!pre_int || pre_int->sum_dt <= 1e-6 || pre_int->sum_dt > 10.0)
            continue;
        IMUFactor *imu_factor = new IMUFactor(pre_int);
        problem.AddResidualBlock(imu_factor, NULL, state_.poseParameter(i), state_.speedBiasParameter(i),
                                 state_.poseParameter(j), state_.speedBiasParameter(j));
    }

    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_slot < state_.windowSize() - 2))
            continue;

        ++feature_index;

        int imu_i = it_per_id.start_slot, imu_j = imu_i - 1;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i == imu_j)
                continue;

            Vector3d pts_j = it_per_frame.point;
            if (vinsParameters().estimateTd())
            {
                ProjectionFactor *f = new ProjectionFactor(
                    pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                    it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td,
                    it_per_id.feature_per_frame[0].uv.y(), it_per_frame.uv.y());
                problem.AddResidualBlock(f, loss_function, state_.poseParameter(imu_i), state_.poseParameter(imu_j),
                                         state_.extrinsicParameter(0), state_.featureParameter(feature_index),
                                         state_.timeDelayParameter());
            }
            else
            {
                ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                problem.AddResidualBlock(f, loss_function, state_.poseParameter(imu_i), state_.poseParameter(imu_j),
                                         state_.extrinsicParameter(0), state_.featureParameter(feature_index));
            }
            f_m_cnt++;
        }
    }

    ROS_DEBUG("visual measurement count: %d", f_m_cnt);
    ROS_DEBUG("prepare for ceres: %f", t_prepare.toc());

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = vinsParameters().numIterations();
    if (marginalization_flag == MARGIN_OLD)
        options.max_solver_time_in_seconds = vinsParameters().solverTime() * 4.0 / 5.0;
    else
        options.max_solver_time_in_seconds = vinsParameters().solverTime();
    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    ROS_DEBUG("solver costs: %f", t_solver.toc());

    syncParametersToState();

    TicToc t_whole_marginalization;
    if (marginalization_flag == MARGIN_OLD)
    {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        syncStateToParameters();

        if (last_marginalization_info)
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {
                if (last_marginalization_parameter_blocks[i] == state_.poseParameter(0) ||
                    last_marginalization_parameter_blocks[i] == state_.speedBiasParameter(0))
                    drop_set.push_back(i);
            }
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info =
                new ResidualBlockInfo(marginalization_factor, NULL, last_marginalization_parameter_blocks, drop_set);
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        {
            auto &pre1 = state_.preIntegrationAtSlot(1);
            if (pre1 && pre1->sum_dt > 1e-6 && pre1->sum_dt < 10.0)
            {
                IMUFactor *imu_factor = new IMUFactor(pre1);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                    imu_factor, NULL,
                    vector<double *>{state_.poseParameter(0), state_.speedBiasParameter(0), state_.poseParameter(1),
                                     state_.speedBiasParameter(1)},
                    vector<int>{0, 1});
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        {
            int feature_index_margin = -1;
            for (auto &it_per_id : f_manager.feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (!(it_per_id.used_num >= 2 && it_per_id.start_slot < state_.windowSize() - 2))
                    continue;

                ++feature_index_margin;

                int imu_i = it_per_id.start_slot, imu_j = imu_i - 1;
                if (imu_i != 0)
                    continue;

                Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;
                    if (imu_i == imu_j)
                        continue;

                    Vector3d pts_j = it_per_frame.point;
                    if (vinsParameters().estimateTd())
                    {
                        ProjectionFactor *f = new ProjectionFactor(
                            pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                            it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td,
                            it_per_id.feature_per_frame[0].uv.y(), it_per_frame.uv.y());
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                            f, loss_function,
                            vector<double *>{state_.poseParameter(imu_i), state_.poseParameter(imu_j),
                                             state_.extrinsicParameter(0), state_.featureParameter(feature_index_margin),
                                             state_.timeDelayParameter()},
                            vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                    else
                    {
                        ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                            f, loss_function,
                            vector<double *>{state_.poseParameter(imu_i), state_.poseParameter(imu_j),
                                             state_.extrinsicParameter(0), state_.featureParameter(feature_index_margin)},
                            vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                }
            }
        }

        TicToc t_pre_margin;
        marginalization_info->preMarginalize();
        ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());

        TicToc t_margin;
        marginalization_info->marginalize();
        ROS_DEBUG("marginalization %f ms", t_margin.toc());

        std::unordered_map<ParameterBlockId, double *> addr_shift;
        for (int i = 1; i <= state_.windowSize(); i++)
        {
            addr_shift[reinterpret_cast<ParameterBlockId>(state_.poseParameter(i))] = state_.poseParameter(i - 1);
            addr_shift[reinterpret_cast<ParameterBlockId>(state_.speedBiasParameter(i))] =
                state_.speedBiasParameter(i - 1);
        }
        for (int i = 0; i < numOfCam(); i++)
            addr_shift[reinterpret_cast<ParameterBlockId>(state_.extrinsicParameter(i))] = state_.extrinsicParameter(i);
        if (vinsParameters().estimateTd())
            addr_shift[reinterpret_cast<ParameterBlockId>(state_.timeDelayParameter())] = state_.timeDelayParameter();

        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
    }
    else
    {
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks),
                       state_.poseParameter(state_.windowSize() - 1)))
        {
            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            syncStateToParameters();
            if (last_marginalization_info)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    ROS_ASSERT(last_marginalization_parameter_blocks[i] != state_.speedBiasParameter(state_.windowSize() - 1));
                    if (last_marginalization_parameter_blocks[i] == state_.poseParameter(state_.windowSize() - 1))
                        drop_set.push_back(i);
                }
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info =
                    new ResidualBlockInfo(marginalization_factor, NULL, last_marginalization_parameter_blocks, drop_set);
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            TicToc t_pre_margin;
            marginalization_info->preMarginalize();
            TicToc t_margin;
            marginalization_info->marginalize();

            std::unordered_map<ParameterBlockId, double *> addr_shift;
            for (int i = 0; i <= state_.windowSize(); i++)
            {
                if (i == state_.windowSize() - 1)
                    continue;
                else if (i == state_.windowSize())
                {
                    addr_shift[reinterpret_cast<ParameterBlockId>(state_.poseParameter(i))] = state_.poseParameter(i - 1);
                    addr_shift[reinterpret_cast<ParameterBlockId>(state_.speedBiasParameter(i))] =
                        state_.speedBiasParameter(i - 1);
                }
                else
                {
                    addr_shift[reinterpret_cast<ParameterBlockId>(state_.poseParameter(i))] = state_.poseParameter(i);
                    addr_shift[reinterpret_cast<ParameterBlockId>(state_.speedBiasParameter(i))] =
                        state_.speedBiasParameter(i);
                }
            }
            for (int i = 0; i < numOfCam(); i++)
                addr_shift[reinterpret_cast<ParameterBlockId>(state_.extrinsicParameter(i))] =
                    state_.extrinsicParameter(i);
            if (vinsParameters().estimateTd())
                addr_shift[reinterpret_cast<ParameterBlockId>(state_.timeDelayParameter())] =
                    state_.timeDelayParameter();

            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
        }
    }
    ROS_DEBUG("whole marginalization costs: %f", t_whole_marginalization.toc());
    ROS_DEBUG("whole time for ceres: %f", t_whole.toc());
}

void Estimator::slideWindow()
{
    const auto mode = (marginalization_flag == MARGIN_OLD) ? StateManager::SlideMode::MARGIN_OLD
                                                         : StateManager::SlideMode::MARGIN_SECOND_NEW;
    state_.slideWindow(mode, acc_0, gyr_0, state_.accBiasAtSlot(state_.windowSize()), state_.gyrBiasAtSlot(state_.windowSize()));
    if (marginalization_flag == MARGIN_OLD)
        slideWindowOld();
    else
        slideWindowNew();
}

void Estimator::slideWindowNew()
{
    sum_of_front++;
    f_manager.removeFront(state_.slotCount());
}

void Estimator::slideWindowOld()
{
    sum_of_back++;

    const bool shift_depth = solver_flag == NON_LINEAR;
    if (shift_depth)
    {
        const ExtrinsicState ex0 = state_.extrinsic(0);
        Matrix3d R0 = state_.backRotation() * ex0.ric;
        Matrix3d R1 = state_.rotationAtSlot(0) * ex0.ric;
        Vector3d P0 = state_.backPosition() + state_.backRotation() * ex0.tic;
        Vector3d P1 = state_.positionAtSlot(0) + state_.rotationAtSlot(0) * ex0.tic;
        f_manager.removeBackShiftDepth(R0, P0, R1, P1);
    }
    else
        f_manager.removeBack();
}
