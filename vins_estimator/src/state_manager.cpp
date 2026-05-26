#include "state_manager.h"

#include <log_value/log_macros.h>
#include "utility/utility.h"

StateManager::StateManager()
{
    clear();
}

void StateManager::allocateStorage()
{
    window_size_ = vinsParameters().windowSize();
    const int num_cam = vinsParameters().numOfCam();
    const int max_feat = vinsParameters().maxFeatureCount();
    const int n_slots = window_size_ + 1;

    frame_ids_.assign(n_slots, kInvalidFrameId);
    Ps_.assign(n_slots, Vector3d::Zero());
    Vs_.assign(n_slots, Vector3d::Zero());
    Rs_.assign(n_slots, Matrix3d::Identity());
    Bas_.assign(n_slots, Vector3d::Zero());
    Bgs_.assign(n_slots, Vector3d::Zero());
    Headers_.assign(n_slots, SimpleHeader{});
    imu_data_.assign(n_slots, FrameImuData{});

    ric_.assign(num_cam, Matrix3d::Identity());
    tic_.assign(num_cam, Vector3d::Zero());

    para_pose_.assign(n_slots, {});
    para_speed_bias_.assign(n_slots, {});
    para_ex_pose_.assign(num_cam, {});
    para_feature_.assign(max_feat, {});
    para_td_ = {0.0};
}

void StateManager::clear()
{
    allocateStorage();

    slot_count_ = 0;
    id_to_slot_.clear();
    td_ = vinsParameters().td();
    tmp_pre_integration_.reset();
    clearMarginalizationPrior();

    back_R0_.setIdentity();
    back_P0_.setZero();
    last_R_.setIdentity();
    last_P_.setZero();
    last_R0_.setIdentity();
    last_P0_.setZero();
}

void StateManager::initFromConfig()
{
    allocateStorage();
    const auto &cfg = vinsParameters();
    const int num_cam = static_cast<int>(std::min(cfg.ric().size(), ric_.size()));
    for (int i = 0; i < num_cam; i++)
    {
        tic_[i] = cfg.tic()[i];
        ric_[i] = cfg.ric()[i];
    }
    td_ = cfg.td();
}

int StateManager::activeSlotLimit() const
{
    return (slot_count_ >= window_size_) ? window_size_ : slot_count_;
}

int StateManager::latestSlot() const
{
    return std::min(slot_count_, window_size_);
}

void StateManager::rebuildIdToSlot()
{
    id_to_slot_.clear();
    const int limit = activeSlotLimit();
    for (int i = 0; i <= limit; i++)
    {
        if (frame_ids_[i] != kInvalidFrameId)
            id_to_slot_[frame_ids_[i]] = i;
    }
}

void StateManager::unregisterSlot(int slot)
{
    const FrameId id = frame_ids_[slot];
    if (id != kInvalidFrameId)
        id_to_slot_.erase(id);
    frame_ids_[slot] = kInvalidFrameId;
}

void StateManager::bindFrame(int slot, FrameId id, const SimpleHeader &header)
{
    frame_ids_[slot] = id;
    Headers_[slot] = header;
    Headers_[slot].seq = id;
    if (id != kInvalidFrameId)
        id_to_slot_[id] = slot;
}

bool StateManager::contains(FrameId id) const
{
    return id_to_slot_.find(id) != id_to_slot_.end();
}

std::optional<int> StateManager::slotOf(FrameId id) const
{
    const auto it = id_to_slot_.find(id);
    if (it == id_to_slot_.end())
        return std::nullopt;
    return it->second;
}

void StateManager::snapshotOldestFrame()
{
    back_R0_ = Rs_[0];
    back_P0_ = Ps_[0];
}

void StateManager::updateKeyframeSnapshot()
{
    last_R_ = Rs_[window_size_];
    last_P_ = Ps_[window_size_];
    last_R0_ = Rs_[0];
    last_P0_ = Ps_[0];
}

void StateManager::propagateImuAtSlot(int slot, double dt, const Vector3d &acc, const Vector3d &gyr,
                                      const Vector3d &acc_prev, const Vector3d &gyr_prev, const Vector3d &gravity)
{
    Vector3d un_acc_0 = Rs_[slot] * (acc_prev - Bas_[slot]) - gravity;
    Vector3d un_gyr = 0.5 * (gyr_prev + gyr) - Bgs_[slot];
    Rs_[slot] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
    Vector3d un_acc_1 = Rs_[slot] * (acc - Bas_[slot]) - gravity;
    Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    Ps_[slot] += dt * Vs_[slot] + 0.5 * dt * dt * un_acc;
    Vs_[slot] += dt * un_acc;
}

void StateManager::pushImuSampleAtSlot(int slot, double dt, const Vector3d &acc, const Vector3d &gyr)
{
    imu_data_[slot].dt_buf.push_back(dt);
    imu_data_[slot].linear_acceleration_buf.push_back(acc);
    imu_data_[slot].angular_velocity_buf.push_back(gyr);
}

void StateManager::clearImuBufferAtSlot(int slot)
{
    imu_data_[slot].dt_buf.clear();
    imu_data_[slot].linear_acceleration_buf.clear();
    imu_data_[slot].angular_velocity_buf.clear();
}

std::shared_ptr<Integrator> &StateManager::preIntegrationAtSlot(int slot)
{
    return imu_data_[slot].pre_integration;
}

void StateManager::initializeFrameAtSlot(int slot, FrameId id, const SimpleHeader &header,
                                         const Vector3d &P, const Matrix3d &R, const Vector3d &V,
                                         const Vector3d &Ba, const Vector3d &Bg,
                                         const Vector3d &acc_0, const Vector3d &gyr_0)
{
    bindFrame(slot, id, header);
    Ps_[slot] = P;
    Rs_[slot] = R;
    Vs_[slot] = V;
    Bas_[slot] = Ba;
    Bgs_[slot] = Bg;
    if (!imu_data_[slot].pre_integration)
        imu_data_[slot].pre_integration = std::make_shared<Integrator>(acc_0, gyr_0, Bas_[slot], Bgs_[slot]);
}

void StateManager::syncToParameters()
{
    for (int i = 0; i <= window_size_; i++)
    {
        para_pose_[i][0] = Ps_[i].x();
        para_pose_[i][1] = Ps_[i].y();
        para_pose_[i][2] = Ps_[i].z();
        Quaterniond q{Rs_[i]};
        para_pose_[i][3] = q.x();
        para_pose_[i][4] = q.y();
        para_pose_[i][5] = q.z();
        para_pose_[i][6] = q.w();

        para_speed_bias_[i][0] = Vs_[i].x();
        para_speed_bias_[i][1] = Vs_[i].y();
        para_speed_bias_[i][2] = Vs_[i].z();

        para_speed_bias_[i][3] = Bas_[i].x();
        para_speed_bias_[i][4] = Bas_[i].y();
        para_speed_bias_[i][5] = Bas_[i].z();

        para_speed_bias_[i][6] = Bgs_[i].x();
        para_speed_bias_[i][7] = Bgs_[i].y();
        para_speed_bias_[i][8] = Bgs_[i].z();
    }
    for (int i = 0; i < static_cast<int>(para_ex_pose_.size()); i++)
    {
        para_ex_pose_[i][0] = tic_[i].x();
        para_ex_pose_[i][1] = tic_[i].y();
        para_ex_pose_[i][2] = tic_[i].z();
        Quaterniond q{ric_[i]};
        para_ex_pose_[i][3] = q.x();
        para_ex_pose_[i][4] = q.y();
        para_ex_pose_[i][5] = q.z();
        para_ex_pose_[i][6] = q.w();
    }
    if (vinsParameters().estimateTd())
        para_td_[0] = td_;
}

void StateManager::applyPosYawAlignment(const Vector3d &origin_P0, const Vector3d &origin_R0_ypr)
{
    Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_pose_[0][6],
                                                      para_pose_[0][3],
                                                      para_pose_[0][4],
                                                      para_pose_[0][5])
                                             .toRotationMatrix());
    double y_diff = origin_R0_ypr.x() - origin_R00.x();
    Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
    if (abs(abs(origin_R0_ypr.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0)
    {
        LOG_TXT_LEVEL(logging::ValueLogger::Level::INFO, "euler singular point!");
        rot_diff = Rs_[0] * Quaterniond(para_pose_[0][6],
                                       para_pose_[0][3],
                                       para_pose_[0][4],
                                       para_pose_[0][5])
                               .toRotationMatrix()
                               .transpose();
    }

    for (int i = 0; i <= window_size_; i++)
    {
        Rs_[i] = rot_diff * Quaterniond(para_pose_[i][6], para_pose_[i][3], para_pose_[i][4], para_pose_[i][5])
                              .normalized()
                              .toRotationMatrix();

        Ps_[i] = rot_diff * Vector3d(para_pose_[i][0] - para_pose_[0][0],
                                  para_pose_[i][1] - para_pose_[0][1],
                                  para_pose_[i][2] - para_pose_[0][2]) +
                origin_P0;

        Vs_[i] = rot_diff * Vector3d(para_speed_bias_[i][0],
                                    para_speed_bias_[i][1],
                                    para_speed_bias_[i][2]);

        Bas_[i] = Vector3d(para_speed_bias_[i][3],
                          para_speed_bias_[i][4],
                          para_speed_bias_[i][5]);

        Bgs_[i] = Vector3d(para_speed_bias_[i][6],
                          para_speed_bias_[i][7],
                          para_speed_bias_[i][8]);
    }
}

void StateManager::syncFromParameters(const SyncFromOptions &opts)
{
    Vector3d origin_R0 = Utility::R2ypr(Rs_[0]);
    Vector3d origin_P0 = Ps_[0];

    if (opts.align_origin_P0.has_value())
        origin_P0 = *opts.align_origin_P0;
    if (opts.align_origin_R0.has_value())
        origin_R0 = Utility::R2ypr(*opts.align_origin_R0);

    applyPosYawAlignment(origin_P0, origin_R0);

    for (int i = 0; i < static_cast<int>(para_ex_pose_.size()); i++)
    {
        tic_[i] = Vector3d(para_ex_pose_[i][0],
                          para_ex_pose_[i][1],
                          para_ex_pose_[i][2]);
        ric_[i] = Quaterniond(para_ex_pose_[i][6],
                             para_ex_pose_[i][3],
                             para_ex_pose_[i][4],
                             para_ex_pose_[i][5])
                       .toRotationMatrix();
    }

    if (vinsParameters().estimateTd())
        td_ = para_td_[0];
}

double *StateManager::poseParameter(int slot) { return para_pose_[slot].data(); }
double *StateManager::speedBiasParameter(int slot) { return para_speed_bias_[slot].data(); }
double *StateManager::extrinsicParameter(int cam) { return para_ex_pose_[cam].data(); }
double *StateManager::featureParameter(int feature_idx) { return para_feature_[feature_idx].data(); }
double *StateManager::timeDelayParameter() { return para_td_.data(); }

void StateManager::clearMarginalizationPrior()
{
    if (last_marginalization_info_ != nullptr)
        delete last_marginalization_info_;
    last_marginalization_info_ = nullptr;
    last_marginalization_parameter_blocks_.clear();
}

void StateManager::slideWindowOld()
{
    snapshotOldestFrame();
    if (slot_count_ != window_size_)
        return;

    unregisterSlot(0);

    for (int i = 0; i < window_size_; i++)
    {
        Rs_[i].swap(Rs_[i + 1]);
        std::swap(imu_data_[i].pre_integration, imu_data_[i + 1].pre_integration);
        imu_data_[i].dt_buf.swap(imu_data_[i + 1].dt_buf);
        imu_data_[i].linear_acceleration_buf.swap(imu_data_[i + 1].linear_acceleration_buf);
        imu_data_[i].angular_velocity_buf.swap(imu_data_[i + 1].angular_velocity_buf);

        Headers_[i] = Headers_[i + 1];
        frame_ids_[i] = frame_ids_[i + 1];
        Ps_[i].swap(Ps_[i + 1]);
        Vs_[i].swap(Vs_[i + 1]);
        Bas_[i].swap(Bas_[i + 1]);
        Bgs_[i].swap(Bgs_[i + 1]);
    }

    Headers_[window_size_] = Headers_[window_size_ - 1];
    frame_ids_[window_size_] = frame_ids_[window_size_ - 1];
    Ps_[window_size_] = Ps_[window_size_ - 1];
    Vs_[window_size_] = Vs_[window_size_ - 1];
    Rs_[window_size_] = Rs_[window_size_ - 1];
    Bas_[window_size_] = Bas_[window_size_ - 1];
    Bgs_[window_size_] = Bgs_[window_size_ - 1];

    rebuildIdToSlot();
}

void StateManager::slideWindowNew()
{
    if (slot_count_ != window_size_)
        return;

    const int fc = window_size_;
    for (unsigned int i = 0; i < imu_data_[fc].dt_buf.size(); i++)
    {
        const double tmp_dt = imu_data_[fc].dt_buf[i];
        const Vector3d tmp_linear_acceleration = imu_data_[fc].linear_acceleration_buf[i];
        const Vector3d tmp_angular_velocity = imu_data_[fc].angular_velocity_buf[i];

        imu_data_[fc - 1].pre_integration->process(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

        imu_data_[fc - 1].dt_buf.push_back(tmp_dt);
        imu_data_[fc - 1].linear_acceleration_buf.push_back(tmp_linear_acceleration);
        imu_data_[fc - 1].angular_velocity_buf.push_back(tmp_angular_velocity);
    }

    Headers_[fc - 1] = Headers_[fc];
    frame_ids_[fc - 1] = frame_ids_[fc];
    Ps_[fc - 1] = Ps_[fc];
    Vs_[fc - 1] = Vs_[fc];
    Rs_[fc - 1] = Rs_[fc];
    Bas_[fc - 1] = Bas_[fc];
    Bgs_[fc - 1] = Bgs_[fc];

    if (frame_ids_[fc] != kInvalidFrameId)
    {
        id_to_slot_.erase(frame_ids_[fc]);
        id_to_slot_[frame_ids_[fc - 1]] = fc - 1;
    }
    frame_ids_[fc] = kInvalidFrameId;
}

void StateManager::slideWindow(SlideMode mode, const Vector3d &acc_0, const Vector3d &gyr_0,
                               const Vector3d &Ba_end, const Vector3d &Bg_end)
{
    if (mode == SlideMode::MARGIN_OLD)
    {
        slideWindowOld();
        if (slot_count_ == window_size_)
        {
            imu_data_[window_size_].pre_integration =
                std::make_shared<Integrator>(acc_0, gyr_0, Ba_end, Bg_end);
            clearImuBufferAtSlot(window_size_);
        }
    }
    else
    {
        slideWindowNew();
        if (slot_count_ == window_size_)
        {
            imu_data_[window_size_].pre_integration =
                std::make_shared<Integrator>(acc_0, gyr_0, Ba_end, Bg_end);
            clearImuBufferAtSlot(window_size_);
        }
    }
}

Vector3d &StateManager::positionAtSlot(int slot) { return Ps_[slot]; }
Vector3d &StateManager::velocityAtSlot(int slot) { return Vs_[slot]; }
Matrix3d &StateManager::rotationAtSlot(int slot) { return Rs_[slot]; }
Vector3d &StateManager::accBiasAtSlot(int slot) { return Bas_[slot]; }
Vector3d &StateManager::gyrBiasAtSlot(int slot) { return Bgs_[slot]; }
const Vector3d &StateManager::positionAtSlot(int slot) const { return Ps_[slot]; }
const Matrix3d &StateManager::rotationAtSlot(int slot) const { return Rs_[slot]; }

ParameterBlocks StateManager::parameterBlocks()
{
    ParameterBlocks blocks;
    blocks.pose = para_pose_.empty() ? nullptr
                                     : reinterpret_cast<double (*)[SIZE_POSE]>(para_pose_.data());
    blocks.speed_bias = para_speed_bias_.empty()
                            ? nullptr
                            : reinterpret_cast<double (*)[SIZE_SPEEDBIAS]>(para_speed_bias_.data());
    blocks.ex_pose = para_ex_pose_.empty() ? nullptr
                                           : reinterpret_cast<double (*)[SIZE_POSE]>(para_ex_pose_.data());
    blocks.feature = para_feature_.empty() ? nullptr
                                           : reinterpret_cast<double (*)[SIZE_FEATURE]>(para_feature_.data());
    blocks.td = para_td_.data();
    return blocks;
}

ExtrinsicState StateManager::extrinsic(int cam_id) const
{
    ExtrinsicState ex;
    ex.ric = ric_[cam_id];
    ex.tic = tic_[cam_id];
    return ex;
}
