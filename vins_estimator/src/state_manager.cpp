#include "state_manager.h"

#include "utility/utility.h"

StateManager::StateManager()
{
    clear();
}

void StateManager::clear()
{
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        Rs_[i].setIdentity();
        Ps_[i].setZero();
        Vs_[i].setZero();
        Bas_[i].setZero();
        Bgs_[i].setZero();
        Headers_[i] = SimpleHeader{};
        frame_ids_[i] = kInvalidFrameId;
        imu_data_[i].dt_buf.clear();
        imu_data_[i].linear_acceleration_buf.clear();
        imu_data_[i].angular_velocity_buf.clear();
        imu_data_[i].pre_integration.reset();
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic_[i].setZero();
        ric_[i].setIdentity();
    }

    slot_count_ = 0;
    id_to_slot_.clear();
    td_ = TD;
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
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic_[i] = TIC[i];
        ric_[i] = RIC[i];
    }
    td_ = TD;
}

void StateManager::copyExtrinsicRotations(Matrix3d ric_out[NUM_OF_CAM]) const
{
    for (int i = 0; i < NUM_OF_CAM; i++)
        ric_out[i] = ric_[i];
}

void StateManager::rebuildIdToSlot()
{
    id_to_slot_.clear();
    const int limit = (slot_count_ >= WINDOW_SIZE) ? WINDOW_SIZE : slot_count_;
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

FrameState StateManager::frameAtSlot(int slot) const
{
    FrameState frame;
    frame.id = frame_ids_[slot];
    frame.P = Ps_[slot];
    frame.V = Vs_[slot];
    frame.R = Rs_[slot];
    frame.Ba = Bas_[slot];
    frame.Bg = Bgs_[slot];
    frame.header = Headers_[slot];
    return frame;
}

void StateManager::setFrameAtSlot(int slot, const FrameState &frame)
{
    bindFrame(slot, frame.id, frame.header);
    Ps_[slot] = frame.P;
    Vs_[slot] = frame.V;
    Rs_[slot] = frame.R;
    Bas_[slot] = frame.Ba;
    Bgs_[slot] = frame.Bg;
}

std::vector<Vector3d> StateManager::collectKeyframePositions() const
{
    std::vector<Vector3d> poses;
    const int limit = (slot_count_ >= WINDOW_SIZE) ? WINDOW_SIZE : slot_count_;
    poses.reserve(static_cast<size_t>(limit + 1));
    for (int i = 0; i <= limit; i++)
        poses.push_back(Ps_[i]);
    return poses;
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

std::optional<FrameId> StateManager::frameIdAtStamp(double stamp_sec) const
{
    const int limit = (slot_count_ >= WINDOW_SIZE) ? WINDOW_SIZE : slot_count_;
    for (int i = 0; i <= limit; i++)
    {
        if (std::abs(Headers_[i].stamp.toSec() - stamp_sec) < 1e-9)
            return frame_ids_[i];
    }
    return std::nullopt;
}

FrameId StateManager::latestFrameId() const
{
    const int slot = std::min(slot_count_, WINDOW_SIZE);
    return frame_ids_[slot];
}

FrameId StateManager::oldestFrameId() const
{
    return frame_ids_[0];
}

void StateManager::snapshotOldestFrame()
{
    back_R0_ = Rs_[0];
    back_P0_ = Ps_[0];
}

void StateManager::updateKeyframeSnapshot()
{
    last_R_ = Rs_[WINDOW_SIZE];
    last_P_ = Ps_[WINDOW_SIZE];
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

FrameImuData &StateManager::imuDataAtSlot(int slot)
{
    return imu_data_[slot];
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
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Pose_[i][0] = Ps_[i].x();
        para_Pose_[i][1] = Ps_[i].y();
        para_Pose_[i][2] = Ps_[i].z();
        Quaterniond q{Rs_[i]};
        para_Pose_[i][3] = q.x();
        para_Pose_[i][4] = q.y();
        para_Pose_[i][5] = q.z();
        para_Pose_[i][6] = q.w();

        para_SpeedBias_[i][0] = Vs_[i].x();
        para_SpeedBias_[i][1] = Vs_[i].y();
        para_SpeedBias_[i][2] = Vs_[i].z();

        para_SpeedBias_[i][3] = Bas_[i].x();
        para_SpeedBias_[i][4] = Bas_[i].y();
        para_SpeedBias_[i][5] = Bas_[i].z();

        para_SpeedBias_[i][6] = Bgs_[i].x();
        para_SpeedBias_[i][7] = Bgs_[i].y();
        para_SpeedBias_[i][8] = Bgs_[i].z();
    }
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        para_Ex_Pose_[i][0] = tic_[i].x();
        para_Ex_Pose_[i][1] = tic_[i].y();
        para_Ex_Pose_[i][2] = tic_[i].z();
        Quaterniond q{ric_[i]};
        para_Ex_Pose_[i][3] = q.x();
        para_Ex_Pose_[i][4] = q.y();
        para_Ex_Pose_[i][5] = q.z();
        para_Ex_Pose_[i][6] = q.w();
    }
    if (ESTIMATE_TD)
        para_Td_[0][0] = td_;
}

void StateManager::applyYawAlignment(const Vector3d &origin_P0, const Vector3d &origin_R0_ypr)
{
    Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose_[0][6],
                                                      para_Pose_[0][3],
                                                      para_Pose_[0][4],
                                                      para_Pose_[0][5])
                                             .toRotationMatrix());
    double y_diff = origin_R0_ypr.x() - origin_R00.x();
    Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
    if (abs(abs(origin_R0_ypr.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0)
    {
        ROS_DEBUG("euler singular point!");
        rot_diff = Rs_[0] * Quaterniond(para_Pose_[0][6],
                                       para_Pose_[0][3],
                                       para_Pose_[0][4],
                                       para_Pose_[0][5])
                               .toRotationMatrix()
                               .transpose();
    }

    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        Rs_[i] = rot_diff * Quaterniond(para_Pose_[i][6], para_Pose_[i][3], para_Pose_[i][4], para_Pose_[i][5])
                              .normalized()
                              .toRotationMatrix();

        Ps_[i] = rot_diff * Vector3d(para_Pose_[i][0] - para_Pose_[0][0],
                                  para_Pose_[i][1] - para_Pose_[0][1],
                                  para_Pose_[i][2] - para_Pose_[0][2]) +
                origin_P0;

        Vs_[i] = rot_diff * Vector3d(para_SpeedBias_[i][0],
                                    para_SpeedBias_[i][1],
                                    para_SpeedBias_[i][2]);

        Bas_[i] = Vector3d(para_SpeedBias_[i][3],
                          para_SpeedBias_[i][4],
                          para_SpeedBias_[i][5]);

        Bgs_[i] = Vector3d(para_SpeedBias_[i][6],
                          para_SpeedBias_[i][7],
                          para_SpeedBias_[i][8]);
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

    applyYawAlignment(origin_P0, origin_R0);

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic_[i] = Vector3d(para_Ex_Pose_[i][0],
                          para_Ex_Pose_[i][1],
                          para_Ex_Pose_[i][2]);
        ric_[i] = Quaterniond(para_Ex_Pose_[i][6],
                             para_Ex_Pose_[i][3],
                             para_Ex_Pose_[i][4],
                             para_Ex_Pose_[i][5])
                       .toRotationMatrix();
    }

    if (ESTIMATE_TD)
        td_ = para_Td_[0][0];
}

double *StateManager::poseParameter(int slot) { return para_Pose_[slot]; }
double *StateManager::speedBiasParameter(int slot) { return para_SpeedBias_[slot]; }
double *StateManager::extrinsicParameter(int cam) { return para_Ex_Pose_[cam]; }
double *StateManager::featureParameter(int feature_idx) { return para_Feature_[feature_idx]; }
double *StateManager::timeDelayParameter() { return para_Td_[0]; }

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
    if (slot_count_ != WINDOW_SIZE)
        return;

    unregisterSlot(0);

    for (int i = 0; i < WINDOW_SIZE; i++)
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

    Headers_[WINDOW_SIZE] = Headers_[WINDOW_SIZE - 1];
    frame_ids_[WINDOW_SIZE] = frame_ids_[WINDOW_SIZE - 1];
    Ps_[WINDOW_SIZE] = Ps_[WINDOW_SIZE - 1];
    Vs_[WINDOW_SIZE] = Vs_[WINDOW_SIZE - 1];
    Rs_[WINDOW_SIZE] = Rs_[WINDOW_SIZE - 1];
    Bas_[WINDOW_SIZE] = Bas_[WINDOW_SIZE - 1];
    Bgs_[WINDOW_SIZE] = Bgs_[WINDOW_SIZE - 1];

    rebuildIdToSlot();
}

void StateManager::slideWindowNew()
{
    if (slot_count_ != WINDOW_SIZE)
        return;

    const int fc = WINDOW_SIZE;
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
        if (slot_count_ == WINDOW_SIZE)
        {
            imu_data_[WINDOW_SIZE].pre_integration =
                std::make_shared<Integrator>(acc_0, gyr_0, Ba_end, Bg_end);
            clearImuBufferAtSlot(WINDOW_SIZE);
        }
    }
    else
    {
        slideWindowNew();
        if (slot_count_ == WINDOW_SIZE)
        {
            imu_data_[WINDOW_SIZE].pre_integration =
                std::make_shared<Integrator>(acc_0, gyr_0, Ba_end, Bg_end);
            clearImuBufferAtSlot(WINDOW_SIZE);
        }
    }
}

int StateManager::activeFrameCount() const
{
    return (slot_count_ >= WINDOW_SIZE) ? (WINDOW_SIZE + 1) : (slot_count_ + 1);
}

int StateManager::requireSlot(FrameId id) const
{
    const auto slot = slotOf(id);
    if (!slot.has_value())
        throw std::out_of_range("FrameId not in active window");
    return *slot;
}

FrameState StateManager::frameState(FrameId id) const
{
    const int slot = requireSlot(id);
    return frameAtSlot(slot);
}

FrameImuData &StateManager::imuData(FrameId id)
{
    return imu_data_[requireSlot(id)];
}

const FrameImuData &StateManager::imuData(FrameId id) const
{
    return imu_data_[requireSlot(id)];
}

Vector3d &StateManager::positionAtSlot(int slot) { return Ps_[slot]; }
Vector3d &StateManager::velocityAtSlot(int slot) { return Vs_[slot]; }
Matrix3d &StateManager::rotationAtSlot(int slot) { return Rs_[slot]; }
Vector3d &StateManager::accBiasAtSlot(int slot) { return Bas_[slot]; }
Vector3d &StateManager::gyrBiasAtSlot(int slot) { return Bgs_[slot]; }
const Vector3d &StateManager::positionAtSlot(int slot) const { return Ps_[slot]; }
const Matrix3d &StateManager::rotationAtSlot(int slot) const { return Rs_[slot]; }

Vector3d &StateManager::position(FrameId id) { return Ps_[requireSlot(id)]; }
Matrix3d &StateManager::rotation(FrameId id) { return Rs_[requireSlot(id)]; }
Vector3d &StateManager::velocity(FrameId id) { return Vs_[requireSlot(id)]; }
Vector3d &StateManager::accBias(FrameId id) { return Bas_[requireSlot(id)]; }
Vector3d &StateManager::gyrBias(FrameId id) { return Bgs_[requireSlot(id)]; }

double *StateManager::poseParameter(FrameId id) { return poseParameter(requireSlot(id)); }
double *StateManager::speedBiasParameter(FrameId id) { return speedBiasParameter(requireSlot(id)); }

std::shared_ptr<Integrator> &StateManager::preIntegration(FrameId id)
{
    return preIntegrationAtSlot(requireSlot(id));
}

void StateManager::pushImuSample(FrameId id, double dt, const Vector3d &acc, const Vector3d &gyr)
{
    pushImuSampleAtSlot(requireSlot(id), dt, acc, gyr);
}

void StateManager::mergeImuBuffer(FrameId from_id, FrameId to_id)
{
    const int from_slot = requireSlot(from_id);
    const int to_slot = requireSlot(to_id);
    for (unsigned int i = 0; i < imu_data_[from_slot].dt_buf.size(); i++)
    {
        const double tmp_dt = imu_data_[from_slot].dt_buf[i];
        const Vector3d tmp_acc = imu_data_[from_slot].linear_acceleration_buf[i];
        const Vector3d tmp_gyr = imu_data_[from_slot].angular_velocity_buf[i];
        imu_data_[to_slot].pre_integration->process(tmp_dt, tmp_acc, tmp_gyr);
        imu_data_[to_slot].dt_buf.push_back(tmp_dt);
        imu_data_[to_slot].linear_acceleration_buf.push_back(tmp_acc);
        imu_data_[to_slot].angular_velocity_buf.push_back(tmp_gyr);
    }
    clearImuBufferAtSlot(from_slot);
}

void StateManager::propagateImu(FrameId id, double dt, const Vector3d &acc, const Vector3d &gyr,
                                const Vector3d &acc_prev, const Vector3d &gyr_prev, const Vector3d &gravity)
{
    propagateImuAtSlot(requireSlot(id), dt, acc, gyr, acc_prev, gyr_prev, gravity);
}

ParameterBlocks StateManager::parameterBlocks()
{
    ParameterBlocks blocks;
    blocks.pose = para_Pose_;
    blocks.speed_bias = para_SpeedBias_;
    blocks.ex_pose = para_Ex_Pose_;
    blocks.feature = para_Feature_;
    blocks.td = para_Td_[0];
    return blocks;
}

ExtrinsicState StateManager::extrinsic(int cam_id) const
{
    ExtrinsicState ex;
    ex.ric = ric_[cam_id];
    ex.tic = tic_[cam_id];
    return ex;
}

void StateManager::setExtrinsic(int cam_id, const Matrix3d &ric, const Vector3d &tic)
{
    ric_[cam_id] = ric;
    tic_[cam_id] = tic;
}
