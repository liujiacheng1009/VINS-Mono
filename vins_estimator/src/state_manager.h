#pragma once

#include "parameters.h"
#include "utility/simple_types.h"
#include "factor/imu_factor.h"
#include "factor/marginalization_factor.h"

#include <array>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

using namespace Eigen;

struct SyncFromOptions
{
    std::optional<Vector3d> align_origin_P0;
    std::optional<Matrix3d> align_origin_R0;
};

struct FrameImuData
{
    std::shared_ptr<Integrator> pre_integration;
    std::vector<double> dt_buf;
    std::vector<Vector3d> linear_acceleration_buf;
    std::vector<Vector3d> angular_velocity_buf;
};

struct ExtrinsicState
{
    Matrix3d ric = Matrix3d::Identity();
    Vector3d tic = Vector3d::Zero();
};

struct ParameterBlocks
{
    double (*pose)[SIZE_POSE] = nullptr;
    double (*speed_bias)[SIZE_SPEEDBIAS] = nullptr;
    double (*ex_pose)[SIZE_POSE] = nullptr;
    double (*feature)[SIZE_FEATURE] = nullptr;
    double *td = nullptr;
};

class StateManager
{
  public:
    StateManager();

    void clear();
    void initFromConfig();

    int slotCount() const { return slot_count_; }
    void setSlotCount(int count) { slot_count_ = count; }

    int windowSize() const { return window_size_; }
    int latestSlot() const;

    void bindFrame(int slot, FrameId id, const SimpleHeader &header);

    bool contains(FrameId id) const;
    std::optional<int> slotOf(FrameId id) const;

    Vector3d &positionAtSlot(int slot);
    Vector3d &velocityAtSlot(int slot);
    Matrix3d &rotationAtSlot(int slot);
    Vector3d &accBiasAtSlot(int slot);
    Vector3d &gyrBiasAtSlot(int slot);
    const Vector3d &positionAtSlot(int slot) const;
    const Matrix3d &rotationAtSlot(int slot) const;

    void snapshotOldestFrame();
    void updateKeyframeSnapshot();

    Matrix3d backRotation() const { return back_R0_; }
    Vector3d backPosition() const { return back_P0_; }
    Matrix3d lastRotation() const { return last_R_; }
    Vector3d lastPosition() const { return last_P_; }
    Matrix3d lastRotation0() const { return last_R0_; }
    Vector3d lastPosition0() const { return last_P0_; }

    enum class SlideMode
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };
    void slideWindowOld();
    void slideWindowNew();
    void slideWindow(SlideMode mode, const Vector3d &acc_0, const Vector3d &gyr_0,
                     const Vector3d &Ba_end, const Vector3d &Bg_end);

    void propagateImuAtSlot(int slot, double dt, const Vector3d &acc, const Vector3d &gyr,
                            const Vector3d &acc_prev, const Vector3d &gyr_prev, const Vector3d &gravity);

    void syncToParameters();
    void syncFromParameters(const SyncFromOptions &opts = {});
    ParameterBlocks parameterBlocks();

    double *poseParameter(int slot);
    double *speedBiasParameter(int slot);
    double *extrinsicParameter(int cam);
    double *featureParameter(int feature_idx);
    double *timeDelayParameter();

    ExtrinsicState extrinsic(int cam_id) const;

    double timeDelay() const { return td_; }
    void setTimeDelay(double td) { td_ = td; }

    MarginalizationInfo *&lastMarginalizationInfo() { return last_marginalization_info_; }
    std::vector<double *> &lastMarginalizationParameterBlocks() { return last_marginalization_parameter_blocks_; }
    void clearMarginalizationPrior();

    std::shared_ptr<Integrator> &preIntegrationAtSlot(int slot);
    void pushImuSampleAtSlot(int slot, double dt, const Vector3d &acc, const Vector3d &gyr);
    void clearImuBufferAtSlot(int slot);

    std::shared_ptr<Integrator> &tmpPreIntegration() { return tmp_pre_integration_; }

    void initializeFrameAtSlot(int slot, FrameId id, const SimpleHeader &header,
                               const Vector3d &P, const Matrix3d &R, const Vector3d &V,
                               const Vector3d &Ba, const Vector3d &Bg,
                               const Vector3d &acc_0, const Vector3d &gyr_0);

  private:
    int activeSlotLimit() const;

    void allocateStorage();
    void rebuildIdToSlot();
    void unregisterSlot(int slot);
    /** Write para_* back to Ps_/Rs_/Vs_/bias; anchor slot-0 position and yaw at origin_P0 / origin_R0_ypr. */
    void applyPosYawAlignment(const Vector3d &origin_P0, const Vector3d &origin_R0_ypr);

    int window_size_ = 10;
    int slot_count_ = 0;
    std::vector<FrameId> frame_ids_;
    std::unordered_map<FrameId, int> id_to_slot_;

    std::vector<Vector3d> Ps_;
    std::vector<Vector3d> Vs_;
    std::vector<Matrix3d> Rs_;
    std::vector<Vector3d> Bas_;
    std::vector<Vector3d> Bgs_;
    std::vector<SimpleHeader> Headers_;

    std::vector<Matrix3d> ric_;
    std::vector<Vector3d> tic_;
    double td_ = 0.0;

    std::vector<std::array<double, SIZE_POSE>> para_pose_;
    std::vector<std::array<double, SIZE_SPEEDBIAS>> para_speed_bias_;
    std::vector<std::array<double, SIZE_FEATURE>> para_feature_;
    std::vector<std::array<double, SIZE_POSE>> para_ex_pose_;
    std::array<double, 1> para_td_{0.0};

    std::shared_ptr<Integrator> tmp_pre_integration_;

    Matrix3d back_R0_;
    Vector3d back_P0_;
    Matrix3d last_R_;
    Vector3d last_P_;
    Matrix3d last_R0_;
    Vector3d last_P0_;

    std::vector<FrameImuData> imu_data_;

    MarginalizationInfo *last_marginalization_info_ = nullptr;
    std::vector<double *> last_marginalization_parameter_blocks_;
};
