#pragma once

#include "parameters.h"
#include "utility/simple_types.h"
#include "factor/imu_factor.h"
#include "factor/marginalization_factor.h"

#include <eigen3/Eigen/Dense>
#include <memory>
#include <optional>
#include <stdexcept>
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

struct FrameState
{
    FrameId id = kInvalidFrameId;
    Vector3d P = Vector3d::Zero();
    Vector3d V = Vector3d::Zero();
    Matrix3d R = Matrix3d::Identity();
    Vector3d Ba = Vector3d::Zero();
    Vector3d Bg = Vector3d::Zero();
    SimpleHeader header;
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

    void copyExtrinsicRotations(Matrix3d ric_out[NUM_OF_CAM]) const;

    void clear();
    void initFromConfig();

    int slotCount() const { return slot_count_; }
    void setSlotCount(int count) { slot_count_ = count; }
    int activeFrameCount() const;

    void bindFrame(int slot, FrameId id, const SimpleHeader &header);

    bool contains(FrameId id) const;
    std::optional<int> slotOf(FrameId id) const;
    std::optional<FrameId> frameIdAtStamp(double stamp_sec) const;

    FrameId latestFrameId() const;
    FrameId oldestFrameId() const;

    FrameState frameAtSlot(int slot) const;
    void setFrameAtSlot(int slot, const FrameState &frame);

    FrameState frameState(FrameId id) const;
    FrameImuData &imuData(FrameId id);
    const FrameImuData &imuData(FrameId id) const;

    Vector3d &positionAtSlot(int slot);
    Vector3d &velocityAtSlot(int slot);
    Matrix3d &rotationAtSlot(int slot);
    Vector3d &accBiasAtSlot(int slot);
    Vector3d &gyrBiasAtSlot(int slot);
    const Vector3d &positionAtSlot(int slot) const;
    const Matrix3d &rotationAtSlot(int slot) const;

    Vector3d &position(FrameId id);
    Matrix3d &rotation(FrameId id);
    Vector3d &velocity(FrameId id);
    Vector3d &accBias(FrameId id);
    Vector3d &gyrBias(FrameId id);

    std::vector<Vector3d> collectKeyframePositions() const;

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
    void propagateImu(FrameId id, double dt, const Vector3d &acc, const Vector3d &gyr,
                    const Vector3d &acc_prev, const Vector3d &gyr_prev, const Vector3d &gravity);

    void syncToParameters();
    void syncFromParameters(const SyncFromOptions &opts = {});
    ParameterBlocks parameterBlocks();

    double *poseParameter(int slot);
    double *speedBiasParameter(int slot);
    double *extrinsicParameter(int cam);
    double *featureParameter(int feature_idx);
    double *timeDelayParameter();

    double *poseParameter(FrameId id);
    double *speedBiasParameter(FrameId id);

    ExtrinsicState extrinsic(int cam_id) const;
    void setExtrinsic(int cam_id, const Matrix3d &ric, const Vector3d &tic);
    Matrix3d cameraRotation(int cam_id) const { return ric_[cam_id]; }
    Vector3d cameraTranslation(int cam_id) const { return tic_[cam_id]; }

    double timeDelay() const { return td_; }
    void setTimeDelay(double td) { td_ = td; }

    MarginalizationInfo *&lastMarginalizationInfo() { return last_marginalization_info_; }
    std::vector<double *> &lastMarginalizationParameterBlocks() { return last_marginalization_parameter_blocks_; }
    void clearMarginalizationPrior();

    std::shared_ptr<Integrator> &preIntegrationAtSlot(int slot);
    std::shared_ptr<Integrator> &preIntegration(FrameId id);
    FrameImuData &imuDataAtSlot(int slot);
    void pushImuSampleAtSlot(int slot, double dt, const Vector3d &acc, const Vector3d &gyr);
    void pushImuSample(FrameId id, double dt, const Vector3d &acc, const Vector3d &gyr);
    void clearImuBufferAtSlot(int slot);
    void mergeImuBuffer(FrameId from_id, FrameId to_id);

    std::shared_ptr<Integrator> &tmpPreIntegration() { return tmp_pre_integration_; }

    void initializeFrameAtSlot(int slot, FrameId id, const SimpleHeader &header,
                               const Vector3d &P, const Matrix3d &R, const Vector3d &V,
                               const Vector3d &Ba, const Vector3d &Bg,
                               const Vector3d &acc_0, const Vector3d &gyr_0);

  private:
    int requireSlot(FrameId id) const;

    void rebuildIdToSlot();
    void unregisterSlot(int slot);
    void applyYawAlignment(const Vector3d &origin_P0, const Vector3d &origin_R0_ypr);

    int slot_count_ = 0;
    FrameId frame_ids_[WINDOW_SIZE + 1];
    std::unordered_map<FrameId, int> id_to_slot_;

    Vector3d Ps_[WINDOW_SIZE + 1];
    Vector3d Vs_[WINDOW_SIZE + 1];
    Matrix3d Rs_[WINDOW_SIZE + 1];
    Vector3d Bas_[WINDOW_SIZE + 1];
    Vector3d Bgs_[WINDOW_SIZE + 1];
    SimpleHeader Headers_[WINDOW_SIZE + 1];

    Matrix3d ric_[NUM_OF_CAM];
    Vector3d tic_[NUM_OF_CAM];
    double td_ = 0.0;

    double para_Pose_[WINDOW_SIZE + 1][SIZE_POSE];
    double para_SpeedBias_[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
    double para_Feature_[NUM_OF_F][SIZE_FEATURE];
    double para_Ex_Pose_[NUM_OF_CAM][SIZE_POSE];
    double para_Td_[1][1];

    std::shared_ptr<Integrator> tmp_pre_integration_;

    Matrix3d back_R0_;
    Vector3d back_P0_;
    Matrix3d last_R_;
    Vector3d last_P_;
    Matrix3d last_R0_;
    Vector3d last_P0_;

    FrameImuData imu_data_[WINDOW_SIZE + 1];

    MarginalizationInfo *last_marginalization_info_ = nullptr;
    std::vector<double *> last_marginalization_parameter_blocks_;
};
