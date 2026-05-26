// Verifies IMU–cam1 extrinsics composed from cam_chain-imucam (cam0 T_cam_imu)
// and cam_chain.yaml (T_cn_cnm1) match independent 4x4 composition.

#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include <eigen3/Eigen/Dense>

#include "cam_chain.h"

#ifndef VINS_TEST_SIM_CONFIG_DIR
#error "VINS_TEST_SIM_CONFIG_DIR must be defined by CMake"
#endif

namespace {

std::string simConfigPath(const char *filename)
{
    return std::string(VINS_TEST_SIM_CONFIG_DIR) + "/" + filename;
}

void normalizeRotation(Eigen::Matrix3d &R)
{
    Eigen::Quaterniond Q(R);
    R = Q.normalized().toRotationMatrix();
}

Eigen::Matrix4d TFromRicTic(const Eigen::Matrix3d &R_ic, const Eigen::Vector3d &t_ic)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R_ic;
    T.block<3, 1>(0, 3) = t_ic;
    return T;
}

void expectMatrixNear(const Eigen::Matrix3d &actual, const Eigen::Matrix3d &expected, double tol)
{
    EXPECT_TRUE(actual.isApprox(expected, tol)) << "actual:\n" << actual << "\nexpected:\n" << expected;
}

void expectVectorNear(const Eigen::Vector3d &actual, const Eigen::Vector3d &expected, double tol)
{
    EXPECT_TRUE(actual.isApprox(expected, tol)) << "actual: " << actual.transpose()
                                              << " expected: " << expected.transpose();
}

} // namespace

class CamChainExtrinsicTest : public ::testing::Test
{
  protected:
    void SetUp() override
    {
        imu_path_ = simConfigPath("cam_chain-imucam.yaml");
        stereo_path_ = simConfigPath("cam_chain.yaml");
        ASSERT_TRUE(loadCamChainYaml(imu_path_, imu_chain_));
        ASSERT_TRUE(loadCamChainYaml(stereo_path_, stereo_chain_));
        ASSERT_TRUE(imu_chain_.cameras.count(0) > 0);
        ASSERT_TRUE(imu_chain_.cameras.at(0).has_T_cam_imu);
        T_cam_imu_cam0_ = imu_chain_.cameras.at(0).T_cam_imu;
        ASSERT_TRUE(stereo_chain_.cameras.count(1) > 0);
        ASSERT_TRUE(stereo_chain_.cameras.at(1).has_T_cn_cnm1);
    }

    std::string imu_path_;
    std::string stereo_path_;
    CamChainFile imu_chain_;
    CamChainFile stereo_chain_;
    Eigen::Matrix4d T_cam_imu_cam0_;
};

TEST_F(CamChainExtrinsicTest, YamlFilesLoad)
{
    EXPECT_FALSE(imu_chain_.cameras.empty());
    EXPECT_TRUE(stereo_chain_.cameras.count(1) > 0);
}

TEST_F(CamChainExtrinsicTest, Cam0ImuMatchesLegacySimulationExtrinsic)
{
    Eigen::Matrix3d R_ic;
    Eigen::Vector3d t_ic;
    ASSERT_TRUE(imuExtrinsicForCamera(0, T_cam_imu_cam0_, stereo_chain_, R_ic, t_ic));

    const Eigen::Matrix3d R_legacy =
        (Eigen::Matrix3d() << 0, 0, -1, -1, 0, 0, 0, 1, 0).finished();
    const Eigen::Vector3d t_legacy(0.0, 0.2, 0.6);

    expectMatrixNear(R_ic, R_legacy, 1e-9);
    expectVectorNear(t_ic, t_legacy, 1e-9);
}

TEST_F(CamChainExtrinsicTest, Cam1ImuMatchesComposedTransform)
{
    Eigen::Matrix3d R_ic1;
    Eigen::Vector3d t_ic1;
    ASSERT_TRUE(imuExtrinsicForCamera(1, T_cam_imu_cam0_, stereo_chain_, R_ic1, t_ic1));

    const Eigen::Matrix4d T_imu_c0 = T_cam_imu_cam0_.inverse();
    const Eigen::Matrix4d &T_c1_c0 = stereo_chain_.cameras.at(1).T_cn_cnm1;
    const Eigen::Matrix4d T_imu_c1 = T_imu_c0 * T_c1_c0.inverse();

    Eigen::Matrix3d R_ref = T_imu_c1.block<3, 3>(0, 0);
    Eigen::Vector3d t_ref = T_imu_c1.block<3, 1>(0, 3);
    normalizeRotation(R_ref);
    normalizeRotation(R_ic1);

    expectMatrixNear(R_ic1, R_ref, 1e-9);
    expectVectorNear(t_ic1, t_ref, 1e-9);
}

TEST_F(CamChainExtrinsicTest, Cam1TranslationReflectsSixCmBaselineAlongCam0X)
{
    Eigen::Matrix3d R_ic0, R_ic1;
    Eigen::Vector3d t_ic0, t_ic1;
    ASSERT_TRUE(imuExtrinsicForCamera(0, T_cam_imu_cam0_, stereo_chain_, R_ic0, t_ic0));
    ASSERT_TRUE(imuExtrinsicForCamera(1, T_cam_imu_cam0_, stereo_chain_, R_ic1, t_ic1));

    expectMatrixNear(R_ic1, R_ic0, 1e-9);

  // T_cn_cnm1: cam1 origin is +6 cm along cam0 X; with aligned axes and fixed R_ic, t shifts in IMU Y.
    const Eigen::Vector3d t_expected(0.0, 0.26, 0.6);
    expectVectorNear(t_ic1, t_expected, 1e-9);
    EXPECT_NEAR(t_ic1.y() - t_ic0.y(), 0.06, 1e-9);
}

TEST_F(CamChainExtrinsicTest, ApplyCameraSetupStereoSlotsMatchPerCameraExtrinsics)
{
    const std::vector<int> camera_ids{0, 1};
    double focal = 0.0;
    double width = 0.0;
    double height = 0.0;
    std::vector<Eigen::Matrix3d> ric;
    std::vector<Eigen::Vector3d> tic;

    ASSERT_TRUE(applyCameraSetup(2, camera_ids, imu_chain_, stereo_chain_, focal, width, height, ric, tic));
    ASSERT_EQ(ric.size(), 2u);
    ASSERT_EQ(tic.size(), 2u);

    Eigen::Matrix3d R0, R1;
    Eigen::Vector3d t0, t1;
    ASSERT_TRUE(imuExtrinsicForCamera(0, T_cam_imu_cam0_, stereo_chain_, R0, t0));
    ASSERT_TRUE(imuExtrinsicForCamera(1, T_cam_imu_cam0_, stereo_chain_, R1, t1));

    expectMatrixNear(ric[0], R0, 1e-9);
    expectVectorNear(tic[0], t0, 1e-9);
    expectMatrixNear(ric[1], R1, 1e-9);
    expectVectorNear(tic[1], t1, 1e-9);
}

TEST_F(CamChainExtrinsicTest, MonoCam1SlotUsesDerivedImuExtrinsic)
{
    const std::vector<int> camera_ids{1};
    double focal = 0.0;
    double width = 0.0;
    double height = 0.0;
    std::vector<Eigen::Matrix3d> ric;
    std::vector<Eigen::Vector3d> tic;

    ASSERT_TRUE(applyCameraSetup(1, camera_ids, imu_chain_, stereo_chain_, focal, width, height, ric, tic));
    ASSERT_EQ(ric.size(), 1u);

    Eigen::Matrix3d R1;
    Eigen::Vector3d t1;
    ASSERT_TRUE(imuExtrinsicForCamera(1, T_cam_imu_cam0_, stereo_chain_, R1, t1));

    expectMatrixNear(ric[0], R1, 1e-9);
    expectVectorNear(tic[0], t1, 1e-9);
    expectVectorNear(tic[0], Eigen::Vector3d(0.0, 0.26, 0.6), 1e-9);
}
