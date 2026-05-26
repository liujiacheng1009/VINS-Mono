// Unit tests for vins_estimator/src/factor/projection_factor.{h,cpp}.
//
// Coverage:
//   1. Constructor stores pts_i/pts_j and zeroes tangent_base in the default
//      pinhole formulation (UNIT_SPHERE_ERROR not defined).
//   2. Evaluate produces zero residual at a self-consistent state pair.
//   3. Evaluate respects a null jacobians pointer.
//   4. sqrt_info scaling is applied uniformly to residual and jacobians.
//   5. Analytical jacobians vs finite-difference (minimal 6-DoF tangent for
//      poses, 1-DoF for inverse depth).
//   6. Optional time-delay parameter block residual and td jacobian.

#include <gtest/gtest.h>

#include <array>
#include <random>

#include <eigen3/Eigen/Dense>

#include "factor/projection_factor.h"
#include "parameters.h"
#include "utility/utility.h"

namespace
{

void setDefaultVisualSqrtInfo()
{
    ProjectionFactor::sqrt_info = (FOCAL_LENGTH / 1.5) * Eigen::Matrix2d::Identity();
    auto &params = vinsParameters();
    params.setImageRow(480.0);
    params.setImageCol(640.0);
    params.setRollingShutterTr(0.01);
}

void packPose(double out[7], const Eigen::Vector3d &p, const Eigen::Quaterniond &q)
{
    out[0] = p.x();
    out[1] = p.y();
    out[2] = p.z();
    out[3] = q.x();
    out[4] = q.y();
    out[5] = q.z();
    out[6] = q.w();
}

// Construct a self-consistent {state, feature observation} setup. Given a 3D
// landmark expressed in camera-i coordinates, propagate it through the i->j
// transform and store the resulting normalized image-plane coordinate as the
// observation in frame j. ProjectionFactor::Evaluate() should then report a
// (near) zero residual.
struct EvaluationSetup
{
    Eigen::Vector3d Pi, Pj, tic;
    Eigen::Quaterniond Qi, Qj, qic;
    double inv_dep_i;
    Eigen::Vector3d pts_i, pts_j;

    std::array<double, 7> pose_i{};
    std::array<double, 7> pose_j{};
    std::array<double, 7> pose_ex{};
    std::array<double, 1> inv_depth{};

    void pack()
    {
        packPose(pose_i.data(), Pi, Qi);
        packPose(pose_j.data(), Pj, Qj);
        packPose(pose_ex.data(), tic, qic);
        inv_depth[0] = inv_dep_i;
    }

    std::array<double *, 4> paramArray()
    {
        return {pose_i.data(), pose_j.data(), pose_ex.data(), inv_depth.data()};
    }
};

struct TdEvaluationSetup : EvaluationSetup
{
    Eigen::Vector2d velocity_i;
    Eigen::Vector2d velocity_j;
    double td_i;
    double td_j;
    double row_i;
    double row_j;
    double td;

    std::array<double, 1> td_param{};

    void pack()
    {
        EvaluationSetup::pack();
        td_param[0] = td;
    }

    std::array<double *, 5> paramArray()
    {
        return {pose_i.data(), pose_j.data(), pose_ex.data(), inv_depth.data(), td_param.data()};
    }
};

EvaluationSetup buildConsistentEvaluation(unsigned seed, bool perturb)
{
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> u(-0.3, 0.3);

    EvaluationSetup s;
    s.Pi = Eigen::Vector3d(0.2 + u(rng), -0.1 + u(rng), 0.05 + u(rng));
    s.Qi = Eigen::Quaterniond(
        Eigen::AngleAxisd(0.25, Eigen::Vector3d(0.2, 0.5, 0.8).normalized()));

    s.Pj = s.Pi + Eigen::Vector3d(0.1 + u(rng), 0.05 + u(rng), -0.02 + u(rng));
    // Build Qj via composition with deltaQ and immediately normalize: the
    // analytical jacobians in ProjectionFactor assume the input quaternions
    // are unit-norm (which is what PoseLocalParameterization::Plus enforces
    // in production). Skipping the normalize() here would silently break
    // the analytical-vs-numerical jacobian check by ~0.5%.
    s.Qj = (s.Qi * Utility::deltaQ(Eigen::Vector3d(0.05, -0.03, 0.04))).normalized();

    s.tic = Eigen::Vector3d(0.02, -0.01, 0.005);
    s.qic = Eigen::Quaterniond(
        Eigen::AngleAxisd(0.1, Eigen::Vector3d(0.0, 0.0, 1.0)));

    // Feature in normalized image-plane coordinates of frame i. Choosing
    // inv_dep_i ~ 1/2m places the landmark 2 m in front of camera i.
    s.pts_i = Eigen::Vector3d(0.15 + u(rng) * 0.1, -0.10 + u(rng) * 0.1, 1.0);
    s.inv_dep_i = 0.5;

    const Eigen::Vector3d pts_camera_i = s.pts_i / s.inv_dep_i;
    const Eigen::Vector3d pts_imu_i    = s.qic * pts_camera_i + s.tic;
    const Eigen::Vector3d pts_w        = s.Qi * pts_imu_i + s.Pi;
    const Eigen::Vector3d pts_imu_j    = s.Qj.inverse() * (pts_w - s.Pj);
    const Eigen::Vector3d pts_camera_j = s.qic.inverse() * (pts_imu_j - s.tic);

    // The observation pts_j is the projection of the same landmark in frame
    // j: dividing by the z component yields normalized image-plane coords
    // (x/z, y/z, 1) that match ProjectionFactor's expected layout.
    s.pts_j = pts_camera_j / pts_camera_j.z();

    if (perturb)
    {
        s.Pj += Eigen::Vector3d(0.01, -0.005, 0.003);
        s.Qj  = (s.Qj * Utility::deltaQ(Eigen::Vector3d(0.005, -0.003, 0.004))).normalized();
        s.pts_j += Eigen::Vector3d(0.002, -0.0015, 0.0);  // small obs noise
    }

    s.pack();
    return s;
}

TdEvaluationSetup buildConsistentTdEvaluation(unsigned seed, bool perturb)
{
    EvaluationSetup base = buildConsistentEvaluation(seed, /*perturb=*/false);

    TdEvaluationSetup s;
    static_cast<EvaluationSetup &>(s) = base;

    s.velocity_i = Eigen::Vector2d(0.015, -0.008);
    s.velocity_j = Eigen::Vector2d(-0.011, 0.006);
    s.td_i = -0.004;
    s.td_j = 0.003;
    const double image_row = vinsParameters().imageRow();
    const double rolling_tr = vinsParameters().rollingShutterTr();
    s.row_i = image_row * 0.25;
    s.row_j = image_row * 0.75;
    s.td = 0.012;

    const double row_i_centered = s.row_i - image_row / 2.0;
    const double row_j_centered = s.row_j - image_row / 2.0;
    const Eigen::Vector3d velocity_i_3d(s.velocity_i.x(), s.velocity_i.y(), 0.0);
    const Eigen::Vector3d velocity_j_3d(s.velocity_j.x(), s.velocity_j.y(), 0.0);

    // Store the raw observations such that ProjectionFactor's time-delay
    // correction recovers the self-consistent rays from buildConsistentEvaluation().
    s.pts_i = base.pts_i + (s.td - s.td_i + rolling_tr / image_row * row_i_centered) * velocity_i_3d;
    s.pts_j = base.pts_j + (s.td - s.td_j + rolling_tr / image_row * row_j_centered) * velocity_j_3d;

    if (perturb)
    {
        s.Pj += Eigen::Vector3d(0.01, -0.005, 0.003);
        s.Qj = (s.Qj * Utility::deltaQ(Eigen::Vector3d(0.005, -0.003, 0.004))).normalized();
        s.pts_j += Eigen::Vector3d(0.002, -0.0015, 0.0);
    }

    s.pack();
    return s;
}

Eigen::Vector2d residualOnly(ProjectionFactor *factor, double *const *params)
{
    Eigen::Vector2d r;
    factor->Evaluate(params, r.data(), nullptr);
    return r;
}

class ProjectionFactorTest : public ::testing::Test
{
  protected:
    void SetUp() override { setDefaultVisualSqrtInfo(); }
};

}  // namespace

TEST_F(ProjectionFactorTest, ConstructorStoresInputsAndZeroesTangentBase)
{
    const Eigen::Vector3d pts_i(0.1, -0.2, 1.0);
    const Eigen::Vector3d pts_j(0.05, 0.15, 1.0);
    ProjectionFactor factor(pts_i, pts_j);

    EXPECT_TRUE(factor.pts_i.isApprox(pts_i));
    EXPECT_TRUE(factor.pts_j.isApprox(pts_j));
#ifndef UNIT_SPHERE_ERROR
    // In the default pinhole formulation tangent_base must be deterministic
    // (zero) so that reads against it never return undefined values.
    EXPECT_TRUE(factor.tangent_base.isZero());
#endif
}

TEST_F(ProjectionFactorTest, EvaluateProducesZeroResidualForConsistentState)
{
    EvaluationSetup s = buildConsistentEvaluation(/*seed=*/42, /*perturb=*/false);
    ProjectionFactor factor(s.pts_i, s.pts_j);

    auto params = s.paramArray();
    Eigen::Vector2d residual;
    ASSERT_TRUE(factor.Evaluate(params.data(), residual.data(), nullptr));
    // sqrt_info scales by FOCAL_LENGTH / 1.5 ~ 307, so allow some slack but
    // the residual should still be essentially zero (the construction is
    // analytically self-consistent).
    EXPECT_LT(residual.norm(), 1e-9);
}

TEST_F(ProjectionFactorTest, EvaluateProducesNonZeroResidualOnPerturbedObservation)
{
    EvaluationSetup s = buildConsistentEvaluation(/*seed=*/2026, /*perturb=*/true);
    ProjectionFactor factor(s.pts_i, s.pts_j);

    auto params = s.paramArray();
    Eigen::Vector2d residual;
    ASSERT_TRUE(factor.Evaluate(params.data(), residual.data(), nullptr));
    EXPECT_GT(residual.norm(), 1e-3);
}

TEST_F(ProjectionFactorTest, EvaluateRespectsNullJacobiansPointer)
{
    EvaluationSetup s = buildConsistentEvaluation(/*seed=*/11, /*perturb=*/true);
    ProjectionFactor factor(s.pts_i, s.pts_j);

    auto params = s.paramArray();
    Eigen::Vector2d r;
    EXPECT_TRUE(factor.Evaluate(params.data(), r.data(), nullptr));
}

TEST_F(ProjectionFactorTest, SqrtInfoScalesResidualLinearly)
{
    EvaluationSetup s = buildConsistentEvaluation(/*seed=*/77, /*perturb=*/true);
    ProjectionFactor factor(s.pts_i, s.pts_j);

    ProjectionFactor::sqrt_info = Eigen::Matrix2d::Identity();
    auto params = s.paramArray();
    Eigen::Vector2d r_unit;
    ASSERT_TRUE(factor.Evaluate(params.data(), r_unit.data(), nullptr));

    constexpr double scale = 5.0;
    ProjectionFactor::sqrt_info = scale * Eigen::Matrix2d::Identity();
    Eigen::Vector2d r_scaled;
    ASSERT_TRUE(factor.Evaluate(params.data(), r_scaled.data(), nullptr));
    EXPECT_LT((r_scaled - scale * r_unit).norm(), 1e-9);
}

TEST_F(ProjectionFactorTest, AnalyticJacobiansMatchFiniteDifferences)
{
    EvaluationSetup s = buildConsistentEvaluation(/*seed=*/7, /*perturb=*/true);
    ProjectionFactor factor(s.pts_i, s.pts_j);

    Eigen::Matrix<double, 2, 7, Eigen::RowMajor> J_pi;
    Eigen::Matrix<double, 2, 7, Eigen::RowMajor> J_pj;
    Eigen::Matrix<double, 2, 7, Eigen::RowMajor> J_ex;
    Eigen::Matrix<double, 2, 1>                 J_id;
    std::array<double *, 4> jacobians = {J_pi.data(), J_pj.data(), J_ex.data(), J_id.data()};

    auto params = s.paramArray();
    Eigen::Vector2d r0;
    ASSERT_TRUE(factor.Evaluate(params.data(), r0.data(), jacobians.data()));

    // Use central differences (O(eps^2) truncation error) rather than forward
    // differences so the comparison tolerance is not dominated by truncation
    // noise. sqrt_info scales the residual by ~FOCAL_LENGTH / 1.5 (~307), which
    // amplifies any one-sided cancellation error from forward differences.
    constexpr double eps = 1e-6;
    auto residual_with = [&](std::array<double *, 4> p) {
        return residualOnly(&factor, p.data());
    };

    auto translation_col = [&](int pose_index, int axis) -> Eigen::Vector2d {
        std::array<double, 7> base{};
        std::copy_n(params[pose_index], 7, base.begin());

        std::array<double, 7> plus = base;
        plus[axis] += eps;
        std::array<double, 7> minus = base;
        minus[axis] -= eps;

        auto p_plus  = s.paramArray();
        auto p_minus = s.paramArray();
        p_plus[pose_index]  = plus.data();
        p_minus[pose_index] = minus.data();
        return (residual_with(p_plus) - residual_with(p_minus)) / (2.0 * eps);
    };

    auto rotation_col = [&](int pose_index, int axis) -> Eigen::Vector2d {
        std::array<double, 7> base{};
        std::copy_n(params[pose_index], 7, base.begin());
        const Eigen::Quaterniond q_orig(base[6], base[3], base[4], base[5]);

        Eigen::Vector3d delta = Eigen::Vector3d::Zero();
        delta[axis] = eps;
        const Eigen::Quaterniond q_plus  = q_orig * Utility::deltaQ(Eigen::Vector3d(delta));
        const Eigen::Quaterniond q_minus = q_orig * Utility::deltaQ(Eigen::Vector3d(-delta));

        std::array<double, 7> plus  = base;
        std::array<double, 7> minus = base;
        plus[3]  = q_plus.x();  plus[4]  = q_plus.y();  plus[5]  = q_plus.z();  plus[6]  = q_plus.w();
        minus[3] = q_minus.x(); minus[4] = q_minus.y(); minus[5] = q_minus.z(); minus[6] = q_minus.w();

        auto p_plus  = s.paramArray();
        auto p_minus = s.paramArray();
        p_plus[pose_index]  = plus.data();
        p_minus[pose_index] = minus.data();
        return (residual_with(p_plus) - residual_with(p_minus)) / (2.0 * eps);
    };

    auto check_pose = [&](int pose_index,
                          const Eigen::Matrix<double, 2, 7, Eigen::RowMajor> &J,
                          const char *name) {
        Eigen::Matrix<double, 2, 6> num_J;
        for (int k = 0; k < 3; ++k) num_J.col(k)     = translation_col(pose_index, k);
        for (int k = 0; k < 3; ++k) num_J.col(3 + k) = rotation_col(pose_index, k);
        EXPECT_TRUE(J.leftCols<6>().isApprox(num_J, 1e-4))
            << name << " analytic:\n" << J.leftCols<6>()
            << "\nnumeric:\n" << num_J
            << "\nmax abs diff: " << (J.leftCols<6>() - num_J).cwiseAbs().maxCoeff();
        // The 7th column corresponds to q.w which is locked by the local
        // parameterisation; ProjectionFactor explicitly zeroes it.
        EXPECT_LT(J.col(6).cwiseAbs().maxCoeff(), 1e-12) << name;
    };

    check_pose(0, J_pi, "pose_i");
    check_pose(1, J_pj, "pose_j");
    check_pose(2, J_ex, "extrinsic");

    // Inverse depth (1D, central difference).
    std::array<double, 1> plus  = s.inv_depth; plus[0]  += eps;
    std::array<double, 1> minus = s.inv_depth; minus[0] -= eps;
    auto p_plus  = s.paramArray();
    auto p_minus = s.paramArray();
    p_plus[3]  = plus.data();
    p_minus[3] = minus.data();
    const Eigen::Vector2d num_J_id =
        (residual_with(p_plus) - residual_with(p_minus)) / (2.0 * eps);
    EXPECT_TRUE(J_id.isApprox(num_J_id, 1e-4))
        << "inv_depth analytic:\n" << J_id
        << "\nnumeric:\n" << num_J_id
        << "\nmax abs diff: " << (J_id - num_J_id).cwiseAbs().maxCoeff();
}

TEST_F(ProjectionFactorTest, TimeDelayParameterProducesZeroResidualForConsistentState)
{
    TdEvaluationSetup s = buildConsistentTdEvaluation(/*seed=*/128, /*perturb=*/false);
    ProjectionFactor factor(s.pts_i, s.pts_j, s.velocity_i, s.velocity_j,
                            s.td_i, s.td_j, s.row_i, s.row_j);

    auto params = s.paramArray();
    Eigen::Vector2d residual;
    ASSERT_TRUE(factor.Evaluate(params.data(), residual.data(), nullptr));
    EXPECT_LT(residual.norm(), 1e-9);
}

TEST_F(ProjectionFactorTest, TimeDelayParameterJacobianMatchesFiniteDifference)
{
    TdEvaluationSetup s = buildConsistentTdEvaluation(/*seed=*/512, /*perturb=*/true);
    ProjectionFactor factor(s.pts_i, s.pts_j, s.velocity_i, s.velocity_j,
                            s.td_i, s.td_j, s.row_i, s.row_j);

    Eigen::Matrix<double, 2, 7, Eigen::RowMajor> J_pi;
    Eigen::Matrix<double, 2, 7, Eigen::RowMajor> J_pj;
    Eigen::Matrix<double, 2, 7, Eigen::RowMajor> J_ex;
    Eigen::Matrix<double, 2, 1>                 J_id;
    Eigen::Matrix<double, 2, 1>                 J_td;
    std::array<double *, 5> jacobians = {
        J_pi.data(), J_pj.data(), J_ex.data(), J_id.data(), J_td.data()};

    auto params = s.paramArray();
    Eigen::Vector2d r0;
    ASSERT_TRUE(factor.Evaluate(params.data(), r0.data(), jacobians.data()));

    auto residual_with = [&](std::array<double *, 5> p) {
        Eigen::Vector2d r;
        factor.Evaluate(p.data(), r.data(), nullptr);
        return r;
    };

    constexpr double eps = 1e-6;
    std::array<double, 1> plus = s.td_param;
    std::array<double, 1> minus = s.td_param;
    plus[0] += eps;
    minus[0] -= eps;

    auto p_plus = s.paramArray();
    auto p_minus = s.paramArray();
    p_plus[4] = plus.data();
    p_minus[4] = minus.data();

    const Eigen::Vector2d num_J_td =
        (residual_with(p_plus) - residual_with(p_minus)) / (2.0 * eps);
    EXPECT_TRUE(J_td.isApprox(num_J_td, 1e-4))
        << "td analytic:\n" << J_td
        << "\nnumeric:\n" << num_J_td
        << "\nmax abs diff: " << (J_td - num_J_td).cwiseAbs().maxCoeff();
}
