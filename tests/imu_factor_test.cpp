// Unit tests for vins_estimator/src/factor/imu_factor.{h,cpp}.
//
// Coverage:
//   1. Integrator construction, propagate, process, repropagate, computeResidual.
//   2. IMUFactor::Evaluate residual at a consistent state pair.
//   3. IMUFactor analytical jacobians vs finite-difference (minimal tangent
//      space for rotations).
//   4. shared_ptr ownership semantics between Integrator <-> IMUFactor.

#include <gtest/gtest.h>

#include "vins_glog_init.h"

#include <algorithm>
#include <array>
#include <memory>
#include <random>
#include <tuple>
#include <vector>

#include <eigen3/Eigen/Dense>

#include "factor/imu_factor.h"
#include "parameters.h"
#include "utility/utility.h"

namespace
{

constexpr double kIntegratorDt = 0.005;

void setDefaultImuNoise()
{
    // Non-zero values are required: the covariance is propagated using `noise`
    // (which is filled inside Integrator's ctor) and IMUFactor::Evaluate later
    // computes `LLT(covariance.inverse())`. With zero noise the covariance is
    // singular and the LLT factor is full of NaN/Inf.
    auto &params = vinsParameters();
    params.setWindowSize(10);
    params.setNumOfCam(1);
    params.setMaxFeatureCount(1000);
    params.setFocalLength(460.0);
    params.setAccNoise(0.1);
    params.setAccRandomWalk(0.001);
    params.setGyrNoise(0.01);
    params.setGyrRandomWalk(0.0001);
    params.setGravity(Eigen::Vector3d(0.0, 0.0, 9.81));
}

// Build a deterministic IMU stream for reuse across tests.
//
// IMU samples are generated as a band-limited random walk rather than i.i.d.
// uniform noise: each step perturbs the previous body-frame acceleration /
// angular velocity by a small amount and the magnitude is clamped to keep the
// resulting trajectory physically plausible (max |a_body| <= 2 m/s^2,
// max |w| <= 0.5 rad/s). The accelerometer measurement additionally includes
// the gravity component on the z axis to mimic a sensor at rest in world
// frame z-up convention.
std::vector<std::tuple<double, Eigen::Vector3d, Eigen::Vector3d>>
makeImuStream(std::mt19937 &rng, int n_steps, double dt)
{
    constexpr double kStepStd = 0.02;       // per-sample jitter on a_body / gyro.
    constexpr double kAccBodyLimit = 2.0;   // |a_body| <= 2 m/s^2.
    constexpr double kGyroLimit = 0.5;      // |w|      <= 0.5 rad/s.
    const Eigen::Vector3d g_world(0.0, 0.0, 9.81);

    std::uniform_real_distribution<double> step(-kStepStd, kStepStd);
    Eigen::Vector3d a_body = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyr = Eigen::Vector3d::Zero();

    std::vector<std::tuple<double, Eigen::Vector3d, Eigen::Vector3d>> stream;
    stream.reserve(n_steps);
    for (int i = 0; i < n_steps; ++i)
    {
        a_body += Eigen::Vector3d(step(rng), step(rng), step(rng));
        gyr    += Eigen::Vector3d(step(rng), step(rng), step(rng));
        if (a_body.norm() > kAccBodyLimit) a_body *= kAccBodyLimit / a_body.norm();
        if (gyr.norm() > kGyroLimit)       gyr    *= kGyroLimit    / gyr.norm();
        stream.emplace_back(dt, a_body + g_world, gyr);
    }
    return stream;
}

// Pack/unpack helpers matching IMUFactor's parameter layout.
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

void packSpeedBias(double out[9],
                   const Eigen::Vector3d &v,
                   const Eigen::Vector3d &ba,
                   const Eigen::Vector3d &bg)
{
    out[0] = v.x();
    out[1] = v.y();
    out[2] = v.z();
    out[3] = ba.x();
    out[4] = ba.y();
    out[5] = ba.z();
    out[6] = bg.x();
    out[7] = bg.y();
    out[8] = bg.z();
}

// Forward-roll the j-state from the i-state and pre-integration so that
// the resulting (Pj, Qj, Vj) makes IMUFactor's residual identically zero.
void rollForwardState(const Integrator &integ,
                      const Eigen::Vector3d &Pi,
                      const Eigen::Quaterniond &Qi,
                      const Eigen::Vector3d &Vi,
                      Eigen::Vector3d *Pj,
                      Eigen::Quaterniond *Qj,
                      Eigen::Vector3d *Vj)
{
    const double t = integ.sum_dt;
    const Eigen::Vector3d &gravity = vinsParameters().gravity();
    *Pj = Pi + Vi * t - 0.5 * gravity * t * t + Qi * integ.delta_p;
    *Qj = Qi * integ.delta_q;
    *Vj = Vi - gravity * t + Qi * integ.delta_v;
}

class IntegratorTest : public ::testing::Test
{
  protected:
    void SetUp() override { setDefaultImuNoise(); }
};

class IMUFactorTest : public ::testing::Test
{
  protected:
    void SetUp() override { setDefaultImuNoise(); }
};

}  // namespace

// ---------------------------------------------------------------------------
// Integrator
// ---------------------------------------------------------------------------

TEST_F(IntegratorTest, ConstructorInitializesStateAndNoise)
{
    const Eigen::Vector3d acc(0.0, 0.0, 9.81);
    const Eigen::Vector3d gyr(0.0, 0.0, 0.0);
    const Eigen::Vector3d ba(0.01, -0.02, 0.005);
    const Eigen::Vector3d bg(-0.001, 0.002, 0.003);

    Integrator integ(acc, gyr, ba, bg);

    EXPECT_EQ(integ.sum_dt, 0.0);
    EXPECT_TRUE(integ.delta_p.isZero());
    EXPECT_TRUE(integ.delta_v.isZero());
    EXPECT_DOUBLE_EQ(integ.delta_q.angularDistance(Eigen::Quaterniond::Identity()), 0.0);
    EXPECT_TRUE(integ.jacobian.isApprox(Eigen::Matrix<double, 15, 15>::Identity()));
    EXPECT_TRUE(integ.covariance.isZero());

    EXPECT_EQ(integ.linearized_acc, acc);
    EXPECT_EQ(integ.linearized_gyr, gyr);
    EXPECT_EQ(integ.linearized_ba, ba);
    EXPECT_EQ(integ.linearized_bg, bg);

    // Spot-check that the noise matrix has the configured diagonal blocks.
    const auto &params = vinsParameters();
    EXPECT_DOUBLE_EQ(integ.noise(0, 0), params.accNoise() * params.accNoise());
    EXPECT_DOUBLE_EQ(integ.noise(3, 3), params.gyrNoise() * params.gyrNoise());
    EXPECT_DOUBLE_EQ(integ.noise(12, 12), params.accRandomWalk() * params.accRandomWalk());
    EXPECT_DOUBLE_EQ(integ.noise(15, 15), params.gyrRandomWalk() * params.gyrRandomWalk());
}

TEST_F(IntegratorTest, ZeroMeasurementsKeepIncrementsZero)
{
    Integrator integ(Eigen::Vector3d::Zero(),
                     Eigen::Vector3d::Zero(),
                     Eigen::Vector3d::Zero(),
                     Eigen::Vector3d::Zero());

    for (int i = 0; i < 200; ++i)
        integ.process(kIntegratorDt, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    EXPECT_NEAR(integ.sum_dt, 200 * kIntegratorDt, 1e-12);
    EXPECT_LT(integ.delta_p.norm(), 1e-12);
    EXPECT_LT(integ.delta_v.norm(), 1e-12);
    EXPECT_NEAR(integ.delta_q.angularDistance(Eigen::Quaterniond::Identity()), 0.0, 1e-12);

    // With non-zero process noise the covariance should still grow strictly
    // monotonically along the diagonal once measurements are integrated.
    EXPECT_GT(integ.covariance.trace(), 0.0);
}

TEST_F(IntegratorTest, ConstantBodyAccelerationMatchesAnalyticPV)
{
    const Eigen::Vector3d acc(1.0, -0.5, 0.25);
    const Eigen::Vector3d gyr = Eigen::Vector3d::Zero();
    Integrator integ(acc, gyr, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    const int n = 1000;
    for (int i = 0; i < n; ++i)
        integ.process(kIntegratorDt, acc, gyr);

    const double t = n * kIntegratorDt;
    const Eigen::Vector3d expected_v = acc * t;
    const Eigen::Vector3d expected_p = 0.5 * acc * t * t;
    EXPECT_LT((integ.delta_v - expected_v).norm(), 1e-8);
    EXPECT_LT((integ.delta_p - expected_p).norm(), 1e-8);
    EXPECT_NEAR(integ.delta_q.angularDistance(Eigen::Quaterniond::Identity()), 0.0, 1e-9);
}

TEST_F(IntegratorTest, ConstantAngularVelocityMatchesClosedFormQuaternion)
{
    const Eigen::Vector3d acc = Eigen::Vector3d::Zero();
    const Eigen::Vector3d gyr(0.05, -0.07, 0.11);
    Integrator integ(acc, gyr, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    const int n = 1000;
    for (int i = 0; i < n; ++i)
        integ.process(kIntegratorDt, acc, gyr);

    const double t = n * kIntegratorDt;
    const double angle = gyr.norm() * t;
    const Eigen::Quaterniond expected(Eigen::AngleAxisd(angle, gyr.normalized()));

    // Mid-point integration on a sphere is second-order: an error <1e-3 rad
    // is expected here.
    EXPECT_NEAR(integ.delta_q.angularDistance(expected), 0.0, 1e-3);
    EXPECT_LT(integ.delta_v.norm(), 1e-9);
    EXPECT_LT(integ.delta_p.norm(), 1e-9);
}

TEST_F(IntegratorTest, RepropagateReproducesFullPropagation)
{
    std::mt19937 rng(42);
    const auto stream = makeImuStream(rng, 60, kIntegratorDt);

    const Eigen::Vector3d ba(0.01, -0.02, 0.03);
    const Eigen::Vector3d bg(-0.005, 0.01, -0.02);

    // Build the reference via the natural process path.
    Integrator reference(std::get<1>(stream.front()), std::get<2>(stream.front()), ba, bg);
    for (const auto &[dt, a, g] : stream)
        reference.process(dt, a, g);

    // Build another instance with different starting biases and then repropagate
    // with the reference biases. The two should match exactly.
    Integrator candidate(std::get<1>(stream.front()), std::get<2>(stream.front()),
                         Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    for (const auto &[dt, a, g] : stream)
        candidate.process(dt, a, g);
    candidate.repropagate(ba, bg);

    EXPECT_LT((reference.delta_p - candidate.delta_p).norm(), 1e-12);
    EXPECT_LT((reference.delta_v - candidate.delta_v).norm(), 1e-12);
    EXPECT_NEAR(reference.delta_q.angularDistance(candidate.delta_q), 0.0, 1e-12);
    EXPECT_TRUE(reference.jacobian.isApprox(candidate.jacobian, 1e-12));
    EXPECT_TRUE(reference.covariance.isApprox(candidate.covariance, 1e-12));
    EXPECT_NEAR(reference.sum_dt, candidate.sum_dt, 1e-12);
}

TEST_F(IntegratorTest, ComputeResidualIsZeroForConsistentStatePair)
{
    std::mt19937 rng(123);
    const auto stream = makeImuStream(rng, 40, kIntegratorDt);

    const Eigen::Vector3d ba = Eigen::Vector3d::Zero();
    const Eigen::Vector3d bg = Eigen::Vector3d::Zero();

    Integrator integ(std::get<1>(stream.front()), std::get<2>(stream.front()), ba, bg);
    for (const auto &[dt, a, g] : stream)
        integ.process(dt, a, g);

    const Eigen::Vector3d Pi(0.3, -0.2, 0.1);
    const Eigen::Quaterniond Qi(
        Eigen::AngleAxisd(0.4, Eigen::Vector3d(0.2, 0.5, 0.8).normalized()));
    const Eigen::Vector3d Vi(0.05, -0.02, 0.1);

    Eigen::Vector3d Pj;
    Eigen::Quaterniond Qj;
    Eigen::Vector3d Vj;
    rollForwardState(integ, Pi, Qi, Vi, &Pj, &Qj, &Vj);

    const Eigen::Matrix<double, 15, 1> r =
        integ.computeResidual(Pi, Qi, Vi, ba, bg, Pj, Qj, Vj, ba, bg);
    EXPECT_LT(r.norm(), 1e-9);
}

TEST_F(IntegratorTest, ComputeResidualGrowsLinearlyWithStatePerturbation)
{
    std::mt19937 rng(7);
    const auto stream = makeImuStream(rng, 30, kIntegratorDt);

    const Eigen::Vector3d ba = Eigen::Vector3d::Zero();
    const Eigen::Vector3d bg = Eigen::Vector3d::Zero();
    Integrator integ(std::get<1>(stream.front()), std::get<2>(stream.front()), ba, bg);
    for (const auto &[dt, a, g] : stream)
        integ.process(dt, a, g);

    const Eigen::Vector3d Pi(0.0, 0.0, 0.0);
    const Eigen::Quaterniond Qi = Eigen::Quaterniond::Identity();
    const Eigen::Vector3d Vi(0.1, -0.05, 0.2);

    Eigen::Vector3d Pj;
    Eigen::Quaterniond Qj;
    Eigen::Vector3d Vj;
    rollForwardState(integ, Pi, Qi, Vi, &Pj, &Qj, &Vj);

    const Eigen::Vector3d dp(0.01, -0.02, 0.005);
    Eigen::Matrix<double, 15, 1> r =
        integ.computeResidual(Pi, Qi, Vi, ba, bg, Pj + dp, Qj, Vj, ba, bg);
    // The Pj perturbation enters the position residual rotated by Qi^{-1}.
    EXPECT_LT((r.segment<3>(O_P) - Qi.inverse() * dp).norm(), 1e-9);
    // Other residual blocks must remain (near) zero.
    EXPECT_LT(r.segment<3>(O_R).norm(), 1e-9);
    EXPECT_LT(r.segment<3>(O_V).norm(), 1e-9);
    EXPECT_LT(r.segment<3>(O_BA).norm(), 1e-9);
    EXPECT_LT(r.segment<3>(O_BG).norm(), 1e-9);
}

// ---------------------------------------------------------------------------
// IMUFactor
// ---------------------------------------------------------------------------

namespace
{

struct EvaluationSetup
{
    std::shared_ptr<Integrator> integrator;
    Eigen::Vector3d Pi, Vi, Bai, Bgi;
    Eigen::Quaterniond Qi;
    Eigen::Vector3d Pj, Vj, Baj, Bgj;
    Eigen::Quaterniond Qj;

    std::array<double, 7> pose_i{};
    std::array<double, 9> sb_i{};
    std::array<double, 7> pose_j{};
    std::array<double, 9> sb_j{};

    void pack()
    {
        packPose(pose_i.data(), Pi, Qi);
        packSpeedBias(sb_i.data(), Vi, Bai, Bgi);
        packPose(pose_j.data(), Pj, Qj);
        packSpeedBias(sb_j.data(), Vj, Baj, Bgj);
    }

    std::array<double *, 4> paramArray()
    {
        return {pose_i.data(), sb_i.data(), pose_j.data(), sb_j.data()};
    }
};

EvaluationSetup buildEvaluation(unsigned seed)
{
    EvaluationSetup s;
    std::mt19937 rng(seed);
    const auto stream = makeImuStream(rng, 25, kIntegratorDt);

    s.Bai = Eigen::Vector3d(0.01, 0.02, -0.01);
    s.Bgi = Eigen::Vector3d(-0.005, 0.003, 0.002);
    s.Baj = s.Bai;
    s.Bgj = s.Bgi;

    s.integrator = std::make_shared<Integrator>(
        std::get<1>(stream.front()), std::get<2>(stream.front()), s.Bai, s.Bgi);
    for (const auto &[dt, a, g] : stream)
        s.integrator->process(dt, a, g);

    s.Pi = Eigen::Vector3d(0.2, -0.1, 0.05);
    s.Qi = Eigen::Quaterniond(
        Eigen::AngleAxisd(0.3, Eigen::Vector3d(0.2, 0.3, 0.5).normalized()));
    s.Vi = Eigen::Vector3d(0.1, -0.05, 0.02);

    rollForwardState(*s.integrator, s.Pi, s.Qi, s.Vi, &s.Pj, &s.Qj, &s.Vj);

    // Slightly perturb j-state so residual is non-trivial.
    s.Pj += Eigen::Vector3d(0.02, -0.015, 0.01);
    s.Vj += Eigen::Vector3d(0.005, -0.002, 0.003);
    s.Qj = s.Qj * Utility::deltaQ(Eigen::Vector3d(0.01, -0.005, 0.008));
    s.pack();
    return s;
}

Eigen::Matrix<double, 15, 1> residualOnly(IMUFactor *factor, double *const *params)
{
    Eigen::Matrix<double, 15, 1> r;
    factor->Evaluate(params, r.data(), nullptr);
    return r;
}

}  // namespace

TEST_F(IMUFactorTest, EvaluateProducesZeroResidualForConsistentState)
{
    EvaluationSetup s = buildEvaluation(99);
    // Roll-forward without extra perturbation.
    rollForwardState(*s.integrator, s.Pi, s.Qi, s.Vi, &s.Pj, &s.Qj, &s.Vj);
    s.pack();

    IMUFactor factor(s.integrator);
    auto params = s.paramArray();
    Eigen::Matrix<double, 15, 1> residual;
    EXPECT_TRUE(factor.Evaluate(params.data(), residual.data(), nullptr));
    EXPECT_LT(residual.norm(), 1e-6);  // sqrt_info scales by ~1/sigma so allow some slack.
}

TEST_F(IMUFactorTest, EvaluateRespectsNullJacobiansPointer)
{
    EvaluationSetup s = buildEvaluation(11);
    IMUFactor factor(s.integrator);
    auto params = s.paramArray();
    Eigen::Matrix<double, 15, 1> r;
    EXPECT_TRUE(factor.Evaluate(params.data(), r.data(), nullptr));
}

TEST_F(IMUFactorTest, AnalyticJacobiansMatchFiniteDifferences)
{
    EvaluationSetup s = buildEvaluation(7);
    IMUFactor factor(s.integrator);
    auto params = s.paramArray();

    Eigen::Matrix<double, 15, 7, Eigen::RowMajor> J_pi;
    Eigen::Matrix<double, 15, 9, Eigen::RowMajor> J_sbi;
    Eigen::Matrix<double, 15, 7, Eigen::RowMajor> J_pj;
    Eigen::Matrix<double, 15, 9, Eigen::RowMajor> J_sbj;
    std::array<double *, 4> jacobians = {J_pi.data(), J_sbi.data(), J_pj.data(), J_sbj.data()};
    Eigen::Matrix<double, 15, 1> r0;
    ASSERT_TRUE(factor.Evaluate(params.data(), r0.data(), jacobians.data()));

    const double eps = 1e-7;
    auto residual_with = [&](std::array<double *, 4> p) { return residualOnly(&factor, p.data()); };

    // ---- pose_i (use the minimal 6-DoF parameterisation) ----
    Eigen::Matrix<double, 15, 6> num_J_pi;
    for (int k = 0; k < 3; ++k)
    {
        std::array<double, 7> perturbed = s.pose_i;
        perturbed[k] += eps;
        auto p = s.paramArray();
        p[0] = perturbed.data();
        num_J_pi.col(k) = (residual_with(p) - r0) / eps;
    }
    for (int k = 0; k < 3; ++k)
    {
        std::array<double, 7> perturbed = s.pose_i;
        Eigen::Quaterniond q_orig(perturbed[6], perturbed[3], perturbed[4], perturbed[5]);
        Eigen::Vector3d delta = Eigen::Vector3d::Zero();
        delta[k] = eps;
        Eigen::Quaterniond q_new = q_orig * Utility::deltaQ(delta);
        perturbed[3] = q_new.x();
        perturbed[4] = q_new.y();
        perturbed[5] = q_new.z();
        perturbed[6] = q_new.w();
        auto p = s.paramArray();
        p[0] = perturbed.data();
        num_J_pi.col(3 + k) = (residual_with(p) - r0) / eps;
    }
    EXPECT_TRUE(J_pi.leftCols<6>().isApprox(num_J_pi, 5e-4))
        << "analytic pose_i (6 cols):\n" << J_pi.leftCols<6>()
        << "\nnumeric pose_i:\n" << num_J_pi
        << "\nmax abs diff: " << (J_pi.leftCols<6>() - num_J_pi).cwiseAbs().maxCoeff();
    // The 7th column corresponds to q.w which is locked by the local
    // parameterisation: the factor is expected to leave it zeroed out.
    EXPECT_LT(J_pi.col(6).cwiseAbs().maxCoeff(), 1e-12);

    // ---- sb_i (Vi, ba_i, bg_i) ----
    Eigen::Matrix<double, 15, 9> num_J_sbi;
    for (int k = 0; k < 9; ++k)
    {
        std::array<double, 9> perturbed = s.sb_i;
        perturbed[k] += eps;
        auto p = s.paramArray();
        p[1] = perturbed.data();
        num_J_sbi.col(k) = (residual_with(p) - r0) / eps;
    }
    EXPECT_TRUE(J_sbi.isApprox(num_J_sbi, 5e-4))
        << "max abs diff (sb_i): " << (J_sbi - num_J_sbi).cwiseAbs().maxCoeff();

    // ---- pose_j (minimal 6 DoF) ----
    Eigen::Matrix<double, 15, 6> num_J_pj;
    for (int k = 0; k < 3; ++k)
    {
        std::array<double, 7> perturbed = s.pose_j;
        perturbed[k] += eps;
        auto p = s.paramArray();
        p[2] = perturbed.data();
        num_J_pj.col(k) = (residual_with(p) - r0) / eps;
    }
    for (int k = 0; k < 3; ++k)
    {
        std::array<double, 7> perturbed = s.pose_j;
        Eigen::Quaterniond q_orig(perturbed[6], perturbed[3], perturbed[4], perturbed[5]);
        Eigen::Vector3d delta = Eigen::Vector3d::Zero();
        delta[k] = eps;
        Eigen::Quaterniond q_new = q_orig * Utility::deltaQ(delta);
        perturbed[3] = q_new.x();
        perturbed[4] = q_new.y();
        perturbed[5] = q_new.z();
        perturbed[6] = q_new.w();
        auto p = s.paramArray();
        p[2] = perturbed.data();
        num_J_pj.col(3 + k) = (residual_with(p) - r0) / eps;
    }
    EXPECT_TRUE(J_pj.leftCols<6>().isApprox(num_J_pj, 5e-4))
        << "max abs diff (pose_j): " << (J_pj.leftCols<6>() - num_J_pj).cwiseAbs().maxCoeff();
    EXPECT_LT(J_pj.col(6).cwiseAbs().maxCoeff(), 1e-12);

    // ---- sb_j (Vj, ba_j, bg_j) ----
    Eigen::Matrix<double, 15, 9> num_J_sbj;
    for (int k = 0; k < 9; ++k)
    {
        std::array<double, 9> perturbed = s.sb_j;
        perturbed[k] += eps;
        auto p = s.paramArray();
        p[3] = perturbed.data();
        num_J_sbj.col(k) = (residual_with(p) - r0) / eps;
    }
    EXPECT_TRUE(J_sbj.isApprox(num_J_sbj, 5e-4))
        << "max abs diff (sb_j): " << (J_sbj - num_J_sbj).cwiseAbs().maxCoeff();
}

TEST_F(IMUFactorTest, SharedPointerOwnershipOutlivesEstimatorReset)
{
    EvaluationSetup s = buildEvaluation(2026);
    IMUFactor *factor = new IMUFactor(s.integrator);
    EXPECT_EQ(s.integrator.use_count(), 2);

    // Simulate the estimator dropping its slot (e.g. slideWindow reset()).
    std::weak_ptr<Integrator> weak = s.integrator;
    s.integrator.reset();
    EXPECT_FALSE(weak.expired());  // factor still pins it alive.

    auto params = s.paramArray();
    Eigen::Matrix<double, 15, 1> r;
    EXPECT_TRUE(factor->Evaluate(params.data(), r.data(), nullptr));

    delete factor;  // mimics Ceres releasing the residual block.
    EXPECT_TRUE(weak.expired());
}
