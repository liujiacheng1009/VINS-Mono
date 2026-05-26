// Unit tests for vins_estimator/src/factor/marginalization_factor.{h,cpp}.
//
// Coverage:
//   1. Schur complement numerical regression for a small linear system.
//   2. MarginalizationFactor::Evaluate() residual and jacobian output.
//   3. ResidualBlockInfo robust-loss residual/jacobian scaling.
//   4. ResidualBlockInfo repeated Evaluate() and no-Evaluate destruction paths.

#include <gtest/gtest.h>

#include "vins_glog_init.h"

#include <array>
#include <memory>
#include <unordered_map>
#include <vector>

#include <ceres/ceres.h>
#include <eigen3/Eigen/Dense>

#include "factor/marginalization_factor.h"

namespace
{

ParameterBlockId blockId(const double *addr)
{
    return reinterpret_cast<ParameterBlockId>(addr);
}

class TwoResidualLinearCost final : public ceres::SizedCostFunction<2, 1, 1>
{
  public:
    bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override
    {
        const double xm = parameters[0][0];
        const double xr = parameters[1][0];

        residuals[0] = xm + 2.0 * xr - 3.0;
        residuals[1] = xm - 1.0;

        if (jacobians)
        {
            if (jacobians[0])
            {
                jacobians[0][0] = 1.0;
                jacobians[0][1] = 1.0;
            }
            if (jacobians[1])
            {
                jacobians[1][0] = 2.0;
                jacobians[1][1] = 0.0;
            }
        }
        return true;
    }
};

class LinearUnaryCost final : public ceres::SizedCostFunction<1, 1>
{
  public:
    explicit LinearUnaryCost(double slope, double intercept = 0.0)
        : slope_(slope), intercept_(intercept)
    {
    }

    bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override
    {
        residuals[0] = slope_ * parameters[0][0] + intercept_;
        if (jacobians && jacobians[0])
            jacobians[0][0] = slope_;
        return true;
    }

  private:
    double slope_;
    double intercept_;
};

class ScalingLoss final : public ceres::LossFunction
{
  public:
    explicit ScalingLoss(double sqrt_scale)
        : scale_(sqrt_scale * sqrt_scale)
    {
    }

    void Evaluate(double sq_norm, double rho[3]) const override
    {
        rho[0] = scale_ * sq_norm;
        rho[1] = scale_;
        rho[2] = 0.0;
    }

  private:
    double scale_;
};

}  // namespace

TEST(MarginalizationFactorTest, SchurComplementMatchesSmallLinearSystem)
{
    double xm[1] = {0.5};
    double xr[1] = {-0.25};

    auto *info = new MarginalizationInfo();
    info->addResidualBlockInfo(new ResidualBlockInfo(
        new TwoResidualLinearCost(), nullptr, std::vector<double *>{xm, xr}, std::vector<int>{0}));

    info->preMarginalize();
    info->marginalize();

    std::unordered_map<ParameterBlockId, double *> addr_shift;
    addr_shift[blockId(xm)] = xm;
    addr_shift[blockId(xr)] = xr;
    const std::vector<double *> keep_blocks = info->getParameterBlocks(addr_shift);

    ASSERT_EQ(keep_blocks.size(), 1u);
    EXPECT_EQ(keep_blocks[0], xr);
    ASSERT_EQ(info->m, 1);
    ASSERT_EQ(info->n, 1);

    // At the linearization point r = [-3, -0.5], J = [[1, 2], [1, 0]].
    // Eliminating xm gives A' = 2 and b' = -2.5 for the remaining xr block.
    const Eigen::MatrixXd recovered_A =
        info->linearized_jacobians.transpose() * info->linearized_jacobians;
    const Eigen::VectorXd recovered_b =
        info->linearized_jacobians.transpose() * info->linearized_residuals;

    ASSERT_EQ(recovered_A.rows(), 1);
    ASSERT_EQ(recovered_A.cols(), 1);
    ASSERT_EQ(recovered_b.size(), 1);
    EXPECT_NEAR(recovered_A(0, 0), 2.0, 1e-9);
    EXPECT_NEAR(recovered_b(0), -2.5, 1e-9);

    delete info;
}

TEST(MarginalizationFactorTest, PriorEvaluateReturnsExpectedResidualAndJacobian)
{
    auto info = std::make_unique<MarginalizationInfo>();
    info->m = 3;
    info->n = 2;
    info->keep_block_size = {2};
    info->keep_block_idx = {3};
    info->parameter_block_data[123] = {1.0, 2.0};
    info->keep_block_data = {info->parameter_block_data[123].data()};
    info->linearized_jacobians.resize(2, 2);
    info->linearized_jacobians << 2.0, -1.0,
                                  0.5,  3.0;
    info->linearized_residuals.resize(2);
    info->linearized_residuals << 0.25, -0.5;

    MarginalizationFactor factor(info.get());

    double x[2] = {1.5, 1.0};
    double *parameters[1] = {x};
    Eigen::Vector2d residual;
    Eigen::Matrix<double, 2, 2, Eigen::RowMajor> jacobian;
    double *jacobians[1] = {jacobian.data()};

    ASSERT_TRUE(factor.Evaluate(parameters, residual.data(), jacobians));

    const Eigen::Vector2d dx(0.5, -1.0);
    const Eigen::Vector2d expected_residual =
        info->linearized_residuals + info->linearized_jacobians * dx;
    EXPECT_TRUE(residual.isApprox(expected_residual, 1e-12));
    EXPECT_TRUE(jacobian.isApprox(info->linearized_jacobians, 1e-12));
}

TEST(MarginalizationFactorTest, RobustLossScalesResidualsAndJacobians)
{
    double x[1] = {2.0};
    ScalingLoss loss(/*sqrt_scale=*/3.0);
    ResidualBlockInfo block(
        new LinearUnaryCost(/*slope=*/4.0, /*intercept=*/1.0),
        &loss,
        std::vector<double *>{x},
        std::vector<int>{});

    block.Evaluate();

    ASSERT_EQ(block.residuals.size(), 1);
    ASSERT_EQ(block.jacobians.size(), 1u);
    EXPECT_NEAR(block.residuals[0], 27.0, 1e-12);
    EXPECT_NEAR(block.jacobians[0](0, 0), 12.0, 1e-12);
}

TEST(MarginalizationFactorTest, ResidualBlockInfoEvaluateCanRepeatSafely)
{
    double x[1] = {1.0};
    ResidualBlockInfo block(
        new LinearUnaryCost(/*slope=*/2.0, /*intercept=*/-1.0),
        nullptr,
        std::vector<double *>{x},
        std::vector<int>{});

    block.Evaluate();
    ASSERT_EQ(block.raw_jacobians.size(), 1u);
    EXPECT_NEAR(block.residuals[0], 1.0, 1e-12);
    EXPECT_NEAR(block.jacobians[0](0, 0), 2.0, 1e-12);

    x[0] = 3.0;
    block.Evaluate();
    ASSERT_EQ(block.raw_jacobians.size(), 1u);
    EXPECT_NEAR(block.residuals[0], 5.0, 1e-12);
    EXPECT_NEAR(block.jacobians[0](0, 0), 2.0, 1e-12);
}

TEST(MarginalizationFactorTest, MarginalizationInfoCanDestroyUnevaluatedBlock)
{
    double x[1] = {1.0};
    {
        MarginalizationInfo info;
        info.addResidualBlockInfo(new ResidualBlockInfo(
            new LinearUnaryCost(/*slope=*/1.0),
            nullptr,
            std::vector<double *>{x},
            std::vector<int>{0}));
    }
    SUCCEED();
}
