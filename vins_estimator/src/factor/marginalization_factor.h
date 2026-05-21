#pragma once

#include <ceres/ceres.h>
#include <Eigen/Dense>

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using ParameterBlockId = std::uintptr_t;

struct ResidualBlockInfo
{
    ResidualBlockInfo(ceres::CostFunction *cost_function,
                      ceres::LossFunction *loss_function,
                      std::vector<double *> parameter_blocks,
                      std::vector<int> drop_set);

    void Evaluate();

    std::unique_ptr<ceres::CostFunction> cost_function;
    ceres::LossFunction *loss_function = nullptr;  // Borrowed; owned by the caller/Ceres problem.
    std::vector<double *> parameter_blocks;
    std::vector<int> drop_set;

    std::vector<double *> raw_jacobians;
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobians;
    Eigen::VectorXd residuals;
};

class MarginalizationInfo
{
  public:
    ~MarginalizationInfo();
    int localSize(int size) const;
    int globalSize(int size) const;
    void addResidualBlockInfo(ResidualBlockInfo *residual_block_info);
    void preMarginalize();
    void marginalize();
    std::vector<double *> getParameterBlocks(std::unordered_map<ParameterBlockId, double *> &addr_shift);

    std::vector<std::unique_ptr<ResidualBlockInfo>> factors;
    int m = 0;
    int n = 0;
    std::unordered_map<ParameterBlockId, int> parameter_block_size;  // Global size.
    int sum_block_size = 0;
    std::unordered_set<ParameterBlockId> drop_blocks;
    std::vector<ParameterBlockId> ordered_blocks;
    std::unordered_map<ParameterBlockId, int> local_block_index;  // Local-size offset.
    std::unordered_map<ParameterBlockId, std::vector<double>> parameter_block_data;

    std::vector<int> keep_block_size;        // Global size.
    std::vector<int> keep_block_idx;         // Local-size offset.
    std::vector<const double *> keep_block_data;

    Eigen::MatrixXd linearized_jacobians;
    Eigen::VectorXd linearized_residuals;
    const double eps = 1e-8;
};

class MarginalizationFactor : public ceres::CostFunction
{
  public:
    explicit MarginalizationFactor(MarginalizationInfo *marginalization_info);
    bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override;

    MarginalizationInfo *marginalization_info = nullptr;
};
