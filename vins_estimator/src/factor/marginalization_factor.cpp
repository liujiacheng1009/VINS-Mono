#include "marginalization_factor.h"

#include "../utility/logging.h"
#include "../utility/utility.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <numeric>
#include <pthread.h>
#include <utility>

namespace
{

constexpr int kNumThreads = 4;

ParameterBlockId blockId(const double *addr)
{
    return reinterpret_cast<ParameterBlockId>(addr);
}

struct ThreadsStruct
{
    std::vector<ResidualBlockInfo *> sub_factors;
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    const std::unordered_map<ParameterBlockId, int> *parameter_block_size = nullptr;
    const std::unordered_map<ParameterBlockId, int> *local_block_index = nullptr;
};

void *ThreadsConstructA(void *threadsstruct)
{
    ThreadsStruct *p = static_cast<ThreadsStruct *>(threadsstruct);
    for (const auto *it : p->sub_factors)
    {
        for (int i = 0; i < static_cast<int>(it->parameter_blocks.size()); i++)
        {
            const ParameterBlockId block_i = blockId(it->parameter_blocks[i]);
            const int idx_i = p->local_block_index->at(block_i);
            int size_i = p->parameter_block_size->at(block_i);
            if (size_i == 7)
                size_i = 6;
            const auto jacobian_i = it->jacobians[i].leftCols(size_i);

            for (int j = i; j < static_cast<int>(it->parameter_blocks.size()); j++)
            {
                const ParameterBlockId block_j = blockId(it->parameter_blocks[j]);
                const int idx_j = p->local_block_index->at(block_j);
                int size_j = p->parameter_block_size->at(block_j);
                if (size_j == 7)
                    size_j = 6;
                const auto jacobian_j = it->jacobians[j].leftCols(size_j);

                if (i == j)
                    p->A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
                else
                {
                    p->A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
                    p->A.block(idx_j, idx_i, size_j, size_i) = p->A.block(idx_i, idx_j, size_i, size_j).transpose();
                }
            }
            p->b.segment(idx_i, size_i) += jacobian_i.transpose() * it->residuals;
        }
    }
    return threadsstruct;
}

}  // namespace

ResidualBlockInfo::ResidualBlockInfo(ceres::CostFunction *_cost_function,
                                     ceres::LossFunction *_loss_function,
                                     std::vector<double *> _parameter_blocks,
                                     std::vector<int> _drop_set)
    : cost_function(_cost_function),
      loss_function(_loss_function),
      parameter_blocks(std::move(_parameter_blocks)),
      drop_set(std::move(_drop_set))
{
}

void ResidualBlockInfo::Evaluate()
{
    residuals.resize(cost_function->num_residuals());

    const std::vector<int> &block_sizes = cost_function->parameter_block_sizes();
    raw_jacobians.resize(block_sizes.size());
    jacobians.resize(block_sizes.size());

    for (int i = 0; i < static_cast<int>(block_sizes.size()); i++)
    {
        jacobians[i].resize(cost_function->num_residuals(), block_sizes[i]);
        raw_jacobians[i] = jacobians[i].data();
    }
    cost_function->Evaluate(parameter_blocks.data(), residuals.data(), raw_jacobians.data());

    if (loss_function)
    {
        double sq_norm = residuals.squaredNorm();
        double rho[3];
        loss_function->Evaluate(sq_norm, rho);

        const double sqrt_rho1_ = std::sqrt(rho[1]);
        double residual_scaling_;
        double alpha_sq_norm_;

        if ((sq_norm == 0.0) || (rho[2] <= 0.0))
        {
            residual_scaling_ = sqrt_rho1_;
            alpha_sq_norm_ = 0.0;
        }
        else
        {
            const double D = 1.0 + 2.0 * sq_norm * rho[2] / rho[1];
            const double alpha = 1.0 - std::sqrt(D);
            residual_scaling_ = sqrt_rho1_ / (1 - alpha);
            alpha_sq_norm_ = alpha / sq_norm;
        }

        for (int i = 0; i < static_cast<int>(parameter_blocks.size()); i++)
        {
            jacobians[i] = sqrt_rho1_ * (jacobians[i] - alpha_sq_norm_ * residuals * (residuals.transpose() * jacobians[i]));
        }

        residuals *= residual_scaling_;
    }
}

MarginalizationInfo::~MarginalizationInfo() = default;

void MarginalizationInfo::addResidualBlockInfo(ResidualBlockInfo *residual_block_info)
{
    std::unique_ptr<ResidualBlockInfo> owned_residual_block(residual_block_info);

    std::vector<double *> &parameter_blocks = owned_residual_block->parameter_blocks;
    const std::vector<int> &parameter_block_sizes = owned_residual_block->cost_function->parameter_block_sizes();

    for (int i = 0; i < static_cast<int>(owned_residual_block->parameter_blocks.size()); i++)
    {
        double *addr = parameter_blocks[i];
        int size = parameter_block_sizes[i];
        const ParameterBlockId id = blockId(addr);
        if (parameter_block_size.find(id) == parameter_block_size.end())
            ordered_blocks.push_back(id);
        parameter_block_size[id] = size;
    }

    for (int i = 0; i < static_cast<int>(owned_residual_block->drop_set.size()); i++)
    {
        double *addr = parameter_blocks[owned_residual_block->drop_set[i]];
        drop_blocks.insert(blockId(addr));
    }

    factors.emplace_back(std::move(owned_residual_block));
}

void MarginalizationInfo::preMarginalize()
{
    for (const auto &it : factors)
    {
        it->Evaluate();

        const std::vector<int> &block_sizes = it->cost_function->parameter_block_sizes();
        for (int i = 0; i < static_cast<int>(block_sizes.size()); i++)
        {
            const ParameterBlockId addr = blockId(it->parameter_blocks[i]);
            int size = block_sizes[i];
            if (parameter_block_data.find(addr) == parameter_block_data.end())
            {
                parameter_block_data[addr] = std::vector<double>(it->parameter_blocks[i], it->parameter_blocks[i] + size);
            }
        }
    }
}

int MarginalizationInfo::localSize(int size) const
{
    return size == 7 ? 6 : size;
}

int MarginalizationInfo::globalSize(int size) const
{
    return size == 6 ? 7 : size;
}

void MarginalizationInfo::marginalize()
{
    local_block_index.clear();

    int pos = 0;
    for (const ParameterBlockId id : ordered_blocks)
    {
        if (drop_blocks.find(id) == drop_blocks.end())
            continue;
        local_block_index[id] = pos;
        pos += localSize(parameter_block_size.at(id));
    }

    m = pos;

    for (const ParameterBlockId id : ordered_blocks)
    {
        if (drop_blocks.find(id) != drop_blocks.end())
            continue;
        local_block_index[id] = pos;
        pos += localSize(parameter_block_size.at(id));
    }

    n = pos - m;

    Eigen::MatrixXd A(pos, pos);
    Eigen::VectorXd b(pos);
    A.setZero();
    b.setZero();

    const int num_threads = std::max(1, std::min(kNumThreads, static_cast<int>(factors.size())));
    std::vector<pthread_t> tids(num_threads);
    std::vector<ThreadsStruct> threadsstruct(num_threads);
    int i = 0;
    for (const auto &it : factors)
    {
        threadsstruct[i].sub_factors.push_back(it.get());
        i = (i + 1) % num_threads;
    }
    for (int i = 0; i < num_threads; i++)
    {
        threadsstruct[i].A = Eigen::MatrixXd::Zero(pos,pos);
        threadsstruct[i].b = Eigen::VectorXd::Zero(pos);
        threadsstruct[i].parameter_block_size = &parameter_block_size;
        threadsstruct[i].local_block_index = &local_block_index;
        int ret = pthread_create(&tids[i], NULL, ThreadsConstructA, static_cast<void *>(&threadsstruct[i]));
        if (ret != 0)
        {
            ROS_WARN("pthread_create error");
            ROS_BREAK();
        }
    }
    for (int i = num_threads - 1; i >= 0; i--)
    {
        pthread_join(tids[i], NULL);
        A += threadsstruct[i].A;
        b += threadsstruct[i].b;
    }

    Eigen::MatrixXd Amm = 0.5 * (A.block(0, 0, m, m) + A.block(0, 0, m, m).transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(Amm);

    Eigen::MatrixXd Amm_inv = saes.eigenvectors() * Eigen::VectorXd((saes.eigenvalues().array() > eps).select(saes.eigenvalues().array().inverse(), 0)).asDiagonal() * saes.eigenvectors().transpose();

    Eigen::VectorXd bmm = b.segment(0, m);
    Eigen::MatrixXd Amr = A.block(0, m, m, n);
    Eigen::MatrixXd Arm = A.block(m, 0, n, m);
    Eigen::MatrixXd Arr = A.block(m, m, n, n);
    Eigen::VectorXd brr = b.segment(m, n);
    A = Arr - Arm * Amm_inv * Amr;
    b = brr - Arm * Amm_inv * bmm;

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes2(A);
    Eigen::VectorXd S = Eigen::VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array(), 0));
    Eigen::VectorXd S_inv = Eigen::VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array().inverse(), 0));

    Eigen::VectorXd S_sqrt = S.cwiseSqrt();
    Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();

    linearized_jacobians = S_sqrt.asDiagonal() * saes2.eigenvectors().transpose();
    linearized_residuals = S_inv_sqrt.asDiagonal() * saes2.eigenvectors().transpose() * b;
}

std::vector<double *> MarginalizationInfo::getParameterBlocks(std::unordered_map<ParameterBlockId, double *> &addr_shift)
{
    std::vector<double *> keep_block_addr;
    keep_block_size.clear();
    keep_block_idx.clear();
    keep_block_data.clear();

    for (const ParameterBlockId id : ordered_blocks)
    {
        const int idx = local_block_index.at(id);
        if (idx >= m)
        {
            keep_block_size.push_back(parameter_block_size.at(id));
            keep_block_idx.push_back(idx);
            keep_block_data.push_back(parameter_block_data.at(id).data());
            keep_block_addr.push_back(addr_shift.at(id));
        }
    }
    sum_block_size = std::accumulate(std::begin(keep_block_size), std::end(keep_block_size), 0);

    return keep_block_addr;
}

MarginalizationFactor::MarginalizationFactor(MarginalizationInfo *_marginalization_info)
    : marginalization_info(_marginalization_info)
{
    for (auto it : marginalization_info->keep_block_size)
        mutable_parameter_block_sizes()->push_back(it);
    set_num_residuals(marginalization_info->n);
}

bool MarginalizationFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    int n = marginalization_info->n;
    int m = marginalization_info->m;
    Eigen::VectorXd dx(n);
    for (int i = 0; i < static_cast<int>(marginalization_info->keep_block_size.size()); i++)
    {
        int size = marginalization_info->keep_block_size[i];
        int idx = marginalization_info->keep_block_idx[i] - m;
        Eigen::Map<const Eigen::VectorXd> x(parameters[i], size);
        Eigen::Map<const Eigen::VectorXd> x0(marginalization_info->keep_block_data[i], size);
        if (size != 7)
            dx.segment(idx, size) = x - x0;
        else
        {
            dx.segment<3>(idx + 0) = x.head<3>() - x0.head<3>();
            dx.segment<3>(idx + 3) = 2.0 * Utility::positify(Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() * Eigen::Quaterniond(x(6), x(3), x(4), x(5))).vec();
            if (!((Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() * Eigen::Quaterniond(x(6), x(3), x(4), x(5))).w() >= 0))
            {
                dx.segment<3>(idx + 3) = 2.0 * -Utility::positify(Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() * Eigen::Quaterniond(x(6), x(3), x(4), x(5))).vec();
            }
        }
    }
    Eigen::Map<Eigen::VectorXd>(residuals, n) = marginalization_info->linearized_residuals + marginalization_info->linearized_jacobians * dx;
    if (jacobians)
    {
        for (int i = 0; i < static_cast<int>(marginalization_info->keep_block_size.size()); i++)
        {
            if (jacobians[i])
            {
                int size = marginalization_info->keep_block_size[i], local_size = marginalization_info->localSize(size);
                int idx = marginalization_info->keep_block_idx[i] - m;
                Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian(jacobians[i], n, size);
                jacobian.setZero();
                jacobian.leftCols(local_size) = marginalization_info->linearized_jacobians.middleCols(idx, local_size);
            }
        }
    }
    return true;
}
