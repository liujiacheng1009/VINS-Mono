#pragma once
#include "../utility/logging.h"
#include <eigen3/Eigen/Dense>

#include "../utility/utility.h"
#include "../parameters.h"
#include "integration_base.h"

#include <ceres/ceres.h>

class IMUFactor : public ceres::SizedCostFunction<15, 7, 9, 7, 9>
{
  private:
    static constexpr double kJacobianNumericalLimit = 1e8;

    struct PoseState
    {
        Eigen::Vector3d p;
        Eigen::Quaterniond q;
    };

    struct SpeedBiasState
    {
        Eigen::Vector3d v;
        Eigen::Vector3d ba;
        Eigen::Vector3d bg;
    };

    static PoseState parsePose(const double *parameters)
    {
        PoseState state;
        state.p = Eigen::Vector3d(parameters[0], parameters[1], parameters[2]);
        state.q = Eigen::Quaterniond(parameters[6], parameters[3], parameters[4], parameters[5]);
        return state;
    }

    static SpeedBiasState parseSpeedBias(const double *parameters)
    {
        SpeedBiasState state;
        state.v = Eigen::Vector3d(parameters[0], parameters[1], parameters[2]);
        state.ba = Eigen::Vector3d(parameters[3], parameters[4], parameters[5]);
        state.bg = Eigen::Vector3d(parameters[6], parameters[7], parameters[8]);
        return state;
    }

  public:
    IMUFactor() = delete;
    IMUFactor(IntegrationBase* _pre_integration):pre_integration_(_pre_integration)
    {
    }
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {

        // parameters[0]: pose i, parameters[1]: speed/bias i,
        // parameters[2]: pose j, parameters[3]: speed/bias j.
        const PoseState pose_i = parsePose(parameters[0]);
        const SpeedBiasState speed_bias_i = parseSpeedBias(parameters[1]);
        const PoseState pose_j = parsePose(parameters[2]);
        const SpeedBiasState speed_bias_j = parseSpeedBias(parameters[3]);

        const Eigen::Vector3d &Pi = pose_i.p;
        const Eigen::Quaterniond &Qi = pose_i.q;
        const Eigen::Vector3d &Vi = speed_bias_i.v;
        const Eigen::Vector3d &Bai = speed_bias_i.ba;
        const Eigen::Vector3d &Bgi = speed_bias_i.bg;

        const Eigen::Vector3d &Pj = pose_j.p;
        const Eigen::Quaterniond &Qj = pose_j.q;
        const Eigen::Vector3d &Vj = speed_bias_j.v;
        const Eigen::Vector3d &Baj = speed_bias_j.ba;
        const Eigen::Vector3d &Bgj = speed_bias_j.bg;

        Eigen::Map<Eigen::Matrix<double, 15, 1>> residual(residuals);
        residual = pre_integration_->evaluate(Pi, Qi, Vi, Bai, Bgi,
                                            Pj, Qj, Vj, Baj, Bgj);

        Eigen::Matrix<double, 15, 15> sqrt_info =
            Eigen::LLT<Eigen::Matrix<double, 15, 15>>(pre_integration_->covariance.inverse()).matrixL().transpose();
        residual = sqrt_info * residual;

        if (jacobians)
        {
            double sum_dt = pre_integration_->sum_dt;
            Eigen::Matrix3d dp_dba = pre_integration_->jacobian.template block<3, 3>(O_P, O_BA);
            Eigen::Matrix3d dp_dbg = pre_integration_->jacobian.template block<3, 3>(O_P, O_BG);

            Eigen::Matrix3d dq_dbg = pre_integration_->jacobian.template block<3, 3>(O_R, O_BG);

            Eigen::Matrix3d dv_dba = pre_integration_->jacobian.template block<3, 3>(O_V, O_BA);
            Eigen::Matrix3d dv_dbg = pre_integration_->jacobian.template block<3, 3>(O_V, O_BG);

            if (pre_integration_->jacobian.maxCoeff() > kJacobianNumericalLimit ||
                pre_integration_->jacobian.minCoeff() < -kJacobianNumericalLimit)
            {
                ROS_WARN("numerical unstable in preintegration");
            }

            // jacobians[0]: pose i.
            if (jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
                jacobian_pose_i.setZero();

                jacobian_pose_i.block<3, 3>(O_P, O_P) = -Qi.inverse().toRotationMatrix();
                jacobian_pose_i.block<3, 3>(O_P, O_R) = Utility::skewSymmetric(Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt));

                Eigen::Quaterniond corrected_delta_q = pre_integration_->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration_->linearized_bg));
                jacobian_pose_i.block<3, 3>(O_R, O_R) =
                    -(Utility::Qleft(Qj.inverse() * Qi) * Utility::Qright(corrected_delta_q)).bottomRightCorner<3, 3>();

                jacobian_pose_i.block<3, 3>(O_V, O_R) = Utility::skewSymmetric(Qi.inverse() * (G * sum_dt + Vj - Vi));

                jacobian_pose_i = sqrt_info * jacobian_pose_i;

                if (jacobian_pose_i.maxCoeff() > kJacobianNumericalLimit ||
                    jacobian_pose_i.minCoeff() < -kJacobianNumericalLimit)
                {
                    ROS_WARN("numerical unstable in preintegration");
                }
            }
            // jacobians[1]: speed/bias i.
            if (jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> jacobian_speedbias_i(jacobians[1]);
                jacobian_speedbias_i.setZero();
                jacobian_speedbias_i.block<3, 3>(O_P, O_V - O_V) = -Qi.inverse().toRotationMatrix() * sum_dt;
                jacobian_speedbias_i.block<3, 3>(O_P, O_BA - O_V) = -dp_dba;
                jacobian_speedbias_i.block<3, 3>(O_P, O_BG - O_V) = -dp_dbg;

                jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) =
                    -Utility::Qleft(Qj.inverse() * Qi * pre_integration_->delta_q).bottomRightCorner<3, 3>() * dq_dbg;

                jacobian_speedbias_i.block<3, 3>(O_V, O_V - O_V) = -Qi.inverse().toRotationMatrix();
                jacobian_speedbias_i.block<3, 3>(O_V, O_BA - O_V) = -dv_dba;
                jacobian_speedbias_i.block<3, 3>(O_V, O_BG - O_V) = -dv_dbg;

                jacobian_speedbias_i.block<3, 3>(O_BA, O_BA - O_V) = -Eigen::Matrix3d::Identity();

                jacobian_speedbias_i.block<3, 3>(O_BG, O_BG - O_V) = -Eigen::Matrix3d::Identity();

                jacobian_speedbias_i = sqrt_info * jacobian_speedbias_i;
            }
            // jacobians[2]: pose j.
            if (jacobians[2])
            {
                Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[2]);
                jacobian_pose_j.setZero();

                jacobian_pose_j.block<3, 3>(O_P, O_P) = Qi.inverse().toRotationMatrix();

                Eigen::Quaterniond corrected_delta_q = pre_integration_->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration_->linearized_bg));
                jacobian_pose_j.block<3, 3>(O_R, O_R) =
                    Utility::Qleft(corrected_delta_q.inverse() * Qi.inverse() * Qj).bottomRightCorner<3, 3>();

                jacobian_pose_j = sqrt_info * jacobian_pose_j;
            }
            // jacobians[3]: speed/bias j.
            if (jacobians[3])
            {
                Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> jacobian_speedbias_j(jacobians[3]);
                jacobian_speedbias_j.setZero();

                jacobian_speedbias_j.block<3, 3>(O_V, O_V - O_V) = Qi.inverse().toRotationMatrix();

                jacobian_speedbias_j.block<3, 3>(O_BA, O_BA - O_V) = Eigen::Matrix3d::Identity();

                jacobian_speedbias_j.block<3, 3>(O_BG, O_BG - O_V) = Eigen::Matrix3d::Identity();

                jacobian_speedbias_j = sqrt_info * jacobian_speedbias_j;
            }
        }

        return true;
    }

    IntegrationBase* pre_integration_;

};

