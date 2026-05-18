#include "imu_factor.h"

#include "../utility/logging.h"

namespace
{

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

PoseState parsePose(const double *parameters)
{
    PoseState state;
    state.p = Eigen::Vector3d(parameters[0], parameters[1], parameters[2]);
    state.q = Eigen::Quaterniond(parameters[6], parameters[3], parameters[4], parameters[5]);
    return state;
}

SpeedBiasState parseSpeedBias(const double *parameters)
{
    SpeedBiasState state;
    state.v = Eigen::Vector3d(parameters[0], parameters[1], parameters[2]);
    state.ba = Eigen::Vector3d(parameters[3], parameters[4], parameters[5]);
    state.bg = Eigen::Vector3d(parameters[6], parameters[7], parameters[8]);
    return state;
}

}  // namespace

IntegrationBase::IntegrationBase(const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                                 const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg)
    : acc_0{_acc_0}, gyr_0{_gyr_0}, linearized_acc{_acc_0}, linearized_gyr{_gyr_0},
      linearized_ba{_linearized_ba}, linearized_bg{_linearized_bg},
      jacobian{Eigen::Matrix<double, 15, 15>::Identity()},
      covariance{Eigen::Matrix<double, 15, 15>::Zero()},
      sum_dt{0.0}, delta_p{Eigen::Vector3d::Zero()},
      delta_q{Eigen::Quaterniond::Identity()}, delta_v{Eigen::Vector3d::Zero()}
{
    noise = Eigen::Matrix<double, 18, 18>::Zero();
    noise.block<3, 3>(0, 0) = (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(3, 3) = (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(6, 6) = (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(9, 9) = (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(12, 12) = (ACC_W * ACC_W) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(15, 15) = (GYR_W * GYR_W) * Eigen::Matrix3d::Identity();
}

void IntegrationBase::push_back(double dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr)
{
    dt_buf.push_back(dt);
    acc_buf.push_back(acc);
    gyr_buf.push_back(gyr);
    propagate(dt, acc, gyr);
}

void IntegrationBase::repropagate(const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg)
{
    sum_dt = 0.0;
    acc_0 = linearized_acc;
    gyr_0 = linearized_gyr;
    delta_p.setZero();
    delta_q.setIdentity();
    delta_v.setZero();
    linearized_ba = _linearized_ba;
    linearized_bg = _linearized_bg;
    jacobian.setIdentity();
    covariance.setZero();
    for (int i = 0; i < static_cast<int>(dt_buf.size()); i++)
        propagate(dt_buf[i], acc_buf[i], gyr_buf[i]);
}

void IntegrationBase::midPointIntegration(double _dt,
                                          const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                                          const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
                                          const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q,
                                          const Eigen::Vector3d &delta_v,
                                          const Eigen::Vector3d &linearized_ba,
                                          const Eigen::Vector3d &linearized_bg,
                                          Eigen::Vector3d &result_delta_p,
                                          Eigen::Quaterniond &result_delta_q,
                                          Eigen::Vector3d &result_delta_v,
                                          Eigen::Vector3d &result_linearized_ba,
                                          Eigen::Vector3d &result_linearized_bg,
                                          bool update_jacobian)
{
    Eigen::Vector3d un_acc_0 = delta_q * (_acc_0 - linearized_ba);
    Eigen::Vector3d un_gyr = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
    result_delta_q = delta_q * Eigen::Quaterniond(1, un_gyr(0) * _dt / 2, un_gyr(1) * _dt / 2, un_gyr(2) * _dt / 2);
    Eigen::Vector3d un_acc_1 = result_delta_q * (_acc_1 - linearized_ba);
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    result_delta_p = delta_p + delta_v * _dt + 0.5 * un_acc * _dt * _dt;
    result_delta_v = delta_v + un_acc * _dt;
    result_linearized_ba = linearized_ba;
    result_linearized_bg = linearized_bg;

    if (update_jacobian)
    {
        Eigen::Vector3d w_x = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
        Eigen::Vector3d a_0_x = _acc_0 - linearized_ba;
        Eigen::Vector3d a_1_x = _acc_1 - linearized_ba;
        Eigen::Matrix3d R_w_x, R_a_0_x, R_a_1_x;

        R_w_x << 0, -w_x(2), w_x(1),
            w_x(2), 0, -w_x(0),
            -w_x(1), w_x(0), 0;
        R_a_0_x << 0, -a_0_x(2), a_0_x(1),
            a_0_x(2), 0, -a_0_x(0),
            -a_0_x(1), a_0_x(0), 0;
        R_a_1_x << 0, -a_1_x(2), a_1_x(1),
            a_1_x(2), 0, -a_1_x(0),
            -a_1_x(1), a_1_x(0), 0;

        Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Zero();
        F.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        F.block<3, 3>(0, 3) = -0.25 * delta_q.toRotationMatrix() * R_a_0_x * _dt * _dt +
                              -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x *
                                  (Eigen::Matrix3d::Identity() - R_w_x * _dt) * _dt * _dt;
        F.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * _dt;
        F.block<3, 3>(0, 9) = -0.25 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt * _dt;
        F.block<3, 3>(0, 12) = -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * _dt * -_dt;
        F.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() - R_w_x * _dt;
        F.block<3, 3>(3, 12) = -1.0 * Eigen::Matrix3d::Identity() * _dt;
        F.block<3, 3>(6, 3) = -0.5 * delta_q.toRotationMatrix() * R_a_0_x * _dt +
                              -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x *
                                  (Eigen::Matrix3d::Identity() - R_w_x * _dt) * _dt;
        F.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();
        F.block<3, 3>(6, 9) = -0.5 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt;
        F.block<3, 3>(6, 12) = -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * -_dt;
        F.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity();
        F.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity();

        Eigen::Matrix<double, 15, 18> V = Eigen::Matrix<double, 15, 18>::Zero();
        V.block<3, 3>(0, 0) = 0.25 * delta_q.toRotationMatrix() * _dt * _dt;
        V.block<3, 3>(0, 3) = 0.25 * -result_delta_q.toRotationMatrix() * R_a_1_x * _dt * _dt * 0.5 * _dt;
        V.block<3, 3>(0, 6) = 0.25 * result_delta_q.toRotationMatrix() * _dt * _dt;
        V.block<3, 3>(0, 9) = V.block<3, 3>(0, 3);
        V.block<3, 3>(3, 3) = 0.5 * Eigen::Matrix3d::Identity() * _dt;
        V.block<3, 3>(3, 9) = 0.5 * Eigen::Matrix3d::Identity() * _dt;
        V.block<3, 3>(6, 0) = 0.5 * delta_q.toRotationMatrix() * _dt;
        V.block<3, 3>(6, 3) = 0.5 * -result_delta_q.toRotationMatrix() * R_a_1_x * _dt * 0.5 * _dt;
        V.block<3, 3>(6, 6) = 0.5 * result_delta_q.toRotationMatrix() * _dt;
        V.block<3, 3>(6, 9) = V.block<3, 3>(6, 3);
        V.block<3, 3>(9, 12) = Eigen::Matrix3d::Identity() * _dt;
        V.block<3, 3>(12, 15) = Eigen::Matrix3d::Identity() * _dt;

        jacobian = F * jacobian;
        covariance = F * covariance * F.transpose() + V * noise * V.transpose();
    }
}

void IntegrationBase::propagate(double _dt, const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1)
{
    dt = _dt;
    acc_1 = _acc_1;
    gyr_1 = _gyr_1;
    Eigen::Vector3d result_delta_p;
    Eigen::Quaterniond result_delta_q;
    Eigen::Vector3d result_delta_v;
    Eigen::Vector3d result_linearized_ba;
    Eigen::Vector3d result_linearized_bg;

    midPointIntegration(_dt, acc_0, gyr_0, _acc_1, _gyr_1, delta_p, delta_q, delta_v,
                        linearized_ba, linearized_bg,
                        result_delta_p, result_delta_q, result_delta_v,
                        result_linearized_ba, result_linearized_bg, true);

    delta_p = result_delta_p;
    delta_q = result_delta_q;
    delta_v = result_delta_v;
    linearized_ba = result_linearized_ba;
    linearized_bg = result_linearized_bg;
    delta_q.normalize();
    sum_dt += dt;
    acc_0 = acc_1;
    gyr_0 = gyr_1;
}

Eigen::Matrix<double, 15, 1> IntegrationBase::computeResidual(
    const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, const Eigen::Vector3d &Vi,
    const Eigen::Vector3d &Bai, const Eigen::Vector3d &Bgi,
    const Eigen::Vector3d &Pj, const Eigen::Quaterniond &Qj, const Eigen::Vector3d &Vj,
    const Eigen::Vector3d &Baj, const Eigen::Vector3d &Bgj)
{
    Eigen::Matrix<double, 15, 1> residuals;

    Eigen::Matrix3d dp_dba = jacobian.block<3, 3>(O_P, O_BA);
    Eigen::Matrix3d dp_dbg = jacobian.block<3, 3>(O_P, O_BG);

    Eigen::Matrix3d dq_dbg = jacobian.block<3, 3>(O_R, O_BG);

    Eigen::Matrix3d dv_dba = jacobian.block<3, 3>(O_V, O_BA);
    Eigen::Matrix3d dv_dbg = jacobian.block<3, 3>(O_V, O_BG);

    Eigen::Vector3d dba = Bai - linearized_ba;
    Eigen::Vector3d dbg = Bgi - linearized_bg;

    Eigen::Quaterniond corrected_delta_q = delta_q * Utility::deltaQ(dq_dbg * dbg);
    Eigen::Vector3d corrected_delta_v = delta_v + dv_dba * dba + dv_dbg * dbg;
    Eigen::Vector3d corrected_delta_p = delta_p + dp_dba * dba + dp_dbg * dbg;

    residuals.block<3, 1>(O_P, 0) = Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt) - corrected_delta_p;
    residuals.block<3, 1>(O_R, 0) = 2 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();
    residuals.block<3, 1>(O_V, 0) = Qi.inverse() * (G * sum_dt + Vj - Vi) - corrected_delta_v;
    residuals.block<3, 1>(O_BA, 0) = Baj - Bai;
    residuals.block<3, 1>(O_BG, 0) = Bgj - Bgi;
    return residuals;
}

bool IMUFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
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
    residual = pre_integration_->computeResidual(Pi, Qi, Vi, Bai, Bgi,
                                                 Pj, Qj, Vj, Baj, Bgj);

    Eigen::Matrix<double, 15, 15> sqrt_info =
        Eigen::LLT<Eigen::Matrix<double, 15, 15>>(pre_integration_->covariance.inverse()).matrixL().transpose();
    residual = sqrt_info * residual;

    if (jacobians)
    {
        double sum_dt = pre_integration_->sum_dt;
        Eigen::Matrix3d dp_dba = pre_integration_->jacobian.block<3, 3>(O_P, O_BA);
        Eigen::Matrix3d dp_dbg = pre_integration_->jacobian.block<3, 3>(O_P, O_BG);

        Eigen::Matrix3d dq_dbg = pre_integration_->jacobian.block<3, 3>(O_R, O_BG);

        Eigen::Matrix3d dv_dba = pre_integration_->jacobian.block<3, 3>(O_V, O_BA);
        Eigen::Matrix3d dv_dbg = pre_integration_->jacobian.block<3, 3>(O_V, O_BG);

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
            jacobian_pose_i.block<3, 3>(O_P, O_R) =
                Utility::skewSymmetric(Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt));

            Eigen::Quaterniond corrected_delta_q =
                pre_integration_->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration_->linearized_bg));
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

            Eigen::Quaterniond corrected_delta_q =
                pre_integration_->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration_->linearized_bg));
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
