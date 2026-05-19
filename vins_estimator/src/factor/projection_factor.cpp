#include "projection_factor.h"

#include "../parameters.h"
#include "../utility/utility.h"

namespace
{

// Build an orthonormal 2x3 tangent basis around the (already-observed) ray
// pts_j on the unit sphere. The two rows of the returned matrix span the
// tangent plane at pts_j.normalized(), so multiplying a 3D vector by this
// matrix projects it onto that plane.
//
// Only used when UNIT_SPHERE_ERROR is enabled; for the default pinhole
// formulation tangent_base is left zero.
Eigen::Matrix<double, 2, 3> computeTangentBase(const Eigen::Vector3d &pts_j)
{
    Eigen::Matrix<double, 2, 3> base;
    base.setZero();

    const Eigen::Vector3d a = pts_j.normalized();
    Eigen::Vector3d tmp(0.0, 0.0, 1.0);
    // Robust degenerate-axis check: the original code compared Vector3d via
    // operator== (exact float equality) which almost never triggers; the
    // intent is "if a is parallel to the chosen seed axis, pick a different
    // seed". A tolerance of 1e-6 covers ~5e-4 rad of misalignment which is
    // tighter than any expected numerical jitter from .normalized().
    if ((a - tmp).norm() < 1e-6)
        tmp << 1.0, 0.0, 0.0;

    const Eigen::Vector3d b1 = (tmp - a * (a.transpose() * tmp)).normalized();
    const Eigen::Vector3d b2 = a.cross(b1);
    base.row(0) = b1.transpose();
    base.row(1) = b2.transpose();
    return base;
}

}  // namespace

Eigen::Matrix2d ProjectionFactor::sqrt_info;

ProjectionFactor::ProjectionFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j)
    : pts_i(_pts_i), pts_j(_pts_j)
{
    tangent_base.setZero();
#ifdef UNIT_SPHERE_ERROR
    tangent_base = computeTangentBase(pts_j);
#endif
}

bool ProjectionFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    const Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    const Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    const Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    const Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    const Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
    const Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

    const double inv_dep_i = parameters[3][0];

    const Eigen::Vector3d pts_camera_i = pts_i / inv_dep_i;
    const Eigen::Vector3d pts_imu_i    = qic * pts_camera_i + tic;
    const Eigen::Vector3d pts_w        = Qi * pts_imu_i + Pi;
    const Eigen::Vector3d pts_imu_j    = Qj.inverse() * (pts_w - Pj);
    const Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);

    Eigen::Map<Eigen::Vector2d> residual(residuals);

#ifdef UNIT_SPHERE_ERROR
    residual = tangent_base * (pts_camera_j.normalized() - pts_j.normalized());
#else
    const double dep_j = pts_camera_j.z();
    residual = (pts_camera_j / dep_j).head<2>() - pts_j.head<2>();
#endif

    residual = sqrt_info * residual;

    if (jacobians)
    {
        const Eigen::Matrix3d Ri  = Qi.toRotationMatrix();
        const Eigen::Matrix3d Rj  = Qj.toRotationMatrix();
        const Eigen::Matrix3d ric = qic.toRotationMatrix();

        // Common sub-expressions reused by multiple jacobian blocks below.
        const Eigen::Matrix3d ric_T          = ric.transpose();
        const Eigen::Matrix3d Rj_T           = Rj.transpose();
        const Eigen::Matrix3d ric_T_Rj_T     = ric_T * Rj_T;
        const Eigen::Matrix3d ric_T_Rj_T_Ri  = ric_T_Rj_T * Ri;

        Eigen::Matrix<double, 2, 3> reduce;
#ifdef UNIT_SPHERE_ERROR
        const double norm = pts_camera_j.norm();
        const double x1 = pts_camera_j(0);
        const double x2 = pts_camera_j(1);
        const double x3 = pts_camera_j(2);
        const double n3 = norm * norm * norm;
        Eigen::Matrix3d norm_jaco;
        norm_jaco <<
            1.0 / norm - x1 * x1 / n3, -x1 * x2 / n3,             -x1 * x3 / n3,
            -x1 * x2 / n3,              1.0 / norm - x2 * x2 / n3, -x2 * x3 / n3,
            -x1 * x3 / n3,             -x2 * x3 / n3,              1.0 / norm - x3 * x3 / n3;
        reduce = tangent_base * norm_jaco;
#else
        reduce <<
            1.0 / dep_j, 0.0,         -pts_camera_j(0) / (dep_j * dep_j),
            0.0,         1.0 / dep_j, -pts_camera_j(1) / (dep_j * dep_j);
#endif
        reduce = sqrt_info * reduce;

        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);

            Eigen::Matrix<double, 3, 6> jaco_i;
            jaco_i.leftCols<3>()  = ric_T_Rj_T;
            jaco_i.rightCols<3>() = ric_T_Rj_T * Ri * -Utility::skewSymmetric(pts_imu_i);

            jacobian_pose_i.leftCols<6>()  = reduce * jaco_i;
            jacobian_pose_i.rightCols<1>().setZero();
        }

        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);

            Eigen::Matrix<double, 3, 6> jaco_j;
            jaco_j.leftCols<3>()  = -ric_T_Rj_T;
            jaco_j.rightCols<3>() =  ric_T * Utility::skewSymmetric(pts_imu_j);

            jacobian_pose_j.leftCols<6>()  = reduce * jaco_j;
            jacobian_pose_j.rightCols<1>().setZero();
        }

        if (jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_ex_pose(jacobians[2]);

            Eigen::Matrix<double, 3, 6> jaco_ex;
            jaco_ex.leftCols<3>() = ric_T * (Rj_T * Ri - Eigen::Matrix3d::Identity());
            const Eigen::Matrix3d tmp_r = ric_T_Rj_T_Ri * ric;
            jaco_ex.rightCols<3>() = -tmp_r * Utility::skewSymmetric(pts_camera_i) +
                                     Utility::skewSymmetric(tmp_r * pts_camera_i) +
                                     Utility::skewSymmetric(ric_T * (Rj_T * (Ri * tic + Pi - Pj) - tic));

            jacobian_ex_pose.leftCols<6>()  = reduce * jaco_ex;
            jacobian_ex_pose.rightCols<1>().setZero();
        }

        if (jacobians[3])
        {
            Eigen::Map<Eigen::Vector2d> jacobian_feature(jacobians[3]);
            jacobian_feature = reduce * ric_T_Rj_T_Ri * ric * pts_i * -1.0 / (inv_dep_i * inv_dep_i);
        }
    }

    return true;
}
