#pragma once

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

// Visual reprojection residual between two keyframes for a single feature.
//
// Parameter blocks (ceres layout):
//   parameters[0]: pose_i (7) = [px, py, pz, qx, qy, qz, qw]
//   parameters[1]: pose_j (7)
//   parameters[2]: extrinsic T_bc (7)
//   parameters[3]: inverse depth in frame i (1)
//
// Residual is 2D: difference between the predicted normalized image-plane
// coordinate in frame j and the observation pts_j (in the same coordinates).
// When UNIT_SPHERE_ERROR is defined the residual is taken on the unit sphere
// using tangent_base built around pts_j.
//
// pts_i / pts_j are stored as Vector3d but represent normalized image-plane
// coordinates with the convention (x/z, y/z, 1).
class ProjectionFactor : public ceres::SizedCostFunction<2, 7, 7, 7, 1>
{
  public:
    ProjectionFactor() = delete;
    explicit ProjectionFactor(const Eigen::Vector3d &pts_i, const Eigen::Vector3d &pts_j);

    bool Evaluate(double const *const *parameters,
                  double *residuals,
                  double **jacobians) const override;

    Eigen::Vector3d pts_i, pts_j;
    Eigen::Matrix<double, 2, 3> tangent_base;

    // Square-root information matrix applied uniformly to all visual residuals.
    // Must be set externally (Estimator::setParameter()) before any Evaluate()
    // call: it defaults to zero, which would silently kill all residuals.
    static Eigen::Matrix2d sqrt_info;
};
