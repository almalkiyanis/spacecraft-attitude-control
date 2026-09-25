#include "sac/attitude/Rotation.hpp"

#include <cmath>
#include <stdexcept>

namespace sac::attitude {

namespace {
// Below this norm, 'axis' is treated as numerically zero : no well-defined
// direction, so no rotation matrix can be  built from it.
constexpr double kMinAxisNorm = 1e-10;
} // namespace

Eigen::Matrix3d skew(const Eigen::Vector3d& u) {
    Eigen::Matrix3d S;
    S <<     0.0, -u.z(),  u.y(),
           u.z(),    0.0, -u.x(),
          -u.y(),  u.x(),    0.0;
    
    return S;

}

Eigen::Matrix3d axisAngleToRotationMatrix(const Eigen::Vector3d& axis, double angle) {
    const double axisNorm = axis.norm();
    if (axisNorm < kMinAxisNorm) {
        throw std::invalid_argument(
            "axisAngleToRotationMatrix: rotation axis must be non-zero.");
        
    }


    // Normalizing here (instead of requiring a pre-normalized unit vector)
    // means the function is well-defined for any non-zero 'axis, and the
    // magnitude of 'axis' never silently distorts the rotation angle.
    const Eigen::Vector3d u = axis / axisNorm;
    const Eigen::Matrix3d K = skew(u);

    // Rodrigues' formula: R = I + sin(theta) K + (1 - cos(theta)) K^2
    const Eigen::Matrix3d R = 
        Eigen::Matrix3d::Identity()
        + std::sin(angle) * K 
        + (1.0 - std::cos(angle)) * (K * K);

    return R;

}

} // namespace sac::attitude



