#pragma once

#include <Eigen/Dense>

namespace sac::attitude {


// Skew-symmetric (cross-product) matrix associated with a 3-vector u, such
// that skew(u) * v = u.cross(v) for every v. Exposed on its own because it
// is a useful, independently testable building block (and will be reused)
// later, e.g. in the angular velocity kinematics).
Eigen::Matrix3d skew(
    const Eigen::Vector3d& u
);

// Builds the rotation matrix R(u, theta) that performs an active rotation
// of 'angle' radians about 'axis', following  the right-hand rule, via
// Rodrigues' formula:
//
//     R = I + sin(theta) [u]_x + (1 - cos(theta)) [u]_x^2
//
// 'axis' does not need to be pre-normalized: it is normalized internally.
// This is a deliberate choice (see Rotation.cpp) rather than requiring the 
// caller to guarantee a unit vector.
//
// Throws std::invalid_argument if 'axis' is (numerically) the zero vector,
// since no rotation axis can be defined in that case.
Eigen::Matrix3d axisAngleToRotationMatrix(
    const Eigen::Vector3d& axis, 
    double angle
);

} //namespace sac::attitude