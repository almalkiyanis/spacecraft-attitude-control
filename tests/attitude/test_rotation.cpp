#include <gtest/gtest.h>

#include <cmath>

#include <Eigen/Dense>

#include "sac/attitude/Rotation.hpp"

using sac::attitude::axisAngleToRotationMatrix;
using sac::attitude::skew;

namespace {
constexpr double kTol = 1e-9;
}

// --- 5 mandatory tests --------------------------------------------------------

TEST(Rotation, ThetaZeroGivesIdentity) {
    const Eigen::Vector3d anyAxis(0.4, -1.1, 2.0); // direction should not matter
    const Eigen::Matrix3d R = axisAngleToRotationMatrix(anyAxis, 0.0);

    EXPECT_TRUE(R.isApprox(Eigen::Matrix3d::Identity(), kTol));

}

TEST(Rotation, IsOrthogonal_RtR_IsIdentity) {
const Eigen::Vector3d axis(1.0, 2.0, 3.0);  // deliberately not unit norm
const double angle = 0.7345;                // arbitrary, radians

const Eigen::Matrix3d R = axisAngleToRotationMatrix(axis, angle);

EXPECT_TRUE((R.transpose() * R).isApprox(Eigen::Matrix3d::Identity(), kTol));

}

TEST(Rotation, DeterminantIsOne) {
    const Eigen::Vector3d axis(-0.3, 0.9, 1.4);
    const double angle = -2.1; //negative angle on purpose

    const Eigen::Matrix3d R = axisAngleToRotationMatrix(axis, angle);

    EXPECT_NEAR(R.determinant(), 1.0, kTol);

}


TEST(Rotation, NinetyDegAboutZGivesExpectedMatrix) {
    const Eigen::Vector3d zAxis(0.0, 0.0, 1.0);
    const double angle = M_PI / 2.0;

    const Eigen::Matrix3d R = axisAngleToRotationMatrix(zAxis, angle);

    Eigen::Matrix3d expected;
    expected <<
        0.0, -1.0, 0.0,
        1.0,  0.0, 0.0,
        0.0,  0.0, 1.0;

EXPECT_TRUE(R.isApprox(expected, kTol));

// Physical cross_check: +90 deg about z should send e_x onto e_y
// (r-h-r), independent of how 'expected' above was typed in.
const Eigen::Vector3d ex(1.0, 0.0, 0.0);
const Eigen::Vector3d ey(0.0, 1.0, 0.0);
EXPECT_TRUE((R * ex).isApprox(ey, kTol));

}

TEST(Rotation, PreservesVectorNorm) {
    const Eigen::Vector3d axis(0.2, -0.5, 0.8);
    const double angle = 1.234;
    const Eigen::Vector3d v(3.0, -1.5, 2.2);

    const Eigen::Matrix3d R = axisAngleToRotationMatrix(axis, angle);

    EXPECT_NEAR((R * v).norm(), v.norm(), kTol);

}

// --- supporting tests ---------------------------------------------------------
// Not requested explicitly, but each one protects a specific design
// decision made in Rotation.cpp; flagging them for review rather than
// silently adding scope.

TEST(Rotation, AxisDoesNotNeedToBePreNormalized) {
    //Same direction, different magnitudes --> same R for the same angle.
    const double angle = 0.9;
    const Eigen::Matrix3d R1 = 
        axisAngleToRotationMatrix(Eigen::Vector3d(0.0, 0.0, 2.0), angle);
        const Eigen::Matrix3d R2 = 
        axisAngleToRotationMatrix(Eigen::Vector3d(0.0, 0.0, 0.5), angle);

    EXPECT_TRUE(R1.isApprox(R2, kTol));

}

TEST(Rotation, ZeroAxisThrows) {
    EXPECT_THROW(
        axisAngleToRotationMatrix(Eigen::Vector3d::Zero(), 1.0),
        std::invalid_argument
    );
    
}

TEST(Rotation, SkewMatchesCrossProduct) {
    const Eigen::Vector3d u(1.0, -2.0, 0.5);
    const Eigen::Vector3d v(0.3, 4.0, -1.1);

    EXPECT_TRUE((skew(u) * v).isApprox(u.cross(v), kTol));
}