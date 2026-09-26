#include "sac/attitude/Quaternion.hpp"
#include "sac/attitude/Rotation.hpp" // reuses skew() for toRtationMatrix()


#include <gtest/gtest.h>

#include <cmath>

#include <Eigen/Dense>

using sac::attitude::axisAngleToRotationMatrix;
using sac::attitude::Quaternion;

namespace {
constexpr double kTol = 1e-9;
}

// --- construction / accessors -----------------------------------------------------

TEST(Quaternion, ComponentsRoundTrip) {
    const Quaternion q(1.0, 2.0, -3.0, 0.5);

    EXPECT_DOUBLE_EQ(q.q0(), 1.0);
    EXPECT_DOUBLE_EQ(q.q1(), 2.0);
    EXPECT_DOUBLE_EQ(q.q2(), -3.0);
    EXPECT_DOUBLE_EQ(q.q3(), 0.5);

    const Eigen::Vector3d expectedVec(2.0, -3.0, 0.5);
    EXPECT_TRUE(q.vec().isApprox(expectedVec, kTol));

    const Eigen::Vector4d expectedCoeffs(1.0, 2.0, -3.0, 0.5);
    EXPECT_TRUE(q.coeffs().isApprox(expectedCoeffs, kTol));
}

TEST(Quaternion, VectorConstructorMatchesComponentConstructor) {
    const Eigen::Vector4d v(0.1, 0.2, 0.3, 0.4);
    const Quaternion fromVec(v);
    const Quaternion fromComponents(0.1, 0.2, 0.3, 0.4);

    EXPECT_TRUE(fromVec.isApprox(fromComponents, kTol));
}

// --- Identity ---------------------------------------------------------------------

TEST(Quaternion, IdentityIsOneZeroZeroZero) {
    const Quaternion id = Quaternion::Identity();

    EXPECT_DOUBLE_EQ(id.q0(), 1.0);
    EXPECT_DOUBLE_EQ(id.q1(), 0.0);
    EXPECT_DOUBLE_EQ(id.q2(), 0.0);
    EXPECT_DOUBLE_EQ(id.q3(), 0.0);
}

TEST(Quaternion, IdentityIsUnit) {
    EXPECT_TRUE(Quaternion::Identity().isUnit());
}

TEST(Quaternion, IdentityIsLeftAndRightMultiplicativeIdentity) {
    const Quaternion q(0.4, -0.1, 0.2, 0.3);  // arbitary, not necessarily unit
    const Quaternion id = Quaternion::Identity();

    EXPECT_TRUE((id * q).isApprox(q, kTol));
    EXPECT_TRUE((q * id).isApprox(q, kTol));
}

// --- norm / squaredNorm / isUnit ------------------------------------------------

TEST(Quaternion, SquaredNormMatchesNormSquared) {
    const Quaternion q(1.0, 2.0, -1.0, 0.5);

    EXPECT_NEAR(q.squaredNorm(), q.norm() * q.norm(), kTol);
    EXPECT_NEAR(q.squaredNorm(), 1.0 + 4.0 + 1.0 + 0.25, kTol);
}

TEST(Quaternion, NormOfUnitQuaternionIsOne) {
    // cos(theta/2), sin(theta/2)*axis for an arbitrary axis/angle: unit by
    // construction (cos^2 + sin^2 * ||axis||^2 = 1 for a unit axis).
    const double theta = 0.9;
    Eigen::Vector3d axis(0.2, -0.4, 0.8);
    axis.normalize();
    const Quaternion q(std::cos(theta / 2.0),
                        std::sin(theta / 2.0) * axis.x(),
                        std::sin(theta / 2.0) * axis.y(),
                        std::sin(theta / 2.0) * axis.z());

    EXPECT_NEAR(q.norm(), 1.0, kTol);
    EXPECT_TRUE(q.isUnit());
                        
}

TEST(Quaternion, NonUnitQuaternionIsNotUnit) {
    const Quaternion q(1.0, 1.0, 0.0, 0.0); // norm = sqrt(2)
    EXPECT_FALSE(q.isUnit());
}

// --- normalized() -----------------------------------------------------------------

TEST(Quaternion, NormalizedHasUnitNorm) {
    const Quaternion q(2.0, 0.0, 0.0, 0.0);
    const Quaternion n = q.normalized();

    EXPECT_NEAR(n.norm(), 1.0, kTol);
    EXPECT_TRUE(n.isApprox(Quaternion(1.0, 0.0, 0.0, 0.0), kTol));
}

TEST(Quaternion, NormalizedZeroThrows) {
    const Quaternion zero(0.0, 0.0, 0.0, 0.0);
    EXPECT_THROW(zero.normalized(), std::invalid_argument);
}

// --- conjugate() -----------------------------------------------------------------

TEST(Quaternion, ConjugateNegatesVectorPartOnly) {
    const Quaternion q(1.0, 2.0, -3.0, 0.5);
    const Quaternion c = q.conjugate();

    EXPECT_DOUBLE_EQ(c.q0(), 1.0);
    EXPECT_DOUBLE_EQ(c.q1(), -2.0);
    EXPECT_DOUBLE_EQ(c.q2(), 3.0);
    EXPECT_DOUBLE_EQ(c.q3(), -0.5);
}

TEST(Quaternion, ConjugateOfNonUnitQuaternionIsDefined) {
    // conjugate() must not require/assume unit norm.
    const Quaternion q(2.0, 4.0, 0.0, 0.0); // norm = sqrt(20), not unit
    const Quaternion c = q.conjugate();

    EXPECT_TRUE(c.isApprox(Quaternion(2.0, -4.0, 0.0, 0.0)));
}

// --- inverse(): general formula q* / ||q||^2 -------------------------------------

TEST(Quaternion, InverseOfUnitQuaternionEqualsConjugate) {
    const double theta = 1.1;
    Eigen::Vector3d axis(0.3, 0.6, -0.1);
    axis.normalize();
    const Quaternion q(std::cos(theta / 2.0 ),
                        std::sin(theta / 2.0) * axis.x(),
                        std::sin(theta / 2.0) * axis.y(),
                        std::sin(theta / 2.0) * axis.z());

    ASSERT_TRUE(q.isUnit());

    EXPECT_TRUE(q.inverse().isApprox(q.conjugate(), kTol));
}

TEST(Quaternion, InverseWorksForNonUNitQuaternion) {
    // General property being tested here: q^{-1} is defined via
    // q* / ||q||^2 for ANY non-zero quaternion, not only unit ones, and it
    // is a genuine two-sided inverse under the Hamilton product regardless.
    const Quaternion q(1.0, 2.0, -1.0, 0.5);
    ASSERT_FALSE(q.isUnit());

    const Quaternion qInv = q.inverse();
    const Quaternion identity = Quaternion::Identity();

    EXPECT_TRUE((q * qInv).isApprox(identity, kTol));
    EXPECT_TRUE((qInv * q).isApprox(identity, kTol));
}

TEST(Quaternion, QTimesInverseIsIdentity_UnitCase) {
    // q (x) q^{-1} = 1, explicitly for a unit quaternion too (the case
    // that will matter most once this feeds attitude/control code).
    const double theta = M_PI / 3.0;
    Eigen::Vector3d axis(0.0, 0.0, 1.0);
    const Quaternion q(std::cos(theta / 2.0), 0.0, 0.0,
                        std::sin(theta / 2.0) * axis.z());
 
    const Quaternion result = q * q.inverse();
    EXPECT_TRUE(result.isApprox(Quaternion::Identity(), kTol));
}
 
TEST(Quaternion, InverseOfZeroThrows) {
    const Quaternion zero(0.0, 0.0, 0.0, 0.0);
    EXPECT_THROW(zero.inverse(), std::invalid_argument);
}
 
// --- Hamilton product (operator*) ----------------------------------------
 
TEST(Quaternion, ProductIsNotCommutativeInGeneral) {
    const Quaternion a(0.1, 0.2, 0.3, 0.4);
    const Quaternion b(0.5, -0.1, 0.2, 0.05);
 
    const Quaternion ab = a * b;
    const Quaternion ba = b * a;
 
    EXPECT_FALSE(ab.isApprox(ba, kTol));
}
 
TEST(Quaternion, ProductMatchesHandComputedExample) {
    // Cross-check against an independently computed Hamilton product
    // (verified numerically before writing this test), to catch sign or
    // term-ordering mistakes in the implementation rather than relying
    // only on algebraic identities that could hide a consistent-but-wrong
    // convention.
    const Quaternion a(0.1, 0.2, 0.3, 0.4);
    const Quaternion b(0.5, -0.1, 0.2, 0.05);
 
    const Quaternion expectedAB(-0.01, 0.025, 0.12, 0.275);
    const Quaternion expectedBA(-0.01, 0.155, 0.22, 0.135);
 
    EXPECT_TRUE((a * b).isApprox(expectedAB, 1e-6));
    EXPECT_TRUE((b * a).isApprox(expectedBA, 1e-6));
}
 
// --- isApprox(): raw coefficient comparison, NOT double-cover-aware -------
 
TEST(Quaternion, IsApproxTrueForEqualCoefficients) {
    const Quaternion a(1.0, 2.0, 3.0, 4.0);
    const Quaternion b(1.0, 2.0, 3.0, 4.0);
 
    EXPECT_TRUE(a.isApprox(b, kTol));
}
 
TEST(Quaternion, IsApproxFalseForDifferentCoefficients) {
    const Quaternion a(1.0, 0.0, 0.0, 0.0);
    const Quaternion b(0.9, 0.1, 0.0, 0.0);
 
    EXPECT_FALSE(a.isApprox(b, kTol));
}
 
TEST(Quaternion, IsApproxDoesNotTreatQAndMinusQAsEqual) {
    // Deliberately documents current, intended behavior: isApprox() is a
    // raw coefficient comparison. q and -q represent the same rotation
    // (double cover), but that is a DIFFERENT question, answered later by
    // a dedicated representsSameRotation()-style comparison once
    // quaternion -> DCM conversion exists. This test protects against
    // someone "fixing" isApprox() to silently special-case the sign.
    const Quaternion q(0.6, 0.2, -0.3, 0.7);
    const Quaternion negQ(-0.6, -0.2, 0.3, -0.7);
 
    EXPECT_FALSE(q.isApprox(negQ, kTol));
}

// --- fromAxisAngle() -------------------------------------------------------

TEST(Quaternion, FromAxisAngleIsAlwaysUnitNorm) {
    // True for any finite angle, since q0 = cos(angle / 2), qv=u*sin(angle/2)
    // with ||u|| = 1 gives cos^2 + sin^2 = 1 regardless of the angle's value.
    for (double angle : {0.0, 2.0 * M_PI, -3.5, 10.0}) {
        const Quaternion q = 
            Quaternion::fromAxisAngle(Eigen::Vector3d(1.0, 2.0, 3.0), angle);
        EXPECT_NEAR(q.norm(), 1.0, kTol) << "angle = " << angle;  
    }
}
 
TEST(Quaternion, FromAxisAngleDoesNotRequirePreNormalizedAxis) {
    const double angle = 0.9;
    const Quaternion q1 =
        Quaternion::fromAxisAngle(Eigen::Vector3d(0.0, 0.0, 2.0), angle);
    const Quaternion q2 =
        Quaternion::fromAxisAngle(Eigen::Vector3d(0.0, 0.0, 0.5), angle);
 
    EXPECT_TRUE(q1.isApprox(q2, kTol));
}
 
TEST(Quaternion, FromAxisAngleZeroAxisThrows) {
    EXPECT_THROW(
        Quaternion::fromAxisAngle(Eigen::Vector3d::Zero(), 1.0),
        std::invalid_argument);
}
 
// --- toRotationMatrix() ----------------------------------------------------
 
TEST(Quaternion, ToRotationMatrixOnNonUnitThrows) {
    const Quaternion q(1.0, 1.0, 0.0, 0.0); // norm = sqrt(2)
    ASSERT_FALSE(q.isUnit());
 
    EXPECT_THROW(q.toRotationMatrix(), std::invalid_argument);
}
 
// --- cross-check: Quaternion path vs. direct Rodrigues path ---------------
//
// The point of these tests: fromAxisAngle(...).toRotationMatrix() and
// axisAngleToRotationMatrix(...) are two independently-implemented routes to the same
// rotation matrix (one via quaternion sandwich product, one via Rodrigues'
// formula directly). Agreement between them is a much stronger check than
// either module's own internal tests, because a bug shared by both
// implementations wouldn't be caught by either alone -- these two were
// derived and coded independently, so the comparison is meaningful.
 
TEST(QuaternionRotationMatrixCrossCheck, Identity) {
    const Eigen::Vector3d axis(0.4, -1.1, 2.0); // direction irrelevant at angle 0
    const double angle = 0.0;
 
    const Eigen::Matrix3d viaQuaternion =
        Quaternion::fromAxisAngle(axis, angle).toRotationMatrix();
    const Eigen::Matrix3d viaRodrigues = axisAngleToRotationMatrix(axis, angle);
 
    EXPECT_TRUE(viaQuaternion.isApprox(viaRodrigues, kTol));
    EXPECT_TRUE(viaQuaternion.isApprox(Eigen::Matrix3d::Identity(), kTol));
}
 
TEST(QuaternionRotationMatrixCrossCheck, NinetyDegAboutX) {
    const Eigen::Vector3d axis(1.0, 0.0, 0.0);
    const double angle = M_PI / 2.0;
 
    const Eigen::Matrix3d viaQuaternion =
        Quaternion::fromAxisAngle(axis, angle).toRotationMatrix();
    const Eigen::Matrix3d viaRodrigues = axisAngleToRotationMatrix(axis, angle);
 
    EXPECT_TRUE(viaQuaternion.isApprox(viaRodrigues, kTol));
}
 
TEST(QuaternionRotationMatrixCrossCheck, NinetyDegAboutZ) {
    const Eigen::Vector3d axis(0.0, 0.0, 1.0);
    const double angle = M_PI / 2.0;
 
    const Eigen::Matrix3d viaQuaternion =
        Quaternion::fromAxisAngle(axis, angle).toRotationMatrix();
    const Eigen::Matrix3d viaRodrigues = axisAngleToRotationMatrix(axis, angle);
 
    EXPECT_TRUE(viaQuaternion.isApprox(viaRodrigues, kTol));
}
 
TEST(QuaternionRotationMatrixCrossCheck, ArbitraryNormalizedAxis) {
    Eigen::Vector3d axis(0.3, -0.6, 0.74);
    axis.normalize();
    const double angle = 1.234;
 
    const Eigen::Matrix3d viaQuaternion =
        Quaternion::fromAxisAngle(axis, angle).toRotationMatrix();
    const Eigen::Matrix3d viaRodrigues = axisAngleToRotationMatrix(axis, angle);
 
    EXPECT_TRUE(viaQuaternion.isApprox(viaRodrigues, kTol));
}
 
TEST(QuaternionRotationMatrixCrossCheck, QAndNegQGiveExactlySameRotationMatrix) {
    const Eigen::Vector3d axis(0.2, 0.5, 0.8);
    const double angle = 0.77;
 
    const Quaternion q = Quaternion::fromAxisAngle(axis, angle);
    const Quaternion negQ(-q.q0(), -q.q1(), -q.q2(), -q.q3());
 
    // Sanity: negQ is a different set of coefficients...
    EXPECT_FALSE(q.isApprox(negQ, kTol));
    // ...but represents the exact same rotation (double cover).
    EXPECT_TRUE(q.toRotationMatrix().isApprox(negQ.toRotationMatrix(), kTol));
 
    // And both still agree with the independent Rodrigues computation.
    const Eigen::Matrix3d viaRodrigues = axisAngleToRotationMatrix(axis, angle);
    EXPECT_TRUE(q.toRotationMatrix().isApprox(viaRodrigues, kTol));
    EXPECT_TRUE(negQ.toRotationMatrix().isApprox(viaRodrigues, kTol));
}

