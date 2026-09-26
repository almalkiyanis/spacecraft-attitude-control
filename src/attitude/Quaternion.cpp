#include "sac/attitude/Quaternion.hpp"

#include <cmath>
#include <stdexcept>

namespace sac::attitude {

namespace {
// Below this norm, a quaternion is treated numrically zero: no
// direction/scale can be recovered form it, so normalized()/inverse()
// cannot be defined. Mirrors kMinAxisNorm in Rotation.cpp.
constexpr double kMinNorm = 1e-10;
} // namespace

// --- construction ------------------------------------------------------------------

Quaternion::Quaternion(double q0, double q1, double q2, double q3)
    : q_(q0, q1, q2, q3) {}

Quaternion::Quaternion(const Eigen::Vector4d& q) : q_(q) {}

Quaternion Quaternion::Identity() {
    return Quaternion(1.0, 0.0, 0.0, 0.0);
}

// --- accessors ---------------------------------------------------------------------

double Quaternion::q0() const { return q_(0); }
double Quaternion::q1() const { return q_(1); }
double Quaternion::q2() const { return q_(2); }
double Quaternion::q3() const { return q_(3); }

Eigen::Vector3d  Quaternion::vec() const {
    return q_.tail<3>();
}

Eigen::Vector4d Quaternion::coeffs() const {
    return q_;
}

// --- core operations ----------------------------------------------------------------

double Quaternion::squaredNorm() const {
    return q_.squaredNorm();
}

double Quaternion::norm() const {
    return q_.norm();
}

bool Quaternion::isUnit(double tol) const {
    return std::abs(norm() - 1.0) <= tol;
}

Quaternion Quaternion::normalized() const {
    const double n = norm();
    if (n < kMinNorm) {
        throw std::invalid_argument(
            "Quaternion::normalized(): quaternion has (numerically) zero"
            "norm; cannot normalize.");
    } 
    return Quaternion(q_ / n);
}

Quaternion Quaternion::conjugate() const {
    //[q0, -qv] Defined for any quaternion, unit or not.
    return Quaternion(q0(), -q1(), -q2(), -q3());
}

Quaternion Quaternion::inverse() const {
    // General formula: q^{-1} = q* / ||q||^2. For a unit quaternion this
    // reduces to conjugate() (verified in tests), but the formula here
    // does not assume unit norm.
    const double n2 = squaredNorm();
    if (n2 < kMinNorm * kMinNorm) {
        throw std::invalid_argument(
            "Quaternion::inverse(): quaternion has (numerically) zero"
            "norm; no inverse exists.");
    }
    const Eigen::Vector4d conjCoeffs =
        conjugate().coeffs();

    return Quaternion(conjCoeffs / n2);    
}

Quaternion Quaternion::operator*(const Quaternion& other) const {
    //  Hamilton product: (a0, av)  ⊗ (b0, bv)
    //    = (a0*b0 - av.bv, a0*bv + b0.av + av x bv)
    const double a0 = q0();
    const Eigen::Vector3d av = vec();
    const double b0 = other.q0();
    const Eigen::Vector3d bv = other.vec();

    const double resultScalar = a0*b0 - av.dot(bv);
    const Eigen::Vector3d resultVec = a0*bv + b0*av + av.cross(bv);

    return Quaternion(resultScalar, resultVec.x(), resultVec.y(),
                        resultVec.z());
}

// --- comparison -------------------------------------------------------------------

bool Quaternion::isApprox(const Quaternion& other, double tol) const {
    // Raw coefficient comparison. Deliberately NOT double-cover-aware: see
    // the header doc comment and test_quaternion.cpp for why q and -q are
    // intentionally treated as different here.
    return (q_ - other.q_).norm() <= tol;
}

} // namespace sac::attitude