#pragma once

#include <Eigen/Dense>

namespace sac::attitude {


// Scalar first quaternion q = [q0, q1, q2, q3] = [q0, qv],
// Hamilton convention, consistent with docs/conventions.md
//
// Immutable: every operation returns a new Quaternion; nothing is mutated
// in place (mirrors how Rotation.cpp always returns Eigne::Matrix3d by
// value)
//
// The constructor does NOT enforce unit norm. A Quaternion can hold any
// 4-vector (useful for differences, intermediate interpolation results,
// etc.); operations that require unit norm as precondition document that 
// requirement instead of enforcing it silently.
//
// Deliberately out of scope for this module (see docs/conventions.md and
// project log): quatenrion <-> DCM conversion, quaternion kinematics
// (q_dot), and any double-cover-aware comparison ("same rotation" as
// opposed to "same coefficients"). Those depend on this module and come
// next; adding operator+/operator-/scalar multiplication is also deferred 
// until an actual use case (e.g. q_dot integration) needs them.

class Quaternion {
public:
    // --- construction ---------------------------------------------------

    // Raw component constructor, scalar-first: q0 is the scalar part.
    Quaternion(double q0, double q1, double q2, double q3);

    // From an Eigen::Vector4d qlready in [q0, q1, q2, q3] order.
    explicit Quaternion(const Eigen::Vector4d& q);

    // The multiplicative identity: [1, 0, 0, 0] (zero rotation).
    static Quaternion Identity();

    // --- accessors -------------------------------------------------------

    double q0() const; // scalar part
    double q1() const;
    double q2() const;
    double q3() const;

    // Vector part (q1, q2, q3), i.e. qv
    Eigen::Vector3d vec() const;

    // All components as [q0, q1, q2, q3]
    Eigen::Vector4d coeffs() const;

    // --- core operations --------------------------------------------------

    // q0^2 + q1^2 + q2^2 + q3^2. Cheaper than norm() when only used for
    // comparisons (avoids the sqrt); norm() is defined in terms of this.
    double squaredNorm() const;

    // Euclidean norm sqrt(squaredNorm()).
    double norm() const;

    // True if norm() is within 'tol' of 1.
    bool isUnit(double tol = 1e-9) const;

    // Returns q / norm(). Throws std::invalid_argument if norm() is
    // (numerically) zero - never returns NaNs silently.
    Quaternion normalized() const;

    // [q0, -qv]. Defined for any quaternion, unit or not.
    Quaternion conjugate() const;

    // General formula: q* / ||q||^2/ Defined for any non-zero quaternion,
    // not just unit ones (the class does not restrict itself to unit
    // quaternions, so this operation shouldn't either). For a unit
    // quaternion this reduces to conjugate(), i.e. q^{-1} = q*, but that
    // is a property to verify in tests, not something inverse() assumes.
    // Throws std::invalid_argument if norm() is (numerically) zero.
    Quaternion inverse() const;

    // Hamilton product: this ⊗ other. NOT commutative.
    Quaternion operator*(const Quaternion& other) const;
    
    // --- comparison ------------------------------------------------------

    // Tests whether two quaternion coefficient vectors are numerically
    // equal, componetwise, within 'tol'. This is a raw coefficient
    // comparison: it does NOT account for the double-cover property
    // (q and -q representing the same rotation). A separate,
    // rotation-aware comparison (e.g. representsSameRotation(...)) will be
    // added once quaternion --> DCM conversion exists; isApprox() answers
    // "are these the same 4-vector", not "are these the same attitude".
    bool isApprox(const Quaternion& other, double tol = 1e-9) const;

private:
    Eigen::Vector4d q_; // [q0, q1, q2, q3]

};



} // namespace sac::attitude



