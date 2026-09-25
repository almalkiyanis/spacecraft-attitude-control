# Attitude Conventions

This document defines the mathematical and physical conventions used throughout the spacecraft attitude control simulator

## Reference frames

I : nominal inertial frame used by the simulator (fixed, non-rotating)
B : body frame (attached to the rigid-body, rotates with it)

## Coordinate system handedness

Right-handed, orthonormal (x,y,z), in either frame (I or B)

## Vector notation

A physical vector v has no "coordinates" of its own; what changes is its numerical representation depending on the frame. Notation : 

    v_I : coordinates of v expressed in the basis of frame I
    v_B : coordinates of v expressed in the basis of frame B

Both columns represent the same physical vector, not different vectors

## Rotation matrix notation

R_BI is the matrix that transforms the coordinates of a given vector, expressed in frame I, into its coordinates expressed in frame B : 

    v_B = R_BI * v_I

How to read the subscripts : "from I to B", cancelling right-to-left (the inner index "I" of R_BI cancels with the v_I on the right, leaving the outer index "B"). This convention allows rotations to be chained naturally : 

    R_CI = R_CB * R_BI

Direct consequence : R_IB = R_BI^{-1} = R_BI^T (orthogonal), and it transforms in the opposite direction v_I = R_IB * v_B

Geometric interpretation (useful as a sanity check when debugging) : 

    * The rows of R_BI are the axes of frame B, expressed in frame I
    * The columns of R_BI are the axes of frame I, expressed in frame B

This follows from (R_BI)_if = e_B^i ⋅ e_I^j for orthonormal bases: row i collects the I-frame components of e_B^i, and column j collects B-frame components of e_I^j

## Rotation convention

R_BI, as defined above, is a coordinate transformation (passive) : the physical vector does not move, only the basis in which it is expressed changes. It is the matrix that "carries" coordinates from frame I to frame B

This is distinct from an active rotation : a matrix R(u, theta) that actually rotates a vector within the same frame. Relationship between the two: if frame B is obtained by actively rotating the axes of I via R(u, theta), applying that rotation to the basis vectors of I, then : 

    R_BI = R(u, theta)^T = R(u, -theta)

This distinction is documented explicitly to avoid confusing "rotating the body" with "rotating the coordinate frame" : they are inverse operations of one another.

## Angular units

Radians throughout all code and internal documentation. Degrees are only allowed at the user-facing input/output boundary, never in internal calculations.

## Angular velocity convention

omega_B denotes the angular velocity of frame B relative to frame I, expressed in the body frame B. It is defined through the kinematic differential equation of R_BI : 

    R'_BI = - [omega_B]_x R_BI

where [omega_B]_x is the skew-symmetric matrix associated with omega_B. This form is consistent with the passive convention v_B = R_BI v_I above : omega_B is the rate at which the body basis rotates relative to the inertial basis, as seen from inside the body frame.

Equivalently, expressed in the inertial frame, using omega_I = R_IB omega_B : 

    R'_IB = [omega_I]_x R_IB

Notes :

    * omega_B is what a body fixed gyroscope measures directly;
        this is the reason it is chosen as the primary representation (rather than omega_I)
    *Unit : rad/s, consistent with the angle convention above
    * Sign check :a positive body angular velocity follows the 
        right-hand rule for the physical rotation of frame B relative to I. Because R_{BI} is a passive coordinate transformation, its matrix evolves with the opposite rotation sense.

## Quaternion convention

TO-DO

## Component ordering

TO-DO

## Multiplication convention

TO-DO

