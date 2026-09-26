# Spacecraft Attitude Control - Roadmap

## Goal

Develop a C++ spacecraft Attitude Determination and Control System (ADCS) simulation, using Python for analysis and validation.

The project is also intended to serve as a long-term learning and portfolio project for GNC/AOCS, spacecraft dynamics, scientific C++, estimation and control.

## Development philosophy

Learn --> derive --> implement --> validate --> document --> commit.

The project should progress by small validated milestones rather than by adding large amounts of functionality at once.

Target workload: approximately 5-8 hours per week.

---

## Technical decisions

- C++ is the simulation core.
- Python is used mainly for validation, plotting, analysis and later Monte Carlo studies.
- Eigen is used for vectors and matrices. A custom Vector3 implementation will not be created.
- GoogleTest + CTest are used for unit testing.
- CMake is used for project configuration and builds.
- Attitude/frame conventions are documented explicitly in `docs/conventions.md`.
- Mathematical correctness and validation take priority over feature count.
- New C++ abstractions are added only when there is a real use case for them.

---

## Main phases

1. Attitude representations and SO(3)
2. Rigid-body attitude dynamics
3. Numerical propagation
4. Environment and sensor models
5. Attitude determination
6. State estimation
7. Actuators
8. Attitude control
9. Closed-loop ADCS simulation
10. Validation, Monte Carlo and documentation

---

## Learning resources

### Phase 1 - Attitude representations

Completed / used:

- Modern Robotics - Chapter 3
  - 3.2.1 Rotation Matrices
  - 3.2.2 Angular Velocities
  - 3.2.3 Exponential Coordinates of Rotation
- Hanspeter Schaub - Attitude Dynamics Fundamentals
  - Sections 1.1 through 1.2.4
  - Rotating frames
  - Direction cosine matrices
  - Euler angles
  - Principal rotation parameters
  - Quaternions / Euler parameters

### Phase 2+

Planned:

- University of Colorado Boulder - Spacecraft Dynamics and Control
- Markley & Crassidis - Fundamentals of Spacecraft Attitude Determination and Control
- MIT Underactuated Robotics - selected control topics later in the project
- Kalman and Bayesian Filters in Python - selected estimation topics later in the project

---

# Current status

## Milestone 0 - Foundations

**Status: COMPLETE**

Goal:

Build a mathematically and computationally consistent foundation for attitude representation.

Completed topics:

- [x] Reference frames
- [x] Right-handed coordinate systems
- [x] Passive vs active rotations
- [x] SO(3)
- [x] Rotation matrices
- [x] Direction Cosine Matrix convention
- [x] Angular velocity convention
- [x] Axis-angle representation
- [x] Skew-symmetric cross-product matrix
- [x] Rodrigues' formula
- [x] Quaternion convention
- [x] Hamilton product convention
- [x] Scalar-first component ordering
- [x] Relationship between frame transformations and active rotations
- [x] Double-cover property of quaternions

Documentation:

- [x] `docs/conventions.md`

Development environment:

- [x] WSL2 / Ubuntu
- [x] GCC / G++
- [x] CMake
- [x] Git / GitHub over SSH
- [x] VS Code
- [x] Eigen
- [x] GoogleTest
- [x] CTest

---

## Milestone 1 - SO(3) rotation utilities

**Status: COMPLETE**

Implemented:

- [x] `skew(v)`
- [x] `axisAngleToRotationMatrix(axis, angle)`
- [x] Axis normalization / invalid-axis handling
- [x] Separation between `.hpp` interface and `.cpp` implementation
- [x] `sac::attitude` namespace

Validated with unit tests:

- [x] Zero angle produces the identity matrix
- [x] `R^T R = I`
- [x] `det(R) = +1`
- [x] 90 degree rotation around z gives the expected result
- [x] Rotation preserves vector norm
- [x] Skew matrix reproduces the vector cross product
- [x] Non-normalized input axes are handled correctly

Build/test status:

- [x] `test_rotation`
- [x] CTest integration
- [x] Tests passing
- [x] Changes committed and pushed to GitHub

---

## Milestone 2 - Quaternion algebra

**Status: COMPLETE**

Design decisions:

- [x] Custom `Quaternion` class
- [x] Explicit scalar-first storage
- [x] Hamilton multiplication convention
- [x] Immutable/value-style API
- [x] Constructor does not force unit norm
- [x] General mathematical quaternions are allowed
- [x] No exact `operator==` for floating-point comparison
- [x] No addition/subtraction/scalar operators until a real use case appears

Implemented:

- [x] Identity quaternion
- [x] Component accessors
- [x] `norm()`
- [x] `squaredNorm()`
- [x] `isUnit()`
- [x] `conjugate()`
- [x] General `inverse()`
- [x] `normalized()`
- [x] Hamilton product
- [x] `isApprox()`

Validated with unit tests:

- [x] Identity quaternion
- [x] Norm / squared norm
- [x] Unit-quaternion detection
- [x] Conjugation
- [x] Normalization
- [x] General inverse
- [x] `q * q^-1 = identity`
- [x] Hamilton product
- [x] Non-commutativity of quaternion multiplication
- [x] Approximate coefficient comparison

Build/test status:

- [x] `test_quaternion`
- [x] `test_rotation`
- [x] 100% CTest pass rate
- [x] Changes committed / ready to remain part of main development history

---

# Current phase

## Milestone 3 - Attitude representation conversions

**Status: NEXT**

Goal:

Connect the mathematical attitude representations already implemented and prove that they are mutually consistent.

### Theory to consolidate

- Quaternion as principal rotation representation
- Quaternion <-> rotation matrix relationship
- Passive DCM vs active rotation matrix interpretation
- Double-cover property: `q` and `-q`
- Euler-angle sequence conventions
- Euler-angle singularities / gimbal lock

### Planned implementation

Quaternion-related:

- [ ] Quaternion --> active rotation matrix
- [ ] Active rotation matrix --> quaternion
- [ ] Axis-angle --> quaternion
- [ ] Quaternion --> axis-angle

Frame/attitude interpretation:

- [ ] Explicit conversion between active rotation matrix and passive DCM when needed
- [ ] Ensure all conversion functions follow `docs/conventions.md`

Euler angles:

- [ ] Choose and document the primary Euler sequence
- [ ] Euler angles --> rotation matrix
- [ ] Rotation matrix --> Euler angles
- [ ] Document and test the singular configuration

### Required validation

- [ ] `q --> R --> q` round trip
- [ ] `R --> q --> R` round trip
- [ ] `q` and `-q` produce the same rotation matrix
- [ ] Axis-angle and quaternion produce the same rotation matrix
- [ ] Known 90 degree rotations around principal axes
- [ ] Composition through quaternions matches composition through matrices
- [ ] Rotation matrices remain in SO(3)
- [ ] Euler-angle conversion reproduces known cases
- [ ] Gimbal-lock behavior is explicitly documented

### Important comparison rule

Quaternion coefficient equality and attitude equality are different concepts.

`isApprox()` compares quaternion coefficients only.

A future attitude-level comparison should account for:

`q ~ -q`

because both represent the same physical rotation.

---

## Milestone 4 - Attitude kinematics

**Status: PLANNED**

Theory:

- DCM differential kinematics
- Quaternion differential kinematics
- Angular velocity expressed in body and inertial frames
- Consistency with the passive `R_BI` convention

Expected implementation:

- [ ] Attitude state representation
- [ ] Quaternion derivative
- [ ] Constant-angular-velocity propagation
- [ ] Basic time integration interface

Validation:

- [ ] Analytical constant-rate rotation
- [ ] Quaternion norm preservation / renormalization behavior
- [ ] Agreement between quaternion and matrix propagation

---

## Milestone 5 - Rigid-body rotational dynamics

**Status: PLANNED**

Theory:

- Inertia tensor
- Principal axes
- Angular momentum
- Rotational kinetic energy
- Euler rigid-body equations

Expected implementation:

- [ ] Rigid-body parameters
- [ ] Angular-rate dynamics
- [ ] Torque-free propagation
- [ ] Coupled attitude + angular-rate state

Validation:

- [ ] Angular momentum conservation
- [ ] Rotational kinetic energy conservation
- [ ] Rotation about a principal axis
- [ ] Spherical-inertia special case

---

## Milestone 6 - Numerical propagation

**Status: PLANNED**

Topics:

- Euler integration
- RK4
- Step-size sensitivity
- Local/global numerical error
- Quaternion renormalization policy

Expected implementation:

- [ ] Euler integrator
- [ ] RK4 integrator
- [ ] Simulation loop
- [ ] CSV output for Python analysis

Validation:

- [ ] Numerical vs analytical solutions
- [ ] Error vs timestep
- [ ] Python plots

Planned release:

`v0.1.0 - attitude kinematics and rigid-body propagation`

---

# Later phases

## Environment and sensor models

Planned topics:

- Gravity-gradient torque
- Magnetic field / torque
- Gyroscope model
- Magnetometer model
- Sun sensor model
- Bias
- White noise
- Random walk

## Attitude determination

Planned topics:

- TRIAD
- Wahba's problem
- QUEST

## State estimation

Planned progression:

- Linear Kalman filter concepts
- Nonlinear estimation
- EKF
- Attitude-estimation architecture
- Possible later MEKF

## Actuators

Planned:

- Reaction wheels
- Magnetorquers
- Saturation
- Wheel momentum
- Possible momentum dumping

## Attitude control

Planned progression:

- Ideal torque control
- PD attitude control
- Quaternion attitude error
- Detumbling / B-dot
- Later LQR comparison

## Closed-loop ADCS simulation

Target scenarios:

1. Detumbling
2. Fine attitude pointing
3. Noisy-sensor estimation and control
4. Monte Carlo robustness analysis

---

# Portfolio target

The final repository should demonstrate that each major subsystem has been:

1. mathematically defined,
2. implemented,
3. independently validated,
4. integrated,
5. documented.

The repository should allow a technical reviewer to inspect:

- frame and attitude conventions,
- SO(3) implementation,
- quaternion algebra,
- analytical validation cases,
- rigid-body dynamics,
- sensor models,
- attitude determination / estimation,
- actuator models,
- controllers,
- closed-loop performance,
- Monte Carlo results.

---

# Immediate next task

Implement and validate the first attitude-representation conversions.

1. Quaternion --> active rotation matrix
2. Axis-angle --> quaternion
3. Verify both produce the same rotation
4. Quaternion --> axis-angle
5. Rotation matrix --> quaternion
6. Add double-cover-aware rotation-equivalence tests
7. Only then move to Euler-angle conversion

Do not start attitude dynamics yet.

The current priority is to make all attitude representations mathematically consistent with one another and with `docs/conventions.md`.
