# Spacecraft Attitude Control

A C++17 spacecraft **Attitude Determination and Control System (ADCS)** simulator, built from first principles, with Python for validation and analysis.

The goal is a complete closed-loop ADCS simulation (dynamics, sensors, estimation, actuators and control) where each subsystem is mathematically defined, implemented, independently validated and documented before being integrated.

> **Status:** early development. Attitude representations (SO(3), quaternion algebra) are implemented and unit-tested. Next up: attitude kinematics and rigid-body dynamics. See the [roadmap](docs/ROADMAP.md).

---

## Highlights

- **Explicit, documented conventions.** Frames, passive vs active rotations, Hamilton scalar-first quaternions, angular velocity and quaternion kinematics are all defined in [`docs/conventions.md`](docs/conventions.md). Every function follows that document.
- **Validation first.** Tests check mathematical properties (orthogonality, `det(R) = +1`, norm preservation, non-commutativity, double cover), not only hand-picked values.
- **Numerically verified signs.** The quaternion kinematics sign and multiplication side were checked by finite differences against `Ṙ_BI = -[ω_B]× R_BI` before any code depended on them.
- **Industry-style tooling.** Eigen, CMake, GoogleTest and CTest.

---

## Conventions at a glance

| Item | Convention |
|---|---|
| Frames | `I` inertial (fixed), `B` body |
| Attitude | Passive DCM `R_BI`: `v_B = R_BI v_I`, chained as `R_CI = R_CB R_BI` |
| Quaternion | Hamilton, scalar-first `q = [q0, q1, q2, q3]`, unit norm for attitudes |
| `q_BI` | `v_B = q_BI ⊗ v_I ⊗ q_BI*` (mirrors `R_BI`) |
| Angular velocity | `ω_B`: body rate relative to `I`, expressed in `B` (what a gyro measures) |
| Kinematics | `Ṙ_BI = -[ω_B]× R_BI`, `q̇_BI = -½ (0, ω_B) ⊗ q_BI` |
| Units | Radians internally; degrees only at user-facing I/O |

`q` and `-q` represent the same attitude. Tests and control laws account for this explicitly. Full details and derivations are in [`docs/conventions.md`](docs/conventions.md).

---

## Progress

| # | Milestone | Status |
|---|---|---|
| 0 | Foundations: frames, SO(3), conventions, toolchain | ✅ Complete |
| 1 | SO(3) utilities: `skew`, Rodrigues axis-angle → rotation matrix | ✅ Complete |
| 2 | Quaternion algebra: Hamilton product, conjugate, inverse, normalization | ✅ Complete |
| 3 | Representation conversions: quaternion ↔ rotation matrix, axis-angle, Euler angles | 🟡 In progress |
| 4 | Attitude kinematics: quaternion/DCM propagation | ⏳ Planned |
| 5 | Rigid-body dynamics: Euler equations, torque-free motion | ⏳ Planned |
| 6 | Numerical propagation: Euler, RK4, CSV output, Python plots | ⏳ Planned |
| — | Environment & sensors · TRIAD / QUEST · EKF / MEKF · reaction wheels & magnetorquers · PD / B-dot / LQR · closed-loop & Monte Carlo | ⏳ Later phases |

Milestone 3 so far: `Quaternion::fromAxisAngle` and `Quaternion::toRotationMatrix`, cross-checked against Rodrigues' formula (identity, 90° about principal axes, arbitrary axis, and `q` / `-q` giving the same matrix).

---

## Repository layout

```
include/sac/attitude/   Public headers (Rotation.hpp, Quaternion.hpp)
src/attitude/           Implementations
tests/attitude/         GoogleTest unit tests
docs/conventions.md     Mathematical and physical conventions
docs/ROADMAP.md         Milestones, validation plan and learning resources
```

---

## Build and test

**Requirements:** a C++17 compiler, CMake ≥ 3.20, Eigen 3 and GoogleTest.

On Ubuntu / WSL2:

```bash
sudo apt install build-essential cmake libeigen3-dev libgtest-dev

git clone https://github.com/almalkiyanis/spacecraft-attitude-control.git
cd spacecraft-attitude-control

cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

---

## References

- H. Schaub, *Attitude Dynamics Fundamentals*
- K. Lynch & F. Park, *Modern Robotics*, ch. 3
- F. L. Markley & J. L. Crassidis, *Fundamentals of Spacecraft Attitude Determination and Control* (upcoming phases)

---

## Author

**Yanis Al Malki**: 

- Physics graduate (Magistère de Physique Fondamentale, Université Paris-Saclay)

- MSc student in Space Science and Technology at Universidad de Alcalá, focusing on GNC/AOCS.
