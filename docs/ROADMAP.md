# Spacecraft Attitude Control - Roadmap

## Goal

Develop a C++ spacecraft Attitude Determination and Control System (ADCS) simulation, uisng Python for analysis and validation.

## Development philosophy 

Learn --> derive --> implement --> validate --> document --> commit.

## Main phases

1. Attitude representions and SO(3)
2. Rigid-body attitude dynamics
3. Numerical propagation
4. Environment and sensor models
5. Attitude determination
6. State estimation
7. Actuators
8. Attitude control
9. Closed-loop ADCS simulation
10. Validation, Monte Carlo and documentation

## Learning resoruces 

### Phase 1 
- Modern Robotics - Chapter 2 (selected sections)
- Modern Robotics - Chapter 3
    - 3.2.1 Rotation Matrices
    - 3.2.2 Angular Velocities
    - 3.2.3 Exponential Coordinates of Rotation

### Phase 2+
- University of Colorado Boulder - Spacecraft Dynamics and Control
- Markley & Crassidis - Fundamentals of Spacecraft Attitude Determination and Control

## Current Milestones 

### Milestone 0 - Foundations


Goal:
Build a mathematically consistent foundation for attitude representation

Topics:
- Reference frames
- SO(3)
- Rotation matrices
- Angular velocity
- Axis-angle representations
- Rodrigue's formula
- Attitude conventions

Expected implementation:
- Basic rotation utiities
- Unit tests for SO(3) properties

## Next task

1. Study the selected sections of Modern Robotics Chapter 2
2. Study Chapter 3.2.1 --> 3.2.3
3. Complete 'docs/conventions.md'
4. Set up Eigen
5. Implement the first SO(3) utility: 'axisAngleToDCM(axis, angle)
6. Validate it with unit tests before adding more functionnality

### Initial validation cases

- Zero angle produces identity matrix
- R^T R = I
- det(R) = +1
- 90° rotation around z gives the expected result
- Rotation preserves vector norm
