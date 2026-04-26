---
title: Joint Torque Sizing
---

# Joint Torque Sizing

This page documents the static sizing method for estimating required joint torque. The first pass intentionally uses force balance and moment arms rather than full rigid-body dynamics.

The goal is not to predict every dynamic load. The goal is to produce a conservative actuator-sizing number that is easy to audit and easy for future members to recompute.

## Design case

A useful conservative case is:

- robot standing quasi-statically,
- only two legs in support,
- center of mass inside or near the two-foot support line,
- safety factor applied for modeling error, impact, acceleration, and uneven load sharing.

For pure vertical support with equal load sharing:

```text
total_support_force = mass * g
force_per_support_leg = total_support_force / number_of_support_legs
design_force_per_leg = force_per_support_leg * safety_factor
```

For the two-leg case:

```text
F_leg = mass * g / 2
F_design = F_leg * safety_factor
```

If the center of mass is not centered between the two support feet, solve vertical force balance plus moment balance to get the two individual foot forces.

## Static force balance for two feet

Assume two support feet A and B on level ground. Let their x-positions be `xA` and `xB`, and let the robot center of mass projection be at `xCOM`.

```text
FA + FB = mass * g
FA * (xCOM - xA) = FB * (xB - xCOM)
```

Solving:

```text
FA = mass * g * (xB - xCOM) / (xB - xA)
FB = mass * g * (xCOM - xA) / (xB - xA)
```

Then apply the safety factor to the larger of the two if we are doing worst-case sizing.

## Joint torque from moment arms

For a planar leg, the torque from a foot force about a joint is the moment of that force about the joint.

```text
tau_joint = r_x * F_z - r_z * F_x
```

Where:

- `r_x` is the horizontal distance from the joint to the foot,
- `r_z` is the vertical distance from the joint to the foot,
- `F_x` is the horizontal foot force on the robot,
- `F_z` is the vertical foot force on the robot.

For vertical-only static support:

```text
tau_joint = r_x * F_z
```

So the worst torque happens when the foot is far horizontally from the joint, not simply when the leg is longest.

## Hip and knee torque

For a serial two-link sagittal leg:

```text
hip_torque  = moment of foot force about hip pitch joint
knee_torque = moment of foot force about knee pitch joint
```

Using position vectors:

```text
tau_hip  = cross2D(foot_position - hip_position,  foot_force)
tau_knee = cross2D(foot_position - knee_position, foot_force)
```

Where:

```text
cross2D(r, F) = r_x * F_z - r_z * F_x
```

This is equivalent to the robotics statics relation:

```text
tau = J(q)' * F
```

Where `J(q)` is the foot-position Jacobian and `F` is the Cartesian force at the foot, expressed in the same frame.

## What this method ignores

This static calculation is a sizing baseline. It does not include:

- link masses,
- motor/transmission masses,
- acceleration during swing,
- impact loads at touchdown,
- friction limits,
- body angular acceleration,
- actuator thermal limits,
- transmission efficiency,
- linkage mechanical advantage variation,
- torque ripple or current limits.

The safety factor should cover some uncertainty, but dynamic validation is still required.

## MATLAB workflow

A starter MATLAB script lives at:

```text
tools/torque_sizing/static_joint_torque_2d.m
```

Fill in:

- robot mass,
- safety factor,
- hip, knee, and foot coordinates for candidate poses,
- support-leg count,
- optional horizontal force.

The script computes the design foot force and the corresponding hip/knee torque from the 2D moment-arm method.

## Cases to calculate for our robot

| Case | Description | Why it matters |
| --- | --- | --- |
| Neutral stand, four legs | Baseline all-feet support. | Sanity check and thermal hold estimate. |
| Neutral stand, two diagonal legs | Trot-like support case. | Conservative static gait case. |
| Forward/backward COM offset | COM closer to one support foot. | Uneven load sharing. |
| Crouched pose | Larger horizontal moment arms likely. | Often high knee/hip torque. |
| Maximum reachable stance | Foot near workspace edge. | Worst static moment arm. |
| Horizontal disturbance | Add `F_x` at foot. | Checks friction/contact robustness. |

## References

- MathWorks, [Compute Joint Torques to Balance an Endpoint Force and Moment](https://www.mathworks.com/help/robotics/ug/compute-joint-torques-to-balance-an-endpoint-force-and-moment.html).
- Iris Lab, [Statics - Modeling and Control of Robots](https://irislab.tech/course_robotics/lec14/statics.html).
- Robotics Wikibooks, [Serial Manipulator Statics](https://en.wikibooks.org/wiki/Robotics_Kinematics_and_Dynamics/Serial_Manipulator_Statics).
- Zhang et al., [Leg Locomotion Adaption for Quadruped Robots with Ground Compliance Estimation](https://pmc.ncbi.nlm.nih.gov/articles/PMC7528129/), for Jacobian use in quadruped leg force/torque transformations.
