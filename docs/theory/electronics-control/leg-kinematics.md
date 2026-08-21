---
title: Leg Kinematics
---

# Leg Kinematics

For legged robots, kinematics is mostly about converting between joint angles and foot position.

- **Forward kinematics:** given joint angles, find where the foot is.
- **Inverse kinematics:** given a desired foot position, find the joint angles needed to place the foot there.

Inverse kinematics, or IK, is especially important because gait planners usually think in terms of foot trajectories: lift the foot, move it forward, place it down, shift the body, and repeat. The motor controller does not directly understand "put the foot here"; it needs joint angle targets.

```text
desired foot position -> inverse kinematics -> joint angle targets -> motor control
```

## Simple 2DOF Planar Leg

Start with a simple planar leg with two links:

- thigh length `l_1`,
- calf length `l_2`,
- hip angle `q_1`,
- knee angle `q_2`,
- desired foot point `(x, z)` relative to the hip.

The distance from hip to foot is:

$$
r = \sqrt{x^2 + z^2}
$$

Using the law of cosines, the knee angle can be found from:

$$
\cos q_2 =
\frac{x^2 + z^2 - l_1^2 - l_2^2}{2l_1l_2}
$$

So:

$$
q_2 =
\cos^{-1}\left(
\frac{x^2 + z^2 - l_1^2 - l_2^2}{2l_1l_2}
\right)
$$

The hip angle can be found by splitting it into two angles:

$$
q_1 =
\tan^{-1}\left(\frac{z}{x}\right)
-
\tan^{-1}\left(
\frac{l_2\sin q_2}{l_1 + l_2\cos q_2}
\right)
$$

In code, use `atan2` instead of plain `atan` so the quadrant is handled correctly:

```text
q_1 = atan2(z, x) - atan2(l_2 sin(q_2), l_1 + l_2 cos(q_2))
```

Depending on the leg convention, knee direction, and whether `z` is positive upward or downward, these equations may need sign changes. Always match the equations to the CAD, motor direction, and encoder zero convention used on the robot.

## Reachability

Not every foot position is physically reachable. For a two-link leg:

$$
|l_1 - l_2| \leq r \leq l_1 + l_2
$$

If the desired foot point is outside this range, the leg cannot reach it. Real robots also have joint limits, linkage interference, body collisions, cable limits, and torque limits, so the usable workspace is smaller than the ideal mathematical workspace.

## Extending To 3DOF Legs

A 3DOF quadruped leg usually adds a hip abduction/adduction, or tilt, joint. A common approach is:

1. Use the tilt joint to rotate the desired foot point into the sagittal plane of the leg.
2. Solve the remaining 2DOF hip/knee IK in that plane.
3. Convert the resulting joint angles into motor commands using the robot's sign conventions and offsets.

The core idea is the same: choose a desired foot position, solve geometry for joint angles, then send those angles to the lower-level controllers.

## Practical Notes

- Define coordinate frames before writing equations.
- Use `atan2`, not plain `atan`.
- Clamp the cosine argument to `[-1, 1]` before calling `acos` to avoid numerical errors.
- Check joint limits after solving IK.
- Test simple poses first: foot straight down, foot forward, foot backward.
- Plot or animate the leg to catch sign mistakes.
- Make sure encoder zero offsets match the IK convention.

## Resources

- [Inverse Kinematics: Go2 Robot's Leg](https://observablehq.com/@christophe-yamahata/inverse-kinematics-go2-robot), an interactive Go2 leg IK reference.
- [Inverse kinematics video explanation](https://www.youtube.com/watch?v=cqpYpj8EaDY&t=91s)
