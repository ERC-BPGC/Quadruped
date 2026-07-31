---
title: Joint Torque Sizing Theory
---

# Joint Torque Sizing Theory

Before choosing motors and gearboxes, we need a first estimate of how much torque each joint must hold. The exact numbers depend on the robot geometry and mass distribution, but the analysis method is general.

The goal is not to perfectly predict running, jumping, impact, or control transients. The goal is to understand which kind of calculation is appropriate at each stage of the design.

## Analysis Methods

| Method | What it assumes | Why it is useful |
| --- | --- | --- |
| Static analysis | The robot is stationary and all forces/moments balance. | Simple first check for standing poses and worst-case support loads. |
| Quasi-static analysis | The robot moves slowly enough that inertial effects are ignored at each instant. | Good for early actuator sizing because it captures pose-dependent torque without needing a full dynamics model. |
| Dynamic analysis | Link accelerations, body acceleration, impacts, and inertia are included. | Needed for aggressive locomotion, jumping, landing, and final validation. |
| Simulation/experiment | Multibody simulation or measured hardware data. | Best for validating controller behavior, contact loads, thermal limits, and real losses. |

For early robot design, quasi-static analysis is usually the most useful starting point. It is easy to audit, easy to recompute when dimensions change, and gives a reasonable baseline before adding safety factors, motor limits, gearbox efficiency, and dynamic validation.

## Resources To Add

- Static equilibrium and free-body diagram videos.
- Robot statics / Jacobian transpose resources.
- Actuator sizing videos or articles.
- Examples from quadruped actuator papers.
