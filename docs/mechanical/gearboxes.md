---
title: Gearboxes
---

# Gearboxes

Gearboxes sit between the motor and joint output. They trade motor speed for joint torque, and their design strongly affects backdrivability, impact tolerance, control bandwidth, efficiency, backlash, and cost.

This page is a placeholder for the gearbox choices relevant to our quadruped.

## Why gear reduction matters

Most compact BLDC motors spin quickly and produce modest torque at the shaft. A reduction stage increases output torque:

```text
output_torque = motor_torque * gear_ratio * efficiency
output_speed  = motor_speed / gear_ratio
```

A higher gear ratio makes torque easier to achieve, but it also increases reflected inertia and usually reduces backdrivability. For dynamic legs, this tradeoff matters because the robot must survive impacts and rapidly control contact forces.

## Common gearbox types

| Type | Strengths | Tradeoffs | Notes |
| --- | --- | --- | --- |
| Planetary gearbox | Compact, common, high torque density. | Backlash and reflected inertia increase with ratio. | Good general-purpose option if backdrivability is acceptable. |
| Cycloidal drive | High shock tolerance and compact high reduction. | More complex, can have efficiency/backlash challenges. | Popular in rugged robotic joints. |
| Harmonic drive | Very compact, high ratio, low backlash. | Expensive, less backdrivable, fatigue-sensitive flexspline. | Common in precision robotics, less ideal for low-cost impact-heavy legs. |
| Belt reduction | Simple, quiet, efficient, helps move mass proximally. | Belt tensioning, packaging, skipped teeth, wear. | Useful for knee transmission and low-ratio reductions. |
| Chain reduction | Robust and inexpensive. | Noisy, backlash, lubrication/wear, packaging. | Less common in compact quadruped legs. |
| Direct / quasi-direct drive | Minimal reduction, high backdrivability. | Needs high-torque motor and high current. | Attractive for dynamic legged robots. |

## Notes for our actuator selection

For this project, the gearbox decision should be tied to:

- required joint torque from the torque sizing page,
- desired joint speed,
- motor torque constant and current limit,
- backdrivability requirement,
- available packaging volume,
- expected impacts and falls,
- cost and manufacturability.

## TODO

- Add the gearbox or transmission used in our current CAD.
- Add ratio, efficiency assumption, backlash estimate, and torque rating.
- Add motor-side and joint-side speed/torque calculations.
- Compare candidate reductions against the joint torque sizing results.
