---
title: Motors
---

# Motors

After estimating joint torque requirements, the next question is: what motor can produce that torque and speed without overheating or making the leg hard to control?

## Common motor types

| Type | Basic idea | Pros | Cons |
| --- | --- | --- | --- |
| Brushed DC motor | Current through brushes commutates the rotor mechanically. | Simple, cheap, easy to drive. | Brush wear, lower efficiency, limited power density, electrical noise. |
| Servo motor | Motor plus gearbox, controller, and position feedback in one package. | Easy position control, simple interface, beginner-friendly. | Often low backdrivability, limited torque/speed transparency, hard to do high-performance force control. |
| Stepper motor | Moves in discrete steps using energized phases. | Simple position control without encoder in low-speed applications, high holding torque. | Inefficient at high speed, can skip steps, poor impact behavior, not ideal for dynamic legs. |
| BLDC motor | Electronic commutation drives a permanent-magnet rotor. | High efficiency, high power density, good torque control, long life. | Needs motor driver, sensing/estimation, current control, and more careful tuning. |

<div className="figure-grid figure-grid--four">
  <figure>
    <img src={require('../../mechanical/assets/motors/DC.jpeg').default} alt="Small brushed DC motor" />
    <figcaption>Brushed DC motor</figcaption>
  </figure>
  <figure>
    <img src={require('../../mechanical/assets/motors/servo.jpeg').default} alt="Small hobby servo motor" />
    <figcaption>Servo motor</figcaption>
  </figure>
  <figure>
    <img src={require('../../mechanical/assets/motors/stepper.jpeg').default} alt="Stepper motor" />
    <figcaption>Stepper motor</figcaption>
  </figure>
  <figure>
    <img src={require('../../mechanical/assets/motors/BLDC.png').default} alt="BLDC outrunner motor" />
    <figcaption>BLDC motor</figcaption>
  </figure>
</div>

## Why BLDCs are common in good quadrupeds

Modern electric quadrupeds usually need high torque density, fast torque response, and good efficiency. BLDC motors fit this well, especially when paired with low or moderate reduction transmissions.

Compared to servos or steppers, BLDC-based actuators can be:

- lighter for the same power,
- more efficient,
- better for torque/current control,
- more suitable for dynamic impacts,
- easier to make backdrivable with low-ratio transmissions.

This is why many research and commercial quadrupeds use custom BLDC actuator modules instead of hobby servos or steppers.

## Why low-KV pancake BLDCs are useful

Legged robots often use **low-KV pancake BLDC motors**: wide, flat outrunner motors with a relatively large diameter and low speed constant.

<figure className="figure--small">
  <img src={require('../../mechanical/assets/motors/pancake_BLDC.jpg').default} alt="Low-KV pancake BLDC motor" />
  <figcaption>Low-KV pancake BLDC motor: large diameter, short axial length, and high torque density for its package.</figcaption>
</figure>

They are useful because:

- **Low `Kv` usually means higher `Kt`**, so the motor produces more torque per ampere.
- **Large diameter gives a bigger magnetic moment arm**, which helps torque density.
- **Flat packaging works well inside compact joint modules.**
- **High pole count and outrunner geometry help produce usable torque at lower speeds.**
- **They pair well with low-ratio reductions**, keeping the actuator more backdrivable and impact tolerant.

The tradeoff is that they still need high-current motor drivers, good thermal design, and careful control.

## Key motor parameters

| Parameter | Why it matters |
| --- | --- |
| Peak torque | Short-duration torque for push-off, recovery, and disturbance handling. |
| Continuous torque | Torque the motor can hold thermally without overheating. |
| Torque constant `Kt` | Converts phase current into motor torque. |
| Velocity constant `Kv` | Relates motor speed to voltage; helps estimate speed limits. |
| Max current | Electrical limit from motor windings, driver, battery, and thermal design. |
| Phase resistance | Affects copper losses and heating: `P_loss = I^2 R`. |
| Rotor inertia | Affects reflected inertia and impact behavior. |
| Mass | Directly affects robot mass and leg inertia. |

## Basic sizing chain

```text
required_joint_torque -> transmission ratio -> required_motor_torque -> required_current
```

With a gearbox, belt, or linkage reduction:

```text
motor_torque = joint_torque / (reduction_ratio * efficiency)
motor_current = motor_torque / Kt
```

