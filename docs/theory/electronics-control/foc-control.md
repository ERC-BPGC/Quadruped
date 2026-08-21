---
title: FOC Control
---

# FOC Control

BLDC motors need electronic commutation: the controller must decide how to energize the stator phases so the rotor keeps producing useful torque. There are a few common ways to do this.

## Common Control Methods

| Method | Basic idea | Notes |
| --- | --- | --- |
| 6-step / trapezoidal commutation | Energize two phases at a time in six discrete steps as the rotor moves. | Simple, efficient enough for many applications, but torque ripple is higher and low-speed control is rougher. |
| Sinusoidal commutation | Drive the phases with smooth sinusoidal currents instead of discrete steps. | Smoother than 6-step, but still does not fully decouple torque and magnetic flux. |
| SVPWM | Space Vector PWM chooses inverter switching states to synthesize the desired voltage vector efficiently. | Often used inside modern motor controllers because it uses bus voltage well and produces smooth phase voltages. |
| FOC | Field-oriented control transforms phase currents into a rotating reference frame so torque-producing current can be controlled directly. | More complex, but gives smooth, accurate torque control across a wide speed range. |

Many hobby drone ESCs traditionally use sensorless trapezoidal/6-step-style commutation because drone motors mostly spin fast in one direction, where this works well. Some newer ESCs use more advanced sinusoidal or FOC-like methods, but the common mental model people know from drones is still: BLDC motor plus ESC, optimized for high-speed propeller control.

Legged robots need something different. A quadruped joint often moves slowly, reverses direction, gets hit by the ground, holds torque at near-zero speed, and needs predictable force output. This is where FOC becomes much more useful.

## Basic Idea Of FOC

FOC measures or estimates the rotor angle, then mathematically rotates the three phase currents into two useful current components:

- `d-axis current`: aligned with the rotor magnetic field,
- `q-axis current`: perpendicular to the rotor field and mainly responsible for torque.

For most permanent-magnet BLDC motors used in robot actuators, the controller tries to keep `d-axis current` near zero and control `q-axis current` to produce the desired torque. In simple terms:

```text
desired torque -> desired q-axis current -> phase currents -> motor torque
```

This makes the motor feel much more like a controllable torque source instead of just a spinning motor.

## Why FOC Is Ideal For Legged Robotics

FOC is more work than a basic ESC. It usually needs rotor position feedback or good sensorless estimation, current sensing, fast control loops, and careful tuning. The payoff is that it gives the kind of actuator behavior legged robots need:

- smooth torque at low speed,
- fast torque response,
- bidirectional control,
- better efficiency and lower torque ripple,
- current limiting for safety,
- torque/current control that can be wrapped inside velocity, position, impedance, or whole-body controllers.

This is why BLDC motors with FOC are commonly used to build robust, dynamic, "springy" robot actuators. Instead of commanding a stiff servo position and hoping the gearbox survives impacts, the controller can make the joint behave like a virtual spring-damper:

```text
joint torque = stiffness * position_error + damping * velocity_error
```

That kind of impedance behavior is extremely useful for legged robots because the feet constantly interact with the ground. Compared to steppers or hobby servos, a BLDC + FOC actuator can be more backdrivable, more impact tolerant, and better suited for dynamic force control.

## Video Resources

- [BLDC motor control methods and FOC explanation](https://www.youtube.com/watch?v=e0sQnVmE7DU)
- [Motor control playlist with MATLAB examples](https://www.youtube.com/watch?v=a5PWePUTR-0&list=PLl6mqZGq1o09k59iLGNs7AdLuJi9FoVcV&index=4)

For the MATLAB playlist, the goal for now is just to understand the control idea. There is no need to work through the MATLAB code yet.
