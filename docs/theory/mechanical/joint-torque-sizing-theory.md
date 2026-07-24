---
title: Joint Torque Sizing Theory
---

# Joint Torque Sizing Theory

Before choosing motors and gearboxes, we need a first estimate of how much torque each joint must hold. This page documents the quasi-static force-balance method used as a starting point for actuator sizing.

The goal is not to perfectly predict running, jumping, impact, or control transients. The goal is to get an auditable torque estimate that future members can recompute when the robot mass, link lengths, or stance assumptions change.

## Analysis methods

| Method | What it assumes | Why it is useful |
| --- | --- | --- |
| Static analysis | The robot is stationary and all forces/moments balance. | Simple first check for standing poses and worst-case support loads. |
| Quasi-static analysis | The robot moves slowly enough that inertial effects are ignored at each instant. | Good for early actuator sizing because it captures pose-dependent torque without needing a full dynamics model. |
| Dynamic analysis | Link accelerations, body acceleration, impacts, and inertia are included. | Needed for aggressive locomotion, jumping, landing, and final validation. |
| Simulation/experiment | Multibody simulation or measured hardware data. | Best for validating controller behavior, contact loads, thermal limits, and real losses. |

For our first sizing pass, quasi-static analysis is a good compromise. It is easy to explain, easy to check by hand, and conservative enough when combined with a sensible safety factor.

## Model

We use a planar sagittal-plane model with two equal leg links. The thigh and calf have length `l`, and each link makes angle `theta` with the horizontal in the symmetric pose.

<figure className="figure--small">
  <img src={require('../../mechanical/assets/torque-sizing/overview.png').default} alt="Overview of the simplified two-link leg model used for quasi-static torque sizing." />
  <figcaption>Overview of the simplified two-link leg model. The stance foot provides a vertical normal reaction and a horizontal friction reaction; gravity acts on the body and link centers of mass.</figcaption>
</figure>

The height relation for this symmetric two-link geometry is:

$$
h = 2l\sin\theta
\qquad\Longrightarrow\qquad
\theta = \arcsin\left(\frac{h}{2l}\right)
$$

## Assumptions

- The robot is analyzed in the sagittal plane.
- Thigh and calf links have equal length `l`.
- The stance pose is symmetric, so both links use the same angle `theta`.
- Motion is quasi-static, so acceleration and impact forces are ignored.
- Loads are shared equally across `n` stance legs.
- The base mass is treated separately from the leg link masses.
- Foot friction is included in the FBD. For the symmetric stance case, the full robot has no external horizontal load, so the symmetric solution gives `F_friction = 0` for each stance foot.
- Transmission losses, bearing friction, motor heating, and gearbox efficiency are handled later using safety factors and motor datasheets.

## Parameters

| Symbol | Meaning |
| --- | --- |
| `m_base` | torso/base mass |
| `m_1` | thigh link mass |
| `m_2` | calf link mass |
| `m_leg` | mass of one leg, `m_1 + m_2` |
| `m_total` | total robot mass, `m_base + 4m_1 + 4m_2` |
| `n` | number of stance feet on the ground |
| `l` | thigh/calf link length |
| `g` | gravitational acceleration |
| `N` | vertical ground reaction at one stance foot |
| `N_1` | vertical load transmitted from torso/base into one stance leg |
| `N_3` | vertical knee reaction used in the thigh FBD |
| `N_4` | horizontal knee reaction used in the thigh FBD |

## Force Balance

For `n` stance legs, the vertical loads used in the FBD are:

$$
N_1 =
\frac{m_{\text{base}}g + m_{\text{leg}}(4-n)g}{n}
$$

$$
N_3 = N_1 + m_1g
$$

$$
N =
\frac{m_{\text{total}}g}{n}
$$

The foot friction term deserves a bit of care. If we isolate one leg, the horizontal equilibrium equations give:

$$
N_4 = F_{\text{friction}},
\qquad
N_2 = N_4
$$

So the single-leg equations tell us that the horizontal contact force propagates through the calf, knee, and hip reactions. They do not determine its magnitude by themselves.

To determine `F_friction`, we need the whole-robot horizontal equilibrium condition. In symmetric standing, there is no external horizontal load and every stance leg is assumed to carry the same horizontal reaction. Therefore:

$$
\sum F_x = 0
\qquad\Longrightarrow\qquad
nF_{\text{friction}} = 0
\qquad\Longrightarrow\qquad
F_{\text{friction}} = 0
$$

For the symmetric stance case:

$$
F_{\text{friction}} = 0,
\qquad
N_4 = F_{\text{friction}} = 0
$$

In a non-symmetric case, such as a body acceleration, slope, external push, or uneven contact distribution, `F_friction` should be solved from the full-body equilibrium/dynamics and checked against the friction limit:

$$
|F_{\text{friction}}| \leq \mu N
$$

## Thigh Link

<figure className="figure--small">
  <img src={require('../../mechanical/assets/torque-sizing/thigh.png').default} alt="Free-body diagram of the thigh link." />
  <figcaption>Thigh link free-body diagram. Taking moments about the hip removes the hip reaction forces from the torque equation.</figcaption>
</figure>

Taking moments about the hip joint:

$$
\tau_1
+ m_1g\cos\theta\frac{l}{2}
+ N_4\sin\theta l
= N_3\cos\theta l
$$

So the hip torque is:

$$
\tau_1 =
N_3\cos\theta l
- m_1g\cos\theta\frac{l}{2}
- N_4\sin\theta l
$$

For the symmetric stance case, `N_4 = 0`, so this reduces to:

$$
\tau_1 =
N_3\cos\theta l
- m_1g\cos\theta\frac{l}{2}
$$

## Calf Link

<figure className="figure--small">
  <img src={require('../../mechanical/assets/torque-sizing/calf.png').default} alt="Free-body diagram of the calf link." />
  <figcaption>Calf link free-body diagram. The knee reactions are equal and opposite to the reactions shown on the thigh FBD.</figcaption>
</figure>

Taking moments about the knee joint:

$$
\tau_2
+ m_2g\cos\theta\frac{l}{2}
+ F_{\text{friction}}\sin\theta l
= N\cos\theta l
$$

So the knee/calf torque is:

$$
\tau_2 =
N\cos\theta l
- m_2g\cos\theta\frac{l}{2}
- F_{\text{friction}}\sin\theta l
$$

For the symmetric stance case:

$$
\tau_2 =
N\cos\theta l
- m_2g\cos\theta\frac{l}{2}
$$

## MATLAB Workflow

The MATLAB script lives at:

```text
tools/torque_sizing/static_joint_torque_2d.m
```

It sweeps body height, computes `theta`, evaluates the hip and knee torque equations, and prints the maximum torque for different stance-leg counts.

```matlab
m_base = 17.5;
m1 = 1.55;
m2 = 0.25;
l = 0.3;
g = 9.81;
stance_cases = [4, 3, 2];
```

The useful sizing cases are:

| Case | Description | Why it matters |
| --- | --- | --- |
| `n = 4` | All four feet on the ground. | Baseline standing load. |
| `n = 3` | One leg lifted. | Slow walking or transition case. |
| `n = 2` | Two stance legs. | Conservative case for trot-like support. |

After MATLAB gives the maximum torque values, we apply a safety factor to account for modeling simplifications, impact loads, acceleration, and transmission losses.
