---
title: Torque Calculations
---

# Torque Calculations

This page documents the quasi-static torque calculation used for our robot. It contains the model assumptions, free-body diagrams, equations, and MATLAB script used to sweep body height and estimate hip/knee torque.

For the general background behind choosing static, quasi-static, or dynamic methods, see [Joint Torque Sizing Theory](../theory/mechanical/joint-torque-sizing-theory.md).

## Model

We use a planar sagittal-plane model with two equal leg links. The thigh and calf have length `l`, and each link makes angle `theta` with the horizontal in the symmetric pose.

<figure className="figure--small">
  <img src={require('./assets/torque-sizing/overview.png').default} alt="Overview of the simplified two-link leg model used for quasi-static torque sizing." />
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
  <img src={require('./assets/torque-sizing/thigh.png').default} alt="Free-body diagram of the thigh link." />
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
  <img src={require('./assets/torque-sizing/calf.png').default} alt="Free-body diagram of the calf link." />
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

## MATLAB Script

This script sweeps body height, computes `theta`, evaluates the hip and knee torque equations, and prints the maximum torque for different stance-leg counts.

```matlab
% Quadruped quasi-static joint torque sizing.
%
% Assumptions:
%   - 2D sagittal-plane analysis
%   - equal thigh/calf lengths
%   - symmetric stance geometry
%   - quasi-static motion
%   - equal load sharing across n stance legs
%   - no horizontal external load in the symmetric stance case

clear; clc; close all;

% -------------------------------------------------------------------------
% Robot parameters
% -------------------------------------------------------------------------
m_base = 17.5;   % [kg] base/torso mass
m1 = 1.55;       % [kg] thigh link mass
m2 = 0.25;       % [kg] calf link mass
l = 0.30;        % [m] thigh/calf link length
g = 9.81;        % [m/s^2] gravity

stance_cases = [4, 3, 2];

% -------------------------------------------------------------------------
% Derived parameters
% -------------------------------------------------------------------------
m_leg = m1 + m2;
m_total = m_base + 4*m1 + 4*m2;

% Avoid h = 0 and h = 2l singular/end-point cases.
h_min = 0.05;
h_max = 2*l*0.99;
h = linspace(h_min, h_max, 500);
theta = asin(h ./ (2*l));
h_cm = h * 100;

fprintf("\n========================================\n");
fprintf("  QUASI-STATIC TORQUE ANALYSIS SUMMARY\n");
fprintf("========================================\n");
fprintf("Base mass:             %.2f kg\n", m_base);
fprintf("Total mass:            %.2f kg\n", m_total);
fprintf("Thigh mass m1:         %.2f kg\n", m1);
fprintf("Calf mass m2:          %.2f kg\n", m2);
fprintf("Link length l:         %.3f m\n", l);
fprintf("Height range:          %.1f cm to %.1f cm\n", h_cm(1), h_cm(end));
fprintf("----------------------------------------\n");

figure("Name", "Quasi-static joint torque sizing", ...
       "Position", [100 100 1000 520]);

tiledlayout(1, 2);
nexttile;
hold on;
grid on;
xlabel("Body height h [cm]");
ylabel("Joint torque [Nm]");
title("Hip and knee torque vs height");

results = struct([]);

for i = 1:numel(stance_cases)
    n = stance_cases(i);

    % Vertical load transmitted from the torso/base into one stance leg.
    N1 = (m_base*g + m_leg*(4 - n)*g) / n;

    % Vertical ground reaction at one stance foot.
    N = m_total*g / n;

    % Vertical reaction at the knee from thigh-link force balance.
    N3 = N1 + m1*g;

    % Symmetric stance: no external horizontal load.
    F_friction = zeros(size(h));
    N4 = F_friction;

    tau_hip = N3 .* cos(theta) .* l ...
        - m1*g .* cos(theta) .* (l/2) ...
        - N4 .* sin(theta) .* l;

    tau_knee = N .* cos(theta) .* l ...
        - m2*g .* cos(theta) .* (l/2) ...
        - F_friction .* sin(theta) .* l;

    [max_hip, idx_hip] = max(tau_hip);
    [max_knee, idx_knee] = max(tau_knee);

    results(i).n = n;
    results(i).N1 = N1;
    results(i).N = N;
    results(i).N3 = N3;
    results(i).max_hip = max_hip;
    results(i).max_knee = max_knee;
    results(i).hip_height_cm = h_cm(idx_hip);
    results(i).knee_height_cm = h_cm(idx_knee);

    plot(h_cm, tau_hip, "LineWidth", 2, ...
        "DisplayName", sprintf("hip, n = %d", n));
    plot(h_cm, tau_knee, "--", "LineWidth", 2, ...
        "DisplayName", sprintf("knee, n = %d", n));

    fprintf("n = %d stance legs\n", n);
    fprintf("  N1 hip vertical load:   %.2f N\n", N1);
    fprintf("  N foot normal:          %.2f N\n", N);
    fprintf("  N3 knee vertical load:  %.2f N\n", N3);
    fprintf("  Max hip torque:         %.2f Nm at h = %.1f cm\n", ...
        max_hip, h_cm(idx_hip));
    fprintf("  Max knee torque:        %.2f Nm at h = %.1f cm\n", ...
        max_knee, h_cm(idx_knee));
    fprintf("----------------------------------------\n");
end

legend("Location", "best");

nexttile;
plot(h_cm, rad2deg(theta), "k", "LineWidth", 2);
grid on;
xlabel("Body height h [cm]");
ylabel("Link angle theta [deg]");
title("Geometry relation");

fprintf("Friction check for symmetric stance: F_friction = 0 N\n");
fprintf("For non-symmetric cases, solve F_friction and check |F| <= mu*N.\n");
fprintf("========================================\n\n");
```

## Sizing Cases

| Case | Description | Why it matters |
| --- | --- | --- |
| `n = 4` | All four feet on the ground. | Baseline standing load. |
| `n = 3` | One leg lifted. | Slow walking or transition case. |
| `n = 2` | Two stance legs. | Conservative case for trot-like support. |

After MATLAB gives the maximum torque values, we apply a safety factor to account for modeling simplifications, impact loads, acceleration, and transmission losses.
