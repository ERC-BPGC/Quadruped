% Quadruped quasi-static joint torque sizing.
%
% This script follows the FBD derivation documented in:
%   docs/mechanical/torque-calculations.md
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
