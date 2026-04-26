% Static joint torque sizing for a planar quadruped leg.
%
% Frame convention:
%   x: forward
%   z: upward
%   hip, knee, and foot positions are [x; z] in meters.
%   footForce is the ground reaction force on the robot at the foot [Fx; Fz].

clear; clc;

g = 9.81;

% TODO: replace with measured robot mass and agreed safety factor.
robotMassKg = 15.0;
safetyFactor = 2.0;
supportLegs = 2;

nominalLegForce = robotMassKg * g / supportLegs;
designVerticalForce = nominalLegForce * safetyFactor;

% TODO: replace these with CAD/kinematics poses.
poses(1).name = "neutral_stand";
poses(1).hip = [0.0; 0.0];
poses(1).knee = [0.03; -0.16];
poses(1).foot = [0.08; -0.32];

poses(2).name = "crouched_forward_foot";
poses(2).hip = [0.0; 0.0];
poses(2).knee = [0.08; -0.12];
poses(2).foot = [0.18; -0.24];

% Optional horizontal contact force. Use zero for vertical-only sizing.
horizontalForce = 0.0;
footForce = [horizontalForce; designVerticalForce];

fprintf("Robot mass: %.3f kg\n", robotMassKg);
fprintf("Support legs: %d\n", supportLegs);
fprintf("Safety factor: %.3f\n", safetyFactor);
fprintf("Design foot force: Fx = %.3f N, Fz = %.3f N\n\n", footForce(1), footForce(2));

for i = 1:numel(poses)
    hip = poses(i).hip;
    knee = poses(i).knee;
    foot = poses(i).foot;

    tauHip = cross2d(foot - hip, footForce);
    tauKnee = cross2d(foot - knee, footForce);

    fprintf("%s\n", poses(i).name);
    fprintf("  hip torque:  %.3f Nm\n", tauHip);
    fprintf("  knee torque: %.3f Nm\n", tauKnee);
    fprintf("  abs max:     %.3f Nm\n\n", max(abs([tauHip, tauKnee])));
end

function moment = cross2d(r, force)
    moment = r(1) * force(2) - r(2) * force(1);
end
